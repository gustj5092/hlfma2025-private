import os
import csv
import datetime
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import Joy
from interfaces_pkg.msg import MotionCommand

#---------------Variable Setting---------------
SUB_JOY_TOPIC_NAME = "joy"
SUB_AUTO_TOPIC_NAME = "topic_control_signal_auto"   # 자율주행 노드(motion_planner 등)의 출력
PUB_TOPIC_NAME = "topic_control_signal"             # serial_sender_node가 구독하는 토픽

# F710 (뒷면 스위치 X 모드) 기준 인덱스 - joy_node로 실측
AXIS_STEER = 0          # 왼쪽 스틱 좌우 (왼쪽 = +1)
AXIS_THROTTLE = 4       # 오른쪽 스틱 상하 (위 = +1)
BTN_MANUAL_DEADMAN = 4  # LB: 누르고 있는 동안만 수동 주행
BTN_AUTO_DEADMAN = 5    # RB: 누르고 있는 동안만 자율주행 명령 통과
BTN_LOG_TOGGLE = 3      # Y: CSV 로깅 on/off
BTN_MODE_TOGGLE = 7     # Start: MANUAL <-> AUTO

DEFAULT_LOG_DIR = os.path.expanduser(
    '~/hlfma2025/src/camera_perception_pkg/camera_perception_pkg/lib/Collected_Datasets')
#----------------------------------------------

MANUAL = 'MANUAL'
AUTO = 'AUTO'


class TeleopMuxNode(Node):
  def __init__(self):
    super().__init__('teleop_mux_node')

    self.sub_joy_topic = self.declare_parameter('sub_joy_topic', SUB_JOY_TOPIC_NAME).value
    self.sub_auto_topic = self.declare_parameter('sub_auto_topic', SUB_AUTO_TOPIC_NAME).value
    self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value

    self.max_steering = self.declare_parameter('max_steering', 7).value
    self.max_speed = min(255, self.declare_parameter('max_speed', 100).value)  # 처음엔 낮게 두고 올릴 것
    # 스틱 입력 곡선: 1.0 = 직선, 2.0 = 스틱 절반에서 속도 1/4 (저속 구간이 넓어져 덜 민감)
    self.throttle_expo = self.declare_parameter('throttle_expo', 2.0).value
    self.rate_hz = self.declare_parameter('rate_hz', 20.0).value
    self.joy_timeout = self.declare_parameter('joy_timeout', 0.5).value    # 이 시간 동안 /joy 없으면 정지
    self.auto_timeout = self.declare_parameter('auto_timeout', 0.5).value  # 자율주행 명령이 끊기면 정지
    self.log_dir = self.declare_parameter('log_dir', DEFAULT_LOG_DIR).value

    self.axis_steer = self.declare_parameter('axis_steer', AXIS_STEER).value
    self.axis_throttle = self.declare_parameter('axis_throttle', AXIS_THROTTLE).value
    self.btn_manual_deadman = self.declare_parameter('btn_manual_deadman', BTN_MANUAL_DEADMAN).value
    self.btn_auto_deadman = self.declare_parameter('btn_auto_deadman', BTN_AUTO_DEADMAN).value
    self.btn_log_toggle = self.declare_parameter('btn_log_toggle', BTN_LOG_TOGGLE).value
    self.btn_mode_toggle = self.declare_parameter('btn_mode_toggle', BTN_MODE_TOGGLE).value

    # serial_sender_node가 RELIABLE로 구독하므로 RELIABLE로 발행
    pub_qos = QoSProfile(
      reliability=QoSReliabilityPolicy.RELIABLE,
      history=QoSHistoryPolicy.KEEP_LAST,
      durability=QoSDurabilityPolicy.VOLATILE,
      depth=1
    )
    # BEST_EFFORT 구독은 RELIABLE/BEST_EFFORT 발행자 모두와 연결됨 (motion_planner는 BEST_EFFORT)
    sub_qos = QoSProfile(
      reliability=QoSReliabilityPolicy.BEST_EFFORT,
      history=QoSHistoryPolicy.KEEP_LAST,
      durability=QoSDurabilityPolicy.VOLATILE,
      depth=1
    )

    self.joy_sub = self.create_subscription(Joy, self.sub_joy_topic, self.joy_callback, sub_qos)
    self.auto_sub = self.create_subscription(MotionCommand, self.sub_auto_topic, self.auto_callback, sub_qos)
    self.publisher = self.create_publisher(MotionCommand, self.pub_topic, pub_qos)

    self.mode = MANUAL
    self.joy = None
    self.joy_time = None
    self.prev_buttons = []
    self.auto_cmd = None
    self.auto_time = None
    self.last_output = None

    self.csv_file = None
    self.csv_writer = None

    self.timer = self.create_timer(1.0 / self.rate_hz, self.timer_callback)
    self.get_logger().info(
      f'mode={self.mode} | LB: 수동주행, RB: 자율주행 통과, Start: 모드전환, Y: 로깅 | max_speed={self.max_speed}')

  def joy_callback(self, msg):
    buttons = list(msg.buttons)
    if self.prev_buttons:
      if self.rising_edge(buttons, self.btn_mode_toggle):
        self.mode = AUTO if self.mode == MANUAL else MANUAL
        self.get_logger().info(f'mode -> {self.mode}')
      if self.rising_edge(buttons, self.btn_log_toggle):
        self.toggle_logging()
    self.prev_buttons = buttons
    self.joy = msg
    self.joy_time = self.get_clock().now()

  def auto_callback(self, msg):
    self.auto_cmd = msg
    self.auto_time = self.get_clock().now()

  def rising_edge(self, buttons, idx):
    return idx < len(buttons) and buttons[idx] == 1 and self.prev_buttons[idx] == 0

  def pressed(self, idx):
    return idx < len(self.joy.buttons) and self.joy.buttons[idx] == 1

  def age(self, stamp):
    return (self.get_clock().now() - stamp).nanoseconds / 1e9

  def compute_command(self):
    """(steering, left_speed, right_speed, 상태설명) 반환. 조건이 하나라도 안 맞으면 정지."""
    if self.joy is None or self.age(self.joy_time) > self.joy_timeout:
      return 0, 0, 0, 'NO_JOY'

    if self.mode == MANUAL:
      if not self.pressed(self.btn_manual_deadman):
        return 0, 0, 0, 'MANUAL(LB off)'
      # 스틱 왼쪽이 +인데 차량 조향은 왼쪽이 - 이므로 부호 반전
      steering = int(round(-self.joy.axes[self.axis_steer] * self.max_steering))
      throttle = self.joy.axes[self.axis_throttle]
      throttle = (1 if throttle >= 0 else -1) * abs(throttle) ** self.throttle_expo
      speed = int(round(throttle * self.max_speed))
      return steering, speed, speed, 'MANUAL'

    if not self.pressed(self.btn_auto_deadman):
      return 0, 0, 0, 'AUTO(RB off)'
    if self.auto_cmd is None or self.age(self.auto_time) > self.auto_timeout:
      return 0, 0, 0, 'AUTO(no cmd)'
    steering = max(-self.max_steering, min(self.max_steering, self.auto_cmd.steering))
    left = max(-self.max_speed, min(self.max_speed, self.auto_cmd.left_speed))
    right = max(-self.max_speed, min(self.max_speed, self.auto_cmd.right_speed))
    return steering, left, right, 'AUTO'

  def timer_callback(self):
    steering, left, right, state = self.compute_command()
    self.publish(steering, left, right)

    if self.csv_writer is not None:
      self.csv_writer.writerow([f'{self.get_clock().now().nanoseconds / 1e9:.3f}', state, steering, left, right])

    output = (steering, left, right, state)
    if output != self.last_output:
      self.get_logger().info(f'[{state}] s{steering} l{left} r{right}')
      self.last_output = output

  def publish(self, steering, left, right):
    msg = MotionCommand()
    msg.steering = int(steering)
    msg.left_speed = int(left)
    msg.right_speed = int(right)
    self.publisher.publish(msg)

  def toggle_logging(self):
    if self.csv_file is None:
      session_path = os.path.join(self.log_dir, datetime.datetime.now().strftime('%Y_%m_%d_%H%M%S'))
      os.makedirs(session_path, exist_ok=True)
      self.csv_file = open(os.path.join(session_path, 'driving_log.csv'), 'w', newline='')
      self.csv_writer = csv.writer(self.csv_file)
      self.csv_writer.writerow(['time', 'state', 'steering', 'left_speed', 'right_speed'])
      self.get_logger().info(f'logging ON -> {session_path}')
    else:
      self.close_log()
      self.get_logger().info('logging OFF')

  def close_log(self):
    if self.csv_file is not None:
      self.csv_file.close()
      self.csv_file = None
      self.csv_writer = None


def main(args=None):
  rclpy.init(args=args)
  node = TeleopMuxNode()
  try:
    rclpy.spin(node)
  except (KeyboardInterrupt, ExternalShutdownException):
    pass
  finally:
    # Ctrl+C면 context가 이미 닫혀 발행 불가 -> 정지는 serial_sender_node의 timeout이 담당
    if rclpy.ok():
      node.publish(0, 0, 0)
    node.close_log()
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()
