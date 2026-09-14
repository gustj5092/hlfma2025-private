import time                                   # 지연(sleep) 등 시간 관련 함수 사용
import serial                                 # PySerial: 시리얼 포트 통신
import rclpy                                  # ROS 2 Python 클라이언트 라이브러리
from rclpy.node import Node                   # ROS 2 노드 기본 클래스
from rclpy.executors import ExternalShutdownException  # CTRL+C 시 spin에서 발생
from rclpy.qos import QoSProfile              # QoS 설정 객체
from rclpy.qos import QoSHistoryPolicy        # QoS: 히스토리 정책
from rclpy.qos import QoSDurabilityPolicy     # QoS: 내구성(지속성) 정책
from rclpy.qos import QoSReliabilityPolicy    # QoS: 신뢰성 정책
from interfaces_pkg.msg import MotionCommand  # 사용자 정의 메시지(조향/좌우 속도)
from .lib import protocol_convert_func_lib as PCFL  # 시리얼 프로토콜 변환 유틸(문자열 생성)

#---------------Variable Setting---------------
# Subscribe할 토픽 이름
SUB_TOPIC_NAME = "topic_control_signal"       # 구독할 토픽명(제어 명령 수신)

# 아두이노 장치 이름 (ls /dev/ttyA* 명령을 터미널 창에 입력하여 확인)
PORT = '/dev/ttyUSB0'                         # 시리얼 포트 경로(아두이노가 잡힌 디바이스)
#----------------------------------------------

STOP_MESSAGE = PCFL.convert_serial_message(0, 0, 0)

class SerialSenderNode(Node):                 # ROS 2 노드: 토픽 구독 → 시리얼 송신
  def __init__(self, sub_topic=SUB_TOPIC_NAME):
    super().__init__('serial_sender_node')    # 노드 이름 등록

    self.declare_parameter('sub_topic', sub_topic)  # 파라미터 선언(토픽명 커스터마이즈용)

    # 파라미터에서 실제 구독할 토픽명 읽기
    self.sub_topic = self.get_parameter('sub_topic').get_parameter_value().string_value

    # QoS 설정: 신뢰성 높음(REL), 최근 1개만 유지(KEEP_LAST, depth=1),揮発성(재시작 시 과거 메시지X)
    qos_profile = QoSProfile(
      reliability=QoSReliabilityPolicy.RELIABLE,
      history=QoSHistoryPolicy.KEEP_LAST,
      durability=QoSDurabilityPolicy.VOLATILE,
      depth=1
    )

    # 시리얼 포트 오픈. 모터 노이즈 등으로 USB가 끊기면 watchdog에서 재연결
    self.ser = None
    self.last_open_try = 0.0
    self.last_open_error = None
    self.open_serial()

    # 토픽 구독 생성: MotionCommand 메시지를 수신하면 data_callback 호출
    self.subscription = self.create_subscription(
      MotionCommand, self.sub_topic, self.data_callback, qos_profile
    )

    # 아두이노는 명령이 끊겨도 마지막 속도를 유지하므로, 여기서 끊김을 감지해 정지 명령 송신
    self.cmd_timeout = self.declare_parameter('cmd_timeout', 0.5).value  # 초
    self.last_msg_time = None
    self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)


  def open_serial(self):                      # 성공 시 True
    self.last_open_try = time.time()
    try:
      self.ser = serial.Serial(PORT, 115200, timeout=1)  # 115200bps, 읽기 타임아웃 1초
    except (serial.SerialException, OSError) as e:
      self.ser = None
      if str(e) != self.last_open_error:      # 같은 에러는 한 번만 출력 (Permission denied면 dialout 그룹 확인)
        self.get_logger().error(f'{PORT} 열기 실패: {e} - 연결될 때까지 재시도')
        self.last_open_error = str(e)
      return False
    self.last_open_error = None
    time.sleep(1.5)                           # 포트 열 때 보드가 리셋되므로 부팅 대기
    return True


  def send(self, message):                    # 시리얼 송신. 실패하면 포트를 닫고 재연결 대기
    if self.ser is None:
      return
    try:
      self.ser.write(message.encode())
    except (serial.SerialException, OSError) as e:
      self.get_logger().error(f'시리얼 송신 실패({e}) - 재연결 시도')
      self.close_serial()


  def close_serial(self):
    if self.ser is not None:
      try:
        self.ser.close()
      except (serial.SerialException, OSError):
        pass
      self.ser = None


  def watchdog_callback(self):
    if self.ser is None:                      # 끊긴 상태면 0.5초마다 재연결 시도
      if time.time() - self.last_open_try > 0.5 and self.open_serial():
        self.get_logger().info(f'{PORT} 재연결됨')
      return
    # 마지막 명령 이후 cmd_timeout 초과 시 정지
    if self.last_msg_time is None:
      return
    if (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9 > self.cmd_timeout:
      self.send(STOP_MESSAGE)


  def data_callback(self, msg):               # 수신 콜백: 메시지를 시리얼 포맷으로 바꿔 전송
    self.last_msg_time = self.get_clock().now()
    steering = msg.steering                   # 조향 값
    left_speed = msg.left_speed               # 좌측 속도
    right_speed = msg.right_speed             # 우측 속도

    serial_msg = PCFL.convert_serial_message( # 시리얼 전송용 문자열 생성(프로토콜에 맞춤)
      steering, left_speed, right_speed
    )
    self.send(serial_msg)                     # 바이트로 인코딩하여 시리얼로 송신


def main(args=None):
  rclpy.init(args=args)                       # ROS 2 초기화
  node = SerialSenderNode()                   # 노드 인스턴스 생성
  try:
      rclpy.spin(node)                        # 콜백 처리 루프(CTRL+C까지 대기/실행)

  except (KeyboardInterrupt, ExternalShutdownException):  # 사용자가 중단(CTRL+C) 시 (humble은 후자로 올라옴)
      print("\n\nshutdown\n\n")
      node.send(STOP_MESSAGE)                 # 안전 정지 명령 송신

  finally:
    node.close_serial()                       # 시리얼 포트 닫기(자원 정리)
    print('closed')

  node.destroy_node()                         # 노드 파괴(정리)
  if rclpy.ok():
    rclpy.shutdown()                          # ROS 2 종료


if __name__ == '__main__':
  main()                                      # 스크립트 직접 실행 시 main() 진입
