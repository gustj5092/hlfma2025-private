from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[
                {'deadzone': 0.05},
                # 스틱을 안 움직여도 계속 발행해야 teleop_mux_node의 끊김 감지가 동작함
                {'autorepeat_rate': 20.0},
            ]
        ),
        Node(
            package='teleop_pkg',
            executable='teleop_mux_node',
            name='teleop_mux_node',
            output='screen',
            parameters=[
                {'max_speed': 100},
            ]
        ),
        Node(
            package='serial_communication_pkg',
            executable='serial_sender_node',
            name='serial_sender_node',
            output='screen'
        ),
        # 자율주행 붙일 때: 출력 토픽만 auto 쪽으로 돌려서 추가
        # Node(
        #     package='decision_making_pkg',
        #     executable='motion_planner_node',
        #     name='motion_planner_node',
        #     output='screen',
        #     parameters=[{'pub_topic': 'topic_control_signal_auto'}]
        # ),
    ])
