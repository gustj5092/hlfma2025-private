import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file to run the lane detection pipeline on a pre-recorded video.
    
    This launch file starts three nodes:
    1. image_publisher_node: Publishes frames from a video file.
    2. yolov8_node: Performs object detection on the video frames.
    3. lane_info_extractor_node: Extracts lane information and publishes the bird's-eye view ROI.
    """
    
    # --- Define paths to resource files ---
    # It's good practice to locate files relative to the workspace root
    ws_root = os.path.expanduser('~/ros2_ws')
    
    # Path to the video file
    video_path = os.path.join(ws_root, 'src/camera_perception_pkg/camera_perception_pkg/lib/Collected_Datasets/driving_simulation.mp4')
    
    # Path to the YOLO model file
    model_path = os.path.join(ws_root, 'src/camera_perception_pkg/models/best.pt')

    # --- Node Definitions ---

    # 1. Image Publisher Node to play the video
    image_publisher_node = Node(
        package='camera_perception_pkg',
        executable='image_publisher_node',
        name='image_publisher_node',
        output='screen',
        parameters=[
            {'data_source': 'video'},
            {'video_path': video_path}
        ]
    )

    # 2. YOLOv8 Node for object detection
    yolov8_node = Node(
        package='camera_perception_pkg',
        executable='yolov8_node',
        name='yolov8_node',
        output='screen',
        parameters=[
            {'model': model_path}
        ],
        remappings=[
            ('image_raw', 'image_raw'),
            ('detections', 'detections')
        ]
    )

    # 3. Lane Info Extractor Node to get the bird's-eye view
    lane_info_extractor_node = Node(
        package='camera_perception_pkg',
        executable='lane_info_extractor_node',
        name='lane_info_extractor_node',
        output='screen',
        parameters=[
            {'sub_detection_topic': 'detections'},
            {'roi_pub_topic': 'roi_image'}
        ]
    )

    return LaunchDescription([
        image_publisher_node,
        yolov8_node,
        lane_info_extractor_node,
    ])