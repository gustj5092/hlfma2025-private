from setuptools import find_packages, setup

package_name = 'camera_perception_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hhk',
    maintainer_email='whaihong@g.skku.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher_node = camera_perception_pkg.image_publisher_node:main',
            'yolov8_node = camera_perception_pkg.yolov8_node:main',
            'yolov8_traffic_light_node = camera_perception_pkg.yolov8_traffic_light_node:main',
            'traffic_light_detector_node = camera_perception_pkg.traffic_light_detector_node:main',
            'lane_info_extractor_node = camera_perception_pkg.lane_info_extractor_node:main',
            'lane_extract = camera_perception_pkg.lane_extract:main',
            'roi_merger_node = camera_perception_pkg.roi_merger_node:main',
            'traffic_light_yolov8_node = camera_perception_pkg.traffic_light_yolov8_node:main',
            'object_detector_node = camera_perception_pkg.object_detector_node:main'

        ],
    },
)

#entry_points에서 정의된 각 노드 맨끝에 :main은 노드의 main함수를 의미함
