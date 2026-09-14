import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'teleop_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hs',
    maintainer_email='gustj5092@yonsei.ac.kr',
    description='F710 joystick teleop + manual/auto mux',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_mux_node = teleop_pkg.teleop_mux_node:main',
        ],
    },
)
