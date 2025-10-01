from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'howyouseeme_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.rviz')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aryan Rai',
    maintainer_email='buzzaryanrai@gmail.com',
    description='HowYouSeeMe ROS2 Computer Vision System',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinect_subscriber = howyouseeme_ros2.kinect_subscriber:main',
            'yolo_detector_node = howyouseeme_ros2.yolo_detector_node:main',
            'slam_interface_node = howyouseeme_ros2.slam_interface_node:main',
            'performance_monitor = howyouseeme_ros2.performance_monitor:main',
            'simple_test_node = howyouseeme_ros2.simple_test_node:main',
        ],
    },
)