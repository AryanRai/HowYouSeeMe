#!/usr/bin/env python3
"""
Detection-Only Launch File
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    howyouseeme_subscriber = Node(
        package='howyouseeme_ros2',
        executable='kinect_subscriber',
        name='howyouseeme_subscriber',
        output='screen'
    )
    
    return LaunchDescription([
        howyouseeme_subscriber
    ])
