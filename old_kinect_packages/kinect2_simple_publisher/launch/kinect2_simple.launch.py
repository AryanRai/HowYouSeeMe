#!/usr/bin/env python3
"""
Launch file for simple Kinect v2 publisher
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'base_name',
            default_value='kinect2',
            description='Base name for topics'
        ),
        DeclareLaunchArgument(
            'fps_limit',
            default_value='30.0',
            description='FPS limit'
        ),
        DeclareLaunchArgument(
            'use_opengl',
            default_value='true',
            description='Use OpenGL pipeline (false for CPU)'
        ),
        
        Node(
            package='kinect2_simple_publisher',
            executable='kinect2_simple_publisher_node',
            name='kinect2_simple_publisher',
            output='screen',
            parameters=[{
                'base_name': LaunchConfiguration('base_name'),
                'fps_limit': LaunchConfiguration('fps_limit'),
                'use_opengl': LaunchConfiguration('use_opengl'),
            }]
        ),
    ])
