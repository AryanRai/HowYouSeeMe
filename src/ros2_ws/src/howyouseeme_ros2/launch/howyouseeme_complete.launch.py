#!/usr/bin/env python3
"""
Complete HowYouSeeMe System Launch File
Launches Kinect bridge, processing pipeline, SLAM, and visualization
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz2 for visualization'
    )
    
    target_fps_arg = DeclareLaunchArgument(
        'target_fps',
        default_value='15',
        description='Target FPS for processing'
    )
    
    # HowYouSeeMe Processing Pipeline
    howyouseeme_pipeline = Node(
        package='howyouseeme_ros2',
        executable='kinect_subscriber',
        name='howyouseeme_subscriber',
        parameters=[{
            'target_fps': LaunchConfiguration('target_fps'),
            'enable_yolo': True,
            'yolo_model': 'yolo11n',
            'confidence_threshold': 0.6
        }],
        output='screen'
    )
    
    # Performance Monitor
    performance_monitor = Node(
        package='howyouseeme_ros2',
        executable='performance_monitor',
        name='performance_monitor',
        output='screen'
    )
    
    return LaunchDescription([
        use_rviz_arg,
        target_fps_arg,
        howyouseeme_pipeline,
        performance_monitor
    ])