#!/usr/bin/env python3
"""
Detection-Only Launch File
Launches just the Kinect bridge and YOLOv12 detection
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    yolo_model_arg = DeclareLaunchArgument(
        'yolo_model',
        default_value='yolo11n',
        description='YOLO model to use (yolo11n, yolo11s, yolo11m, etc.)'
    )
    
    confidence_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.6',
        description='Confidence threshold for detections'
    )
    
    # HowYouSeeMe Subscriber (basic mode)
    howyouseeme_subscriber = Node(
        package='howyouseeme_ros2',
        executable='kinect_subscriber',
        name='howyouseeme_subscriber',
        parameters=[{
            'yolo_model': LaunchConfiguration('yolo_model'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        yolo_model_arg,
        confidence_arg,
        howyouseeme_subscriber
    ])