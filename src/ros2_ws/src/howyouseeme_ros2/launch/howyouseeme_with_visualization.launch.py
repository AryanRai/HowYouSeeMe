#!/usr/bin/env python3
"""
Complete HowYouSeeMe System with Visualization Launch File
Launches Kinect bridge, YOLO detection, SLAM, and RViz2 visualization
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )
    
    enable_yolo_arg = DeclareLaunchArgument(
        'enable_yolo',
        default_value='true',
        description='Enable YOLO object detection'
    )
    
    enable_slam_arg = DeclareLaunchArgument(
        'enable_slam',
        default_value='true',
        description='Enable SLAM interface'
    )
    
    # Main HowYouSeeMe subscriber
    howyouseeme_subscriber = Node(
        package='howyouseeme_ros2',
        executable='kinect_subscriber',
        name='howyouseeme_subscriber',
        output='screen'
    )
    
    # YOLO Detector Node
    yolo_detector = Node(
        package='howyouseeme_ros2',
        executable='yolo_detector_node',
        name='yolo_detector',
        condition=IfCondition(LaunchConfiguration('enable_yolo')),
        output='screen'
    )
    
    # SLAM Interface Node
    slam_interface = Node(
        package='howyouseeme_ros2',
        executable='slam_interface_node',
        name='slam_interface',
        condition=IfCondition(LaunchConfiguration('enable_slam')),
        output='screen'
    )
    
    # Performance Monitor
    performance_monitor = Node(
        package='howyouseeme_ros2',
        executable='performance_monitor',
        name='performance_monitor',
        output='screen'
    )
    
    # Static TF for Kinect
    kinect_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='kinect2_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'kinect2_link']
    )
    
    # Map to odom transform
    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # RViz2 Visualization
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('howyouseeme_ros2'),
            'config',
            'howyouseeme_visualization.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )
    
    return LaunchDescription([
        use_rviz_arg,
        enable_yolo_arg,
        enable_slam_arg,
        howyouseeme_subscriber,
        yolo_detector,
        slam_interface,
        performance_monitor,
        kinect_tf,
        map_odom_tf,
        rviz2
    ])