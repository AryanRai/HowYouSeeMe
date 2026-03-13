#!/usr/bin/env python3
"""
Complete System Launch: Kinect + IMU + ORB-SLAM3 + TSDF
Launches all components needed for Phase 2 & 3
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Paths
    orb_slam3_root = os.path.expanduser('~/ORB_SLAM3')
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))))
    
    voc_file = os.path.join(orb_slam3_root, 'Vocabulary', 'ORBvoc.txt')
    settings_file = os.path.join(workspace_root, 'orb_slam3_configs', 'Kinect2_RGBD_IMU.yaml')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    voxel_length_arg = DeclareLaunchArgument(
        'voxel_length',
        default_value='0.04',
        description='TSDF voxel size (0.02=high detail, 0.04=balanced, 0.08=coarse)'
    )
    
    # 1. Kinect v2 Bridge - launch directly as in working script
    kinect_bridge = ExecuteProcess(
        cmd=['ros2', 'launch', 'kinect2_bridge', 'kinect2_bridge_launch.yaml'],
        output='screen',
        name='kinect2_bridge'
    )
    
    # 2. BlueLily IMU Node
    bluelily_imu = Node(
        package='bluelily_bridge',
        executable='bluelily_imu_node',
        name='bluelily_imu',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'port': '/dev/ttyACM0',
            'baud_rate': 115200,
            'frame_id': 'bluelily_imu',
        }]
    )
    
    # Static TF: kinect2_link -> bluelily_imu (10cm behind camera)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='kinect_to_imu_tf',
        arguments=['-0.1', '0', '0', '0', '0', '0', 'kinect2_link', 'bluelily_imu']
    )
    
    # 3. ORB-SLAM3 RGB-D+IMU node
    orb_slam3_node = Node(
        package='kinect2_slam',
        executable='orb_slam3_node',
        name='orb_slam3',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'voc_file': voc_file,
            'settings_file': settings_file,
        }],
        remappings=[
            ('/camera/rgb/image_raw', '/kinect2/hd/image_color'),
            ('/camera/depth_registered/image_raw', '/kinect2/hd/image_depth_rect'),
            ('/imu', '/imu/data'),
        ]
    )
    
    # 4. TSDF Integrator node
    tsdf_node = Node(
        package='kinect2_slam',
        executable='tsdf_integrator',
        name='tsdf_integrator',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'voxel_length': LaunchConfiguration('voxel_length'),
            'sdf_trunc': 0.08,
            'publish_rate': 1.0,
            'export_path': '/tmp/tsdf_mesh.ply',
            'fx': 1081.37,
            'fy': 1081.37,
            'cx': 960.0,
            'cy': 540.0,
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        voxel_length_arg,
        kinect_bridge,
        bluelily_imu,
        static_tf,
        orb_slam3_node,
        tsdf_node,
    ])
