#!/usr/bin/env python3
"""
HowYouSeeMe Memory System Launch File
Launches Tier 1-3 memory nodes (Tier 5 MCP server runs separately)
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Get user home directory
    home_dir = os.path.expanduser('~')
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('checkpoint_dir', default_value='/tmp/stm'),
        DeclareLaunchArgument('max_checkpoints', default_value='200'),
        DeclareLaunchArgument('synthesis_interval', default_value='3.0'),
        DeclareLaunchArgument('persistence_path', 
                            default_value=f'{home_dir}/howyouseeme_persistent/named_memories.json'),
        
        # Tier 1 — Event Checkpointer (always on)
        Node(
            package='kinect2_slam',
            executable='event_checkpointer',
            name='event_checkpointer',
            output='screen',
            parameters=[{
                'checkpoint_dir': LaunchConfiguration('checkpoint_dir'),
                'max_checkpoints': LaunchConfiguration('max_checkpoints'),
                'person_always_save': True,
                'min_confidence': 0.85,
                'cooldown_seconds': 5.0
            }]
        ),
        
        # Tier 2 — Async Analyser (event-driven background worker)
        Node(
            package='kinect2_slam',
            executable='async_analyser',
            name='async_analyser',
            output='screen',
            parameters=[{
                'checkpoint_dir': LaunchConfiguration('checkpoint_dir'),
                'max_workers': 2
            }]
        ),
        
        # Tier 3 — World Synthesiser
        Node(
            package='kinect2_slam',
            executable='world_synthesiser',
            name='world_synthesiser',
            output='screen',
            parameters=[{
                'world_state_path': '/tmp/world_state.json',
                'synthesis_interval': LaunchConfiguration('synthesis_interval'),
                'object_timeout': 60.0,
                'recent_events_window': 300.0,
                'fx': 1081.37,
                'fy': 1081.37,
                'cx': 959.5,
                'cy': 539.5
            }]
        ),
        
        # Tier 3 — Named Memory Store
        Node(
            package='kinect2_slam',
            executable='named_memory',
            name='named_memory',
            output='screen',
            parameters=[{
                'persistence_path': LaunchConfiguration('persistence_path')
            }]
        ),
    ])
