#!/usr/bin/env python3
"""
SLAM Interface ROS2 Node
"""

import rclpy
import sys
import os

# Add the src directory to Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

try:
    from perception.slam.ros2_slam_interface import ROS2SLAMInterface
except ImportError as e:
    print(f"Import error: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        slam_interface = ROS2SLAMInterface()
        rclpy.spin(slam_interface)
    except KeyboardInterrupt:
        print("Shutting down SLAM interface...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()