#!/usr/bin/env python3
"""
YOLOv12 Detector ROS2 Node
"""

import rclpy
import sys
import os

# Add the src directory to Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

try:
    from perception.detection.ros2_object_detector import ROS2YOLODetector
except ImportError as e:
    print(f"Import error: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        detector = ROS2YOLODetector()
        rclpy.spin(detector)
    except KeyboardInterrupt:
        print("Shutting down YOLO detector...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()