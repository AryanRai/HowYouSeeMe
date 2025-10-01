#!/usr/bin/env python3
"""
HowYouSeeMe ROS2 Kinect Subscriber Node

Main processing node that subscribes to kinect2_bridge topics and coordinates
SLAM and object detection processing.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
import sys
import os

# Try to import HowYouSeeMe components
COMPONENTS_AVAILABLE = False
try:
    from howyouseeme_ros2.perception.ros2_sensor_interface import ROS2KinectInterface
    from howyouseeme_ros2.perception.slam.ros2_slam_interface import ROS2SLAMInterface
    from howyouseeme_ros2.perception.detection.ros2_object_detector import ROS2YOLODetector
    COMPONENTS_AVAILABLE = True
except ImportError as e:
    print(f"Warning: HowYouSeeMe components not available: {e}")

class HowYouSeeMeSubscriber(Node):
    """Main HowYouSeeMe processing node"""
    
    def __init__(self):
        super().__init__('howyouseeme_subscriber')
        
        if COMPONENTS_AVAILABLE:
            self.get_logger().info("Starting HowYouSeeMe ROS2 System with full components...")
            
            # Initialize components
            self.kinect_interface = ROS2KinectInterface()
            self.slam_interface = ROS2SLAMInterface()
            self.detector = ROS2YOLODetector()
            
            self.get_logger().info("HowYouSeeMe ROS2 System initialized successfully!")
        else:
            self.get_logger().info("HowYouSeeMe ROS2 Subscriber initialized in basic mode (no SLAM/detection)")
            
            # Basic subscriber for testing
            self.image_sub = self.create_subscription(
                Image,
                '/kinect2/hd/image_color',
                self._image_callback,
                QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=2
                )
            )
            
            self.status_pub = self.create_publisher(String, '/howyouseeme/status', 10)
            self.timer = self.create_timer(5.0, self._publish_status)
            
            self.frame_count = 0
    
    def _image_callback(self, msg):
        """Basic image callback for testing"""
        self.frame_count += 1
        if self.frame_count % 30 == 0:  # Log every 30 frames
            self.get_logger().info(f"Received frame {self.frame_count}")
    
    def _publish_status(self):
        """Publish system status"""
        status_msg = String()
        status_msg.data = f"HowYouSeeMe running - frames: {self.frame_count}"
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = HowYouSeeMeSubscriber()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down HowYouSeeMe...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()