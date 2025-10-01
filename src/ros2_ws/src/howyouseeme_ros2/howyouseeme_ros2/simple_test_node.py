#!/usr/bin/env python3
"""
Simple ROS2 Test Node for HowYouSeeMe
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class SimpleTestNode(Node):
    """Simple test node to verify ROS2 is working"""
    
    def __init__(self):
        super().__init__('howyouseeme_test')
        
        # Publisher
        self.publisher = self.create_publisher(String, '/howyouseeme/test', 10)
        
        # Timer
        self.timer = self.create_timer(1.0, self.publish_message)
        
        self.counter = 0
        self.get_logger().info("HowYouSeeMe Test Node started!")
    
    def publish_message(self):
        """Publish test message"""
        msg = String()
        msg.data = f"HowYouSeeMe ROS2 System Test - Count: {self.counter}"
        self.publisher.publish(msg)
        
        self.get_logger().info(f"Published: {msg.data}")
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SimpleTestNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down test node...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()