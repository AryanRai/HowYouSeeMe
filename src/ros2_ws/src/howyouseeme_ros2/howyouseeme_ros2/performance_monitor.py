#!/usr/bin/env python3
"""
Performance Monitor ROS2 Node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time

class PerformanceMonitor(Node):
    """Monitor system performance and publish metrics"""
    
    def __init__(self):
        super().__init__('performance_monitor')
        
        # Publishers
        self.stats_pub = self.create_publisher(
            Float32MultiArray,
            '/howyouseeme/system_stats',
            10
        )
        
        # Timer for publishing stats
        self.timer = self.create_timer(5.0, self.publish_stats)
        
        self.get_logger().info("Performance Monitor started")
    
    def publish_stats(self):
        """Publish system performance statistics"""
        stats_msg = Float32MultiArray()
        stats_msg.data = [
            float(time.time()),  # Timestamp
            0.0,  # CPU usage (placeholder)
            0.0,  # Memory usage (placeholder)
            0.0,  # GPU usage (placeholder)
        ]
        
        self.stats_pub.publish(stats_msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        monitor = PerformanceMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("Shutting down performance monitor...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()