#!/usr/bin/env python3
"""
Simple Kinect v2 ROS2 Publisher using libfreenect2 directly
This bypasses the kinect2_bridge C++ issues and publishes directly to ROS2
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import sys
import ctypes
import os

# Add libfreenect2 library path
os.environ['LD_LIBRARY_PATH'] = '/home/aryan/Documents/GitHub/HowYouSeeMe/libfreenect2/freenect2/lib:' + os.environ.get('LD_LIBRARY_PATH', '')

try:
    from pylibfreenect2 import Freenect2, SyncMultiFrameListener
    from pylibfreenect2 import FrameType, Registration, Frame
    from pylibfreenect2 import createConsoleLogger, setGlobalLogger
    from pylibfreenect2 import LoggerLevel
    PYLIBFREENECT2_AVAILABLE = True
except ImportError:
    PYLIBFREENECT2_AVAILABLE = False
    print("pylibfreenect2 not available, will try ctypes approach")


class SimpleKinectPublisher(Node):
    def __init__(self):
        super().__init__('simple_kinect_publisher')
        
        self.bridge = CvBridge()
        
        # Publishers for SD (512x424) resolution - most reliable
        self.rgb_pub = self.create_publisher(Image, '/kinect2/sd/image_color', 10)
        self.depth_pub = self.create_publisher(Image, '/kinect2/sd/image_depth', 10)
        self.ir_pub = self.create_publisher(Image, '/kinect2/sd/image_ir', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/kinect2/sd/camera_info', 10)
        
        self.get_logger().info('Simple Kinect Publisher initialized')
        self.get_logger().info('Publishing to /kinect2/sd/* topics')
        
        # Try to initialize Kinect
        if not self.init_kinect():
            self.get_logger().error('Failed to initialize Kinect')
            return
        
        # Create timer for publishing
        self.timer = self.create_timer(0.033, self.publish_frames)  # ~30 Hz
        self.frame_count = 0
        
    def init_kinect(self):
        """Initialize Kinect v2 device"""
        if not PYLIBFREENECT2_AVAILABLE:
            self.get_logger().error('pylibfreenect2 not available')
            self.get_logger().info('Install with: pip install pylibfreenect2')
            return False
        
        try:
            # Set up logging
            logger = createConsoleLogger(LoggerLevel.Warning)
            setGlobalLogger(logger)
            
            # Initialize Freenect2
            self.fn = Freenect2()
            num_devices = self.fn.enumerateDevices()
            
            if num_devices == 0:
                self.get_logger().error('No Kinect devices found')
                return False
            
            self.serial = self.fn.getDeviceSerialNumber(0)
            self.get_logger().info(f'Found Kinect with serial: {self.serial}')
            
            # Open device with CPU pipeline (most compatible)
            from pylibfreenect2 import OpenGLPacketPipeline, CpuPacketPipeline
            
            try:
                pipeline = CpuPacketPipeline()
                self.get_logger().info('Using CPU pipeline')
            except:
                self.get_logger().warn('CPU pipeline failed, trying OpenGL')
                pipeline = OpenGLPacketPipeline()
            
            self.device = self.fn.openDevice(self.serial, pipeline=pipeline)
            
            # Set up frame listener
            self.listener = SyncMultiFrameListener(
                FrameType.Color | FrameType.Ir | FrameType.Depth
            )
            
            self.device.setColorFrameListener(self.listener)
            self.device.setIrAndDepthFrameListener(self.listener)
            
            # Start device
            self.device.start()
            
            self.get_logger().info('Kinect started successfully')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Kinect: {e}')
            return False
    
    def publish_frames(self):
        """Capture and publish frames"""
        try:
            frames = self.listener.waitForNewFrame()
            
            # Get frames
            color = frames["color"]
            ir = frames["ir"]
            depth = frames["depth"]
            
            # Create header with timestamp
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'kinect2_link'
            
            # Publish color (RGBX -> RGB)
            color_array = color.asarray()[:, :, :3]  # Remove alpha channel
            color_msg = self.bridge.cv2_to_imgmsg(color_array, encoding='rgb8')
            color_msg.header = header
            self.rgb_pub.publish(color_msg)
            
            # Publish IR
            ir_array = ir.asarray()
            ir_msg = self.bridge.cv2_to_imgmsg(ir_array, encoding='32FC1')
            ir_msg.header = header
            self.ir_pub.publish(ir_msg)
            
            # Publish depth
            depth_array = depth.asarray()
            depth_msg = self.bridge.cv2_to_imgmsg(depth_array, encoding='32FC1')
            depth_msg.header = header
            self.depth_pub.publish(depth_msg)
            
            # Publish camera info
            camera_info = CameraInfo()
            camera_info.header = header
            camera_info.height = 424
            camera_info.width = 512
            self.camera_info_pub.publish(camera_info)
            
            # Release frames
            self.listener.release(frames)
            
            self.frame_count += 1
            if self.frame_count % 150 == 0:
                self.get_logger().info(f'Published {self.frame_count} frames')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing frames: {e}')
    
    def destroy_node(self):
        """Clean up"""
        try:
            if hasattr(self, 'device'):
                self.device.stop()
                self.device.close()
            self.get_logger().info('Kinect stopped')
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleKinectPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
