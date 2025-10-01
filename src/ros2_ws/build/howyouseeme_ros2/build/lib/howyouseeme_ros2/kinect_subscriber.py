#!/usr/bin/env python3
"""
HowYouSeeMe ROS2 Kinect Subscriber
Subscribes to kinect2_bridge topics and processes with our SLAM/detection pipeline
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys
import os

# Add our source path - adjust this path as needed
current_dir = os.path.dirname(os.path.abspath(__file__))
howyouseeme_src = os.path.join(current_dir, '../../../../src')
if os.path.exists(howyouseeme_src):
    sys.path.append(howyouseeme_src)

try:
    from perception.slam.slam_interface import BasicSLAM
    from perception.detection.object_detector import YOLODetector, ResourceManager
    COMPONENTS_AVAILABLE = True
except ImportError as e:
    print(f"Warning: HowYouSeeMe components not available: {e}")
    COMPONENTS_AVAILABLE = False

class HowYouSeeMeSubscriber(Node):
    def __init__(self):
        super().__init__('howyouseeme_subscriber')
        
        # ROS2 setup
        self.bridge = CvBridge()
        
        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image, '/kinect2/hd/image_color', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/kinect2/hd/image_depth_rect', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/kinect2/hd/camera_info', self.camera_info_callback, 10)
        
        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/howyouseeme/pose', 10)
        
        # Data storage
        self.latest_rgb = None
        self.latest_depth = None
        self.camera_info = None
        
        # Initialize our components
        self.slam = None
        self.detector = None
        
        if COMPONENTS_AVAILABLE:
            self.resource_manager = ResourceManager(max_gpu_memory_gb=4.0)
        
        # Processing intervals (optimized from our tests)
        self.frame_count = 0
        self.slam_interval = 2      # Process SLAM every 2nd frame
        self.detection_interval = 10 # Process detection every 10th frame
        
        # Performance tracking
        self.start_time = self.get_clock().now()
        
        status = "with full pipeline" if COMPONENTS_AVAILABLE else "in basic mode (no SLAM/detection)"
        self.get_logger().info(f'HowYouSeeMe ROS2 Subscriber initialized {status}')
    
    def camera_info_callback(self, msg):
        """Store camera info for SLAM initialization"""
        if self.camera_info is None:
            self.camera_info = msg
            if COMPONENTS_AVAILABLE:
                self.initialize_components()
    
    def initialize_components(self):
        """Initialize SLAM and object detection"""
        if self.camera_info is None or not COMPONENTS_AVAILABLE:
            return
        
        try:
            # Extract camera intrinsics
            K = np.array(self.camera_info.k).reshape(3, 3)
            rgb_intrinsics = {
                'fx': K[0, 0],
                'fy': K[1, 1], 
                'cx': K[0, 2],
                'cy': K[1, 2],
                'width': self.camera_info.width,
                'height': self.camera_info.height
            }
            
            # Initialize SLAM
            self.slam = BasicSLAM(rgb_intrinsics)
            self.get_logger().info('SLAM initialized')
            
            # Initialize object detector
            self.detector = YOLODetector(
                model_name="yolov5n",  # Fast nano model
                confidence_threshold=0.6,
                resource_manager=self.resource_manager
            )
            self.get_logger().info('Object detector initialized')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize components: {e}')
    
    def rgb_callback(self, msg):
        """Process RGB image"""
        try:
            # Convert ROS image to OpenCV
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            
            # Process if we have both RGB and depth
            if self.latest_depth is not None:
                self.process_frames()
                
        except Exception as e:
            self.get_logger().error(f'RGB callback error: {e}')
    
    def depth_callback(self, msg):
        """Process depth image"""
        try:
            # Convert ROS depth image to OpenCV (16-bit to float)
            depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            self.latest_depth = depth_raw.astype(np.float32) / 1000.0  # Convert mm to meters
            
        except Exception as e:
            self.get_logger().error(f'Depth callback error: {e}')
    
    def process_frames(self):
        """Process RGB-D frames with our pipeline"""
        if self.latest_rgb is None or self.latest_depth is None:
            return
        
        self.frame_count += 1
        
        # Basic processing (always available)
        rgb_shape = self.latest_rgb.shape
        depth_shape = self.latest_depth.shape
        
        # Advanced processing (if components available)
        slam_result = None
        detections = []
        
        if COMPONENTS_AVAILABLE and self.slam is not None and self.detector is not None:
            # SLAM processing (every slam_interval frames)
            if self.frame_count % self.slam_interval == 0:
                slam_result = self.slam.process_frame(self.latest_rgb, self.latest_depth)
                
                # Publish pose if tracking
                if slam_result and slam_result.get('pose') is not None:
                    self.publish_pose(slam_result['pose'])
            
            # Object detection (every detection_interval frames)
            if self.frame_count % self.detection_interval == 0:
                detections = self.detector.detect_objects(self.latest_rgb)
        
        # Log progress
        if self.frame_count % 150 == 0:  # Every ~10 seconds at 15 FPS
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            fps = self.frame_count / elapsed
            
            if COMPONENTS_AVAILABLE and slam_result:
                slam_status = 'OK' if slam_result.get('is_tracking') else 'LOST'
                self.get_logger().info(
                    f'Frame {self.frame_count}: {fps:.1f} FPS, '
                    f'SLAM: {slam_status}, Objects: {len(detections)}'
                )
            else:
                self.get_logger().info(
                    f'Frame {self.frame_count}: {fps:.1f} FPS, '
                    f'RGB: {rgb_shape}, Depth: {depth_shape}'
                )
    
    def publish_pose(self, pose_matrix):
        """Publish SLAM pose to ROS2"""
        try:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            
            # Extract position and orientation from 4x4 matrix
            pose_msg.pose.position.x = float(pose_matrix[0, 3])
            pose_msg.pose.position.y = float(pose_matrix[1, 3])
            pose_msg.pose.position.z = float(pose_matrix[2, 3])
            
            # Convert rotation matrix to quaternion (simplified)
            pose_msg.pose.orientation.w = 1.0
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            
            self.pose_pub.publish(pose_msg)
            
        except Exception as e:
            self.get_logger().error(f'Pose publishing error: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    node = HowYouSeeMeSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
