#!/usr/bin/env python3
"""
SLAM Interface ROS2 Node for HowYouSeeMe

This node provides basic SLAM functionality using ORB features and publishes
pose estimates and trajectory information.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np
from typing import List, Dict, Any, Optional
import time
from collections import deque

# ROS2 message types
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Header, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

# TF2 for coordinate transforms
import tf2_ros
from tf2_ros import TransformBroadcaster

class SLAMInterfaceNode(Node):
    """Basic SLAM interface using ORB features"""
    
    def __init__(self):
        super().__init__('slam_interface')
        
        # SLAM state
        self.current_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # x, y, z, roll, pitch, yaw
        self.poses = deque(maxlen=1000)
        self.trajectory_path = Path()
        
        # ORB feature detector
        self.orb = cv2.ORB_create(nfeatures=500)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
        # Previous frame data
        self.prev_frame = None
        self.prev_keypoints = None
        self.prev_descriptors = None
        self.is_initialized = False
        
        # Performance tracking
        self.frame_count = 0
        self.processing_times = deque(maxlen=100)
        self.tracking_quality = 1.0
        
        # ROS2 components
        self.cv_bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Camera info
        self.camera_info = None
        self.camera_matrix = None
        
        # QoS profiles
        self.image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2
        )
        
        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image,
            '/kinect2/hd/image_color',
            self._rgb_callback,
            self.image_qos
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/kinect2/hd/image_depth_rect',
            self._depth_callback,
            self.image_qos
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/kinect2/hd/camera_info',
            self._camera_info_callback,
            10
        )
        
        # Publishers
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/howyouseeme/pose',
            10
        )
        
        self.odom_pub = self.create_publisher(
            Odometry,
            '/howyouseeme/odometry',
            10
        )
        
        self.path_pub = self.create_publisher(
            Path,
            '/howyouseeme/trajectory',
            10
        )
        
        self.stats_pub = self.create_publisher(
            Float32MultiArray,
            '/howyouseeme/slam_stats',
            10
        )
        
        self.markers_pub = self.create_publisher(
            MarkerArray,
            '/howyouseeme/slam_markers',
            10
        )
        
        # Frame storage
        self.current_rgb = None
        self.current_depth = None
        
        # Processing timer
        self.processing_timer = self.create_timer(0.1, self._process_slam)  # 10 Hz
        
        # Stats timer
        self.stats_timer = self.create_timer(2.0, self._publish_stats)
        
        self.get_logger().info("SLAM Interface Node initialized")
    
    def _rgb_callback(self, msg: Image):
        """Handle RGB image messages"""
        try:
            self.current_rgb = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"RGB callback error: {e}")
    
    def _depth_callback(self, msg: Image):
        """Handle depth image messages"""
        try:
            self.current_depth = self.cv_bridge.imgmsg_to_cv2(msg, "16UC1")
        except Exception as e:
            self.get_logger().error(f"Depth callback error: {e}")
    
    def _camera_info_callback(self, msg: CameraInfo):
        """Handle camera info messages"""
        self.camera_info = msg
        self.camera_matrix = np.array([
            [msg.k[0], 0, msg.k[2]],
            [0, msg.k[4], msg.k[5]],
            [0, 0, 1]
        ])
    
    def _process_slam(self):
        """Main SLAM processing loop"""
        if self.current_rgb is None:
            return
        
        try:
            start_time = time.time()
            
            # Convert to grayscale
            gray = cv2.cvtColor(self.current_rgb, cv2.COLOR_BGR2GRAY)
            
            # Extract ORB features
            keypoints, descriptors = self.orb.detectAndCompute(gray, None)
            
            if descriptors is None:
                return
            
            # Initialize or track
            if not self.is_initialized:
                self._initialize_slam(gray, keypoints, descriptors)
            else:
                self._track_frame(gray, keypoints, descriptors)
            
            # Update trajectory
            self._update_trajectory()
            
            # Publish results
            self._publish_slam_results()
            
            # Track performance
            processing_time = (time.time() - start_time) * 1000
            self.processing_times.append(processing_time)
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f"SLAM processing error: {e}")
    
    def _initialize_slam(self, gray: np.ndarray, keypoints: List, descriptors: np.ndarray):
        """Initialize SLAM with first frame"""
        self.get_logger().info("Initializing SLAM with first frame")
        
        # Set initial pose
        self.current_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.poses.append(self.current_pose.copy())
        
        # Store frame data
        self.prev_frame = gray.copy()
        self.prev_keypoints = keypoints
        self.prev_descriptors = descriptors
        
        self.is_initialized = True
    
    def _track_frame(self, gray: np.ndarray, keypoints: List, descriptors: np.ndarray):
        """Track camera motion between frames"""
        if self.prev_descriptors is None:
            return
        
        # Match features
        matches = self.matcher.match(self.prev_descriptors, descriptors)
        matches = sorted(matches, key=lambda x: x.distance)
        
        if len(matches) < 20:  # Minimum matches for reliable tracking
            self.tracking_quality = 0.5
            return
        
        # Extract matched points
        prev_pts = np.float32([self.prev_keypoints[m.queryIdx].pt for m in matches[:50]])
        curr_pts = np.float32([keypoints[m.trainIdx].pt for m in matches[:50]])
        
        # Estimate motion using essential matrix
        if self.camera_matrix is not None:
            self._estimate_pose_change(prev_pts, curr_pts)
        else:
            # Simple 2D motion estimation
            self._estimate_2d_motion(prev_pts, curr_pts)
        
        # Update tracking quality
        self.tracking_quality = min(1.0, len(matches) / 100.0)
        
        # Store current frame data
        self.prev_frame = gray.copy()
        self.prev_keypoints = keypoints
        self.prev_descriptors = descriptors
    
    def _estimate_pose_change(self, prev_pts: np.ndarray, curr_pts: np.ndarray):
        """Estimate pose change using essential matrix"""
        try:
            # Find essential matrix
            E, mask = cv2.findEssentialMat(
                prev_pts, curr_pts, self.camera_matrix,
                method=cv2.RANSAC, prob=0.999, threshold=1.0
            )
            
            if E is None:
                return
            
            # Recover pose
            _, R, t, mask = cv2.recoverPose(E, prev_pts, curr_pts, self.camera_matrix)
            
            # Simple integration (in practice, use proper pose composition)
            motion_scale = 0.01  # Scale factor for motion
            self.current_pose[0] += t[0, 0] * motion_scale  # x
            self.current_pose[1] += t[1, 0] * motion_scale  # y
            self.current_pose[2] += t[2, 0] * motion_scale  # z
            
            # Add to poses
            self.poses.append(self.current_pose.copy())
            
        except Exception as e:
            self.get_logger().debug(f"Pose estimation failed: {e}")
    
    def _estimate_2d_motion(self, prev_pts: np.ndarray, curr_pts: np.ndarray):
        """Simple 2D motion estimation"""
        try:
            # Calculate average motion
            motion = np.mean(curr_pts - prev_pts, axis=0)
            
            # Simple integration
            motion_scale = 0.001
            self.current_pose[0] += motion[0] * motion_scale
            self.current_pose[1] += motion[1] * motion_scale
            
            # Add to poses
            self.poses.append(self.current_pose.copy())
            
        except Exception as e:
            self.get_logger().debug(f"2D motion estimation failed: {e}")
    
    def _update_trajectory(self):
        """Update trajectory path message"""
        self.trajectory_path.header.stamp = self.get_clock().now().to_msg()
        self.trajectory_path.header.frame_id = "map"
        
        # Add current pose to path
        pose_stamped = PoseStamped()
        pose_stamped.header = self.trajectory_path.header
        pose_stamped.pose.position.x = self.current_pose[0]
        pose_stamped.pose.position.y = self.current_pose[1]
        pose_stamped.pose.position.z = self.current_pose[2]
        pose_stamped.pose.orientation.w = 1.0  # Identity quaternion
        
        self.trajectory_path.poses.append(pose_stamped)
        
        # Limit path length
        if len(self.trajectory_path.poses) > 1000:
            self.trajectory_path.poses = self.trajectory_path.poses[-1000:]
    
    def _publish_slam_results(self):
        """Publish SLAM results"""
        try:
            current_time = self.get_clock().now()
            
            # Publish pose
            pose_msg = PoseStamped()
            pose_msg.header.stamp = current_time.to_msg()
            pose_msg.header.frame_id = "map"
            pose_msg.pose.position.x = self.current_pose[0]
            pose_msg.pose.position.y = self.current_pose[1]
            pose_msg.pose.position.z = self.current_pose[2]
            pose_msg.pose.orientation.w = 1.0
            self.pose_pub.publish(pose_msg)
            
            # Publish odometry
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time.to_msg()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"
            odom_msg.pose.pose = pose_msg.pose
            self.odom_pub.publish(odom_msg)
            
            # Publish trajectory
            self.path_pub.publish(self.trajectory_path)
            
            # Publish TF transform
            transform = TransformStamped()
            transform.header.stamp = current_time.to_msg()
            transform.header.frame_id = "odom"
            transform.child_frame_id = "base_link"
            transform.transform.translation.x = self.current_pose[0]
            transform.transform.translation.y = self.current_pose[1]
            transform.transform.translation.z = self.current_pose[2]
            transform.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(transform)
            
            # Publish markers
            self._publish_markers()
            
        except Exception as e:
            self.get_logger().error(f"Publishing error: {e}")
    
    def _publish_markers(self):
        """Publish visualization markers"""
        try:
            marker_array = MarkerArray()
            
            # Current pose marker
            pose_marker = Marker()
            pose_marker.header.frame_id = "map"
            pose_marker.header.stamp = self.get_clock().now().to_msg()
            pose_marker.ns = "slam_pose"
            pose_marker.id = 0
            pose_marker.type = Marker.ARROW
            pose_marker.action = Marker.ADD
            pose_marker.pose.position.x = self.current_pose[0]
            pose_marker.pose.position.y = self.current_pose[1]
            pose_marker.pose.position.z = self.current_pose[2]
            pose_marker.pose.orientation.w = 1.0
            pose_marker.scale.x = 0.3
            pose_marker.scale.y = 0.05
            pose_marker.scale.z = 0.05
            pose_marker.color.r = 1.0
            pose_marker.color.g = 0.0
            pose_marker.color.b = 0.0
            pose_marker.color.a = 1.0
            
            marker_array.markers.append(pose_marker)
            self.markers_pub.publish(marker_array)
            
        except Exception as e:
            self.get_logger().error(f"Marker publishing error: {e}")
    
    def _publish_stats(self):
        """Publish SLAM statistics"""
        try:
            avg_processing_time = np.mean(self.processing_times) if self.processing_times else 0.0
            
            stats_msg = Float32MultiArray()
            stats_msg.data = [
                float(self.frame_count),
                float(len(self.poses)),
                avg_processing_time,
                self.tracking_quality,
                float(self.is_initialized)
            ]
            
            self.stats_pub.publish(stats_msg)
            
            # Log stats
            if self.frame_count % 100 == 0:  # Log every 100 frames
                self.get_logger().info(
                    f"SLAM Stats - Frames: {self.frame_count}, "
                    f"Poses: {len(self.poses)}, "
                    f"Quality: {self.tracking_quality:.2f}"
                )
            
        except Exception as e:
            self.get_logger().error(f"Stats publishing error: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        slam_interface = SLAMInterfaceNode()
        rclpy.spin(slam_interface)
    except KeyboardInterrupt:
        print("Shutting down SLAM interface...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()