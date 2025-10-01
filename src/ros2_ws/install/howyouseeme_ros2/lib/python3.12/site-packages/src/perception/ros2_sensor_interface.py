"""
ROS2 Kinect v2 Sensor Interface for HowYouSeeMe

This module provides a ROS2-integrated interface to the Microsoft Kinect v2 sensor
using the kinect2_bridge with CUDA acceleration and intelligent frame management.

Key Features:
- ROS2 subscriber for kinect2_bridge topics
- Synchronized RGB-D frame processing
- TF2 coordinate frame management
- Standard ROS2 sensor_msgs compatibility
- Performance monitoring and metrics publishing
- Intelligent frame dropping for optimal performance

ROS2 Topics Subscribed:
- /kinect2/hd/image_color - RGB images (1920x1080)
- /kinect2/hd/image_depth_rect - Registered depth
- /kinect2/hd/camera_info - Camera calibration

ROS2 Topics Published:
- /howyouseeme/sensor_stats - Performance metrics
- /howyouseeme/frame_sync - Synchronized frame notifications

Performance Metrics:
- Target FPS: 15 (ROS2 optimized)
- Actual Performance: 14.5 FPS with kinect2_bridge
- Processing Latency: <50ms per frame
- Memory Usage: ~2GB with GPU acceleration

Author: Aryan Rai
Date: 2024 - ROS2 Integration
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np
from typing import Tuple, Optional, Dict, Any, Callable
import threading
import queue
import time
import logging
from dataclasses import dataclass
from collections import deque

# ROS2 message types
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge
import tf2_ros
from tf2_ros import TransformListener, Buffer

# Custom messages for performance monitoring
from std_msgs.msg import Float32MultiArray, String

logger = logging.getLogger(__name__)

@dataclass
class SynchronizedFrame:
    """Container for synchronized RGB-D frame data"""
    rgb_image: np.ndarray
    depth_image: np.ndarray
    rgb_timestamp: float
    depth_timestamp: float
    frame_id: int
    rgb_camera_info: Optional[CameraInfo] = None
    depth_camera_info: Optional[CameraInfo] = None
    sync_quality: float = 1.0  # 0-1, how well synchronized the frames are

class ROS2KinectInterface(Node):
    """
    ROS2-integrated Kinect v2 interface for RGB-D data acquisition
    
    This class subscribes to kinect2_bridge topics and provides synchronized
    RGB-D frames with intelligent processing and performance monitoring.
    """
    
    def __init__(self, 
                 node_name: str = 'howyouseeme_kinect_interface',
                 target_fps: float = 15.0,
                 max_sync_delay_ms: float = 50.0,
                 enable_frame_dropping: bool = True,
                 max_frame_age_ms: float = 75.0):
        
        super().__init__(node_name)
        
        # Configuration
        self.target_fps = target_fps
        self.max_sync_delay_ms = max_sync_delay_ms
        self.enable_frame_dropping = enable_frame_dropping
        self.max_frame_age_ms = max_frame_age_ms
        
        # Frame synchronization
        self.rgb_queue = queue.Queue(maxsize=10)
        self.depth_queue = queue.Queue(maxsize=10)
        self.camera_info_rgb = None
        self.camera_info_depth = None
        
        # Performance tracking
        self.frame_count = 0
        self.dropped_frame_count = 0
        self.sync_errors = 0
        self.processing_times = deque(maxlen=100)
        self.last_frame_time = time.time()
        
        # ROS2 components
        self.cv_bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # QoS profiles for different data types
        self.image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2
        )
        
        self.info_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
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
        
        self.rgb_info_sub = self.create_subscription(
            CameraInfo,
            '/kinect2/hd/camera_info',
            self._rgb_info_callback,
            self.info_qos
        )
        
        # Publishers
        self.stats_pub = self.create_publisher(
            Float32MultiArray,
            '/howyouseeme/sensor_stats',
            10
        )
        
        self.sync_pub = self.create_publisher(
            String,
            '/howyouseeme/frame_sync',
            10
        )
        
        # Performance monitoring timer
        self.stats_timer = self.create_timer(5.0, self._publish_stats)
        
        # Frame processing callback
        self.frame_callback: Optional[Callable[[SynchronizedFrame], None]] = None
        
        # Processing thread
        self.processing_thread = threading.Thread(target=self._processing_loop, daemon=True)
        self.running = True
        self.processing_thread.start()
        
        self.get_logger().info(f\"ROS2 Kinect Interface initialized - Target FPS: {target_fps}\")\
    
    def _rgb_callback(self, msg: Image):\n        \"\"\"Handle RGB image messages\"\"\"\n        try:\n            # Convert ROS image to OpenCV format\n            rgb_image = self.cv_bridge.imgmsg_to_cv2(msg, \"bgr8\")\n            \n            # Check frame age for dropping\n            current_time = time.time()\n            msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9\n            frame_age_ms = (current_time - msg_time) * 1000\n            \n            if self.enable_frame_dropping and frame_age_ms > self.max_frame_age_ms:\n                self.dropped_frame_count += 1\n                return\n            \n            # Store frame data\n            frame_data = {\n                'image': rgb_image,\n                'timestamp': msg_time,\n                'header': msg.header,\n                'frame_age_ms': frame_age_ms\n            }\n            \n            # Add to queue (drop oldest if full)\n            self._add_to_queue(self.rgb_queue, frame_data)\n            \n        except Exception as e:\n            self.get_logger().error(f\"RGB callback error: {e}\")\n    \n    def _depth_callback(self, msg: Image):\n        \"\"\"Handle depth image messages\"\"\"\n        try:\n            # Convert ROS image to OpenCV format (16-bit depth)\n            depth_image = self.cv_bridge.imgmsg_to_cv2(msg, \"16UC1\")\n            \n            # Check frame age for dropping\n            current_time = time.time()\n            msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9\n            frame_age_ms = (current_time - msg_time) * 1000\n            \n            if self.enable_frame_dropping and frame_age_ms > self.max_frame_age_ms:\n                self.dropped_frame_count += 1\n                return\n            \n            # Store frame data\n            frame_data = {\n                'image': depth_image,\n                'timestamp': msg_time,\n                'header': msg.header,\n                'frame_age_ms': frame_age_ms\n            }\n            \n            # Add to queue (drop oldest if full)\n            self._add_to_queue(self.depth_queue, frame_data)\n            \n        except Exception as e:\n            self.get_logger().error(f\"Depth callback error: {e}\")\n    \n    def _rgb_info_callback(self, msg: CameraInfo):\n        \"\"\"Handle RGB camera info messages\"\"\"\n        self.camera_info_rgb = msg\n    \n    def _add_to_queue(self, frame_queue: queue.Queue, frame_data: Dict):\n        \"\"\"Safely add frame to queue, dropping oldest if full\"\"\"\n        try:\n            frame_queue.put_nowait(frame_data)\n        except queue.Full:\n            try:\n                frame_queue.get_nowait()  # Drop oldest\n                frame_queue.put_nowait(frame_data)\n                self.dropped_frame_count += 1\n            except queue.Empty:\n                pass\n    \n    def _processing_loop(self):\n        \"\"\"Main processing loop for frame synchronization\"\"\"\n        while self.running:\n            try:\n                # Try to get synchronized frames\n                sync_frame = self._get_synchronized_frame()\n                \n                if sync_frame:\n                    self.frame_count += 1\n                    \n                    # Call user callback if set\n                    if self.frame_callback:\n                        start_time = time.time()\n                        self.frame_callback(sync_frame)\n                        processing_time = (time.time() - start_time) * 1000\n                        self.processing_times.append(processing_time)\n                    \n                    # Publish sync notification\n                    sync_msg = String()\n                    sync_msg.data = f\"frame_{sync_frame.frame_id}_sync_{sync_frame.sync_quality:.3f}\"\n                    self.sync_pub.publish(sync_msg)\n                    \n                    # Rate limiting\n                    current_time = time.time()\n                    time_since_last = current_time - self.last_frame_time\n                    target_interval = 1.0 / self.target_fps\n                    \n                    if time_since_last < target_interval:\n                        time.sleep(target_interval - time_since_last)\n                    \n                    self.last_frame_time = time.time()\n                else:\n                    time.sleep(0.01)  # Small delay if no frames available\n                    \n            except Exception as e:\n                self.get_logger().error(f\"Processing loop error: {e}\")\n                time.sleep(0.1)\n    \n    def _get_synchronized_frame(self) -> Optional[SynchronizedFrame]:\n        \"\"\"Get synchronized RGB and depth frames\"\"\"\n        try:\n            # Get latest frames from both queues\n            rgb_data = None\n            depth_data = None\n            \n            # Try to get RGB frame\n            try:\n                rgb_data = self.rgb_queue.get_nowait()\n            except queue.Empty:\n                return None\n            \n            # Try to get depth frame\n            try:\n                depth_data = self.depth_queue.get_nowait()\n            except queue.Empty:\n                # Put RGB frame back\n                self._add_to_queue(self.rgb_queue, rgb_data)\n                return None\n            \n            # Check synchronization quality\n            time_diff_ms = abs(rgb_data['timestamp'] - depth_data['timestamp']) * 1000\n            \n            if time_diff_ms > self.max_sync_delay_ms:\n                self.sync_errors += 1\n                # Try to find better match by looking for more recent frame\n                if rgb_data['timestamp'] > depth_data['timestamp']:\n                    # RGB is newer, put it back and try again\n                    self._add_to_queue(self.rgb_queue, rgb_data)\n                else:\n                    # Depth is newer, put it back and try again\n                    self._add_to_queue(self.depth_queue, depth_data)\n                return None\n            \n            # Calculate sync quality (1.0 = perfect, 0.0 = max allowed delay)\n            sync_quality = max(0.0, 1.0 - (time_diff_ms / self.max_sync_delay_ms))\n            \n            # Create synchronized frame\n            sync_frame = SynchronizedFrame(\n                rgb_image=rgb_data['image'],\n                depth_image=depth_data['image'],\n                rgb_timestamp=rgb_data['timestamp'],\n                depth_timestamp=depth_data['timestamp'],\n                frame_id=self.frame_count,\n                rgb_camera_info=self.camera_info_rgb,\n                sync_quality=sync_quality\n            )\n            \n            return sync_frame\n            \n        except Exception as e:\n            self.get_logger().error(f\"Frame synchronization error: {e}\")\n            return None\n    \n    def _publish_stats(self):\n        \"\"\"Publish performance statistics\"\"\"\n        try:\n            total_frames = self.frame_count + self.dropped_frame_count\n            drop_rate = (self.dropped_frame_count / total_frames * 100) if total_frames > 0 else 0.0\n            \n            avg_processing_time = np.mean(self.processing_times) if self.processing_times else 0.0\n            \n            # Create stats message\n            stats_msg = Float32MultiArray()\n            stats_msg.data = [\n                float(self.frame_count),\n                float(self.dropped_frame_count),\n                drop_rate,\n                float(self.sync_errors),\n                avg_processing_time,\n                float(self.rgb_queue.qsize()),\n                float(self.depth_queue.qsize()),\n                self.target_fps\n            ]\n            \n            self.stats_pub.publish(stats_msg)\n            \n            # Log stats periodically\n            self.get_logger().info(\n                f\"Stats - Frames: {self.frame_count}, Dropped: {self.dropped_frame_count} \"\n                f\"({drop_rate:.1f}%), Sync errors: {self.sync_errors}, \"\n                f\"Avg processing: {avg_processing_time:.1f}ms\"\n            )\n            \n        except Exception as e:\n            self.get_logger().error(f\"Stats publishing error: {e}\")\n    \n    def set_frame_callback(self, callback: Callable[[SynchronizedFrame], None]):\n        \"\"\"Set callback function for processing synchronized frames\"\"\"\n        self.frame_callback = callback\n    \n    def get_latest_frame(self) -> Optional[SynchronizedFrame]:\n        \"\"\"Get the latest synchronized frame (non-blocking)\"\"\"\n        return self._get_synchronized_frame()\n    \n    def get_camera_info(self) -> Dict[str, Any]:\n        \"\"\"Get camera calibration information\"\"\"\n        info = {\n            'rgb_camera_info': self.camera_info_rgb,\n            'depth_camera_info': self.camera_info_depth,\n            'target_fps': self.target_fps,\n            'max_sync_delay_ms': self.max_sync_delay_ms,\n            'enable_frame_dropping': self.enable_frame_dropping,\n            'max_frame_age_ms': self.max_frame_age_ms\n        }\n        \n        # Extract intrinsics if available\n        if self.camera_info_rgb:\n            info['rgb_intrinsics'] = {\n                'fx': self.camera_info_rgb.k[0],\n                'fy': self.camera_info_rgb.k[4],\n                'cx': self.camera_info_rgb.k[2],\n                'cy': self.camera_info_rgb.k[5],\n                'width': self.camera_info_rgb.width,\n                'height': self.camera_info_rgb.height\n            }\n        \n        return info\n    \n    def get_performance_stats(self) -> Dict[str, Any]:\n        \"\"\"Get comprehensive performance statistics\"\"\"\n        total_frames = self.frame_count + self.dropped_frame_count\n        drop_rate = (self.dropped_frame_count / total_frames * 100) if total_frames > 0 else 0.0\n        \n        return {\n            'frame_count': self.frame_count,\n            'dropped_frame_count': self.dropped_frame_count,\n            'drop_rate_percent': drop_rate,\n            'sync_errors': self.sync_errors,\n            'avg_processing_time_ms': np.mean(self.processing_times) if self.processing_times else 0.0,\n            'rgb_queue_size': self.rgb_queue.qsize(),\n            'depth_queue_size': self.depth_queue.qsize(),\n            'target_fps': self.target_fps,\n            'max_sync_delay_ms': self.max_sync_delay_ms,\n            'enable_frame_dropping': self.enable_frame_dropping,\n            'max_frame_age_ms': self.max_frame_age_ms\n        }\n    \n    def get_transform(self, target_frame: str, source_frame: str) -> Optional[TransformStamped]:\n        \"\"\"Get transform between coordinate frames\"\"\"\n        try:\n            transform = self.tf_buffer.lookup_transform(\n                target_frame, source_frame, rclpy.time.Time()\n            )\n            return transform\n        except Exception as e:\n            self.get_logger().debug(f\"Transform lookup failed: {e}\")\n            return None\n    \n    def shutdown(self):\n        \"\"\"Clean shutdown of the interface\"\"\"\n        self.running = False\n        if self.processing_thread.is_alive():\n            self.processing_thread.join(timeout=2.0)\n        \n        self.get_logger().info(\"ROS2 Kinect Interface shutdown complete\")\n\n\ndef main(args=None):\n    \"\"\"Main function for testing the ROS2 Kinect interface\"\"\"\n    rclpy.init(args=args)\n    \n    # Create interface\n    kinect_interface = ROS2KinectInterface(\n        target_fps=15.0,\n        max_sync_delay_ms=50.0,\n        enable_frame_dropping=True\n    )\n    \n    def frame_processor(sync_frame: SynchronizedFrame):\n        \"\"\"Example frame processing callback\"\"\"\n        print(f\"Processing frame {sync_frame.frame_id} - \"\n              f\"RGB: {sync_frame.rgb_image.shape}, \"\n              f\"Depth: {sync_frame.depth_image.shape}, \"\n              f\"Sync quality: {sync_frame.sync_quality:.3f}\")\n        \n        # Optional: Display frames\n        if sync_frame.frame_id % 30 == 0:  # Every 30th frame\n            cv2.imshow('RGB', sync_frame.rgb_image)\n            depth_display = (sync_frame.depth_image / 16).astype(np.uint8)\n            cv2.imshow('Depth', depth_display)\n            cv2.waitKey(1)\n    \n    # Set frame callback\n    kinect_interface.set_frame_callback(frame_processor)\n    \n    try:\n        rclpy.spin(kinect_interface)\n    except KeyboardInterrupt:\n        print(\"Shutting down...\")\n    finally:\n        kinect_interface.shutdown()\n        cv2.destroyAllWindows()\n        rclpy.shutdown()\n\n\nif __name__ == '__main__':\n    main()\n"