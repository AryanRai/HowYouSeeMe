#!/usr/bin/env python3
"""
YOLOv12 Detector ROS2 Node for HowYouSeeMe

This node subscribes to RGB images and publishes object detections using YOLOv12.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np
from typing import List, Dict, Any
import time
from collections import deque

# ROS2 message types
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, ObjectHypothesis
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Header, Float32MultiArray, String
from cv_bridge import CvBridge

# Try to import YOLO and torch
YOLO_AVAILABLE = False
torch = None
try:
    from ultralytics import YOLO
    import torch
    YOLO_AVAILABLE = True
except ImportError:
    print("Warning: ultralytics not available. Install with: pip install ultralytics")
    # Create a dummy torch module for fallback
    class DummyTorch:
        class cuda:
            @staticmethod
            def is_available():
                return False
    torch = DummyTorch()

class YOLODetectorNode(Node):
    """ROS2 YOLOv12 object detector node"""
    
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Configuration
        self.model_name = 'yolo11n'  # Start with nano model
        self.confidence_threshold = 0.6
        self.nms_threshold = 0.45
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu' if YOLO_AVAILABLE else 'cpu'
        
        # Model state
        self.model = None
        self.model_loaded = False
        
        # Performance tracking
        self.frame_count = 0
        self.detection_count = 0
        self.processing_times = deque(maxlen=100)
        
        # ROS2 components
        self.cv_bridge = CvBridge()
        
        # QoS profiles
        self.image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2
        )
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/kinect2/hd/image_color',
            self._image_callback,
            self.image_qos
        )
        
        # Publishers
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/howyouseeme/detections',
            10
        )
        
        self.annotated_image_pub = self.create_publisher(
            Image,
            '/howyouseeme/detection_image',
            self.image_qos
        )
        
        self.stats_pub = self.create_publisher(
            Float32MultiArray,
            '/howyouseeme/detection_stats',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/howyouseeme/detector_status',
            10
        )
        
        # Stats timer
        self.stats_timer = self.create_timer(5.0, self._publish_stats)
        
        # Initialize model
        if YOLO_AVAILABLE:
            self._initialize_model()
        else:
            self.get_logger().warn("YOLO not available - running in simulation mode")
        
        self.get_logger().info(f"YOLO Detector Node initialized - Model: {self.model_name}")
    
    def _initialize_model(self):
        """Initialize YOLOv12 model"""
        try:
            self.get_logger().info(f"Loading {self.model_name} model...")
            
            # Load YOLO model
            self.model = YOLO(f"{self.model_name}.pt")
            self.model.to(self.device)
            
            # Warm up
            dummy_input = np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8)
            _ = self.model(dummy_input, verbose=False)
            
            self.model_loaded = True
            self.get_logger().info(f"Successfully loaded {self.model_name} on {self.device}")
            
            # Publish status
            status_msg = String()
            status_msg.data = f"model_loaded_{self.model_name}_{self.device}"
            self.status_pub.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            self.model_loaded = False
    
    def _image_callback(self, msg: Image):
        """Handle incoming image messages"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Process detection
            start_time = time.time()
            detections = self._detect_objects(cv_image)
            processing_time = (time.time() - start_time) * 1000
            self.processing_times.append(processing_time)
            
            # Publish results
            if detections or not YOLO_AVAILABLE:
                self._publish_detections(detections, msg.header)
                
                # Create and publish annotated image
                annotated_image = self._annotate_image(cv_image, detections)
                self._publish_annotated_image(annotated_image, msg.header)
            
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f"Image callback error: {e}")
    
    def _detect_objects(self, image: np.ndarray) -> List[Dict]:
        """Detect objects in image"""
        if not YOLO_AVAILABLE or not self.model_loaded:
            # Simulation mode - create fake detections
            return self._create_fake_detections(image)
        
        try:
            # Run YOLO inference
            results = self.model(
                image,
                conf=self.confidence_threshold,
                iou=self.nms_threshold,
                verbose=False
            )
            
            # Parse results
            detections = []
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        conf = box.conf[0].cpu().numpy()
                        cls = int(box.cls[0].cpu().numpy())
                        
                        detection = {
                            'bbox': [int(x1), int(y1), int(x2 - x1), int(y2 - y1)],
                            'confidence': float(conf),
                            'class_id': cls,
                            'class_name': self.model.names[cls],
                            'timestamp': time.time()
                        }
                        detections.append(detection)
            
            self.detection_count += len(detections)
            return detections
            
        except Exception as e:
            self.get_logger().error(f"Detection failed: {e}")
            return []
    
    def _create_fake_detections(self, image: np.ndarray) -> List[Dict]:
        """Create fake detections for testing when YOLO is not available"""
        h, w = image.shape[:2]
        
        # Create some fake detections
        fake_detections = [
            {
                'bbox': [w//4, h//4, w//4, h//4],
                'confidence': 0.85,
                'class_id': 0,
                'class_name': 'person',
                'timestamp': time.time()
            },
            {
                'bbox': [w//2, h//2, w//6, h//6],
                'confidence': 0.72,
                'class_id': 67,
                'class_name': 'cell phone',
                'timestamp': time.time()
            }
        ]
        
        return fake_detections
    
    def _publish_detections(self, detections: List[Dict], header: Header):
        """Publish detections as ROS2 message"""
        try:
            detection_array = Detection2DArray()
            detection_array.header = header
            detection_array.header.frame_id = "kinect2_rgb_optical_frame"
            
            for det in detections:
                detection_msg = Detection2D()
                detection_msg.header = detection_array.header
                
                # Set bounding box
                x, y, w, h = det['bbox']
                detection_msg.bbox.center.position.x = float(x + w / 2)
                detection_msg.bbox.center.position.y = float(y + h / 2)
                detection_msg.bbox.size_x = float(w)
                detection_msg.bbox.size_y = float(h)
                
                # Set hypothesis
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(det['class_id'])
                hypothesis.hypothesis.score = det['confidence']
                detection_msg.results.append(hypothesis)
                
                detection_array.detections.append(detection_msg)
            
            self.detection_pub.publish(detection_array)
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish detections: {e}")
    
    def _annotate_image(self, image: np.ndarray, detections: List[Dict]) -> np.ndarray:
        """Annotate image with detection results"""
        annotated = image.copy()
        
        for det in detections:
            x, y, w, h = det['bbox']
            x1, y1, x2, y2 = x, y, x + w, y + h
            
            # Draw bounding box
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw label
            label = f"{det['class_name']}: {det['confidence']:.2f}"
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            
            # Background for label
            cv2.rectangle(annotated, (x1, y1 - label_size[1] - 10),
                         (x1 + label_size[0], y1), (0, 255, 0), -1)
            
            # Label text
            cv2.putText(annotated, label, (x1, y1 - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        # Add performance info
        if self.processing_times:
            avg_time = np.mean(self.processing_times)
            fps_text = f"FPS: {1000/avg_time:.1f} | Detections: {len(detections)}"
            cv2.putText(annotated, fps_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return annotated
    
    def _publish_annotated_image(self, annotated_image: np.ndarray, header: Header):
        """Publish annotated image"""
        try:
            img_msg = self.cv_bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            img_msg.header = header
            img_msg.header.frame_id = "kinect2_rgb_optical_frame"
            
            self.annotated_image_pub.publish(img_msg)
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish annotated image: {e}")
    
    def _publish_stats(self):
        """Publish performance statistics"""
        try:
            avg_processing_time = np.mean(self.processing_times) if self.processing_times else 0.0
            current_fps = 1000 / avg_processing_time if avg_processing_time > 0 else 0.0
            
            stats_msg = Float32MultiArray()
            stats_msg.data = [
                float(self.frame_count),
                float(self.detection_count),
                avg_processing_time,
                current_fps,
                self.confidence_threshold,
                float(self.model_loaded)
            ]
            
            self.stats_pub.publish(stats_msg)
            
            # Log stats
            if self.frame_count % 50 == 0:  # Log every 50 frames
                self.get_logger().info(
                    f"YOLO Stats - Frames: {self.frame_count}, "
                    f"Detections: {self.detection_count}, "
                    f"FPS: {current_fps:.1f}"
                )
            
        except Exception as e:
            self.get_logger().error(f"Stats publishing error: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        detector = YOLODetectorNode()
        rclpy.spin(detector)
    except KeyboardInterrupt:
        print("Shutting down YOLO detector...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()