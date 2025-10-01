"""
Object Detection for HowYouSeeMe - ROS2 Integration
YOLOv12-based object detection with ROS2 publishing and resource management
"""

import torch
import torchvision.transforms as transforms
from typing import List, Dict, Tuple, Optional, Any
import cv2
import numpy as np
import logging
import time
from dataclasses import dataclass
import threading
import queue

# ROS2 imports (optional - graceful fallback if not available)
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from geometry_msgs.msg import Point
    from std_msgs.msg import Header, String
    from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

logger = logging.getLogger(__name__)

@dataclass
class Detection:
    """Object detection result"""
    bbox: List[int]  # [x, y, width, height]
    confidence: float
    class_name: str
    class_id: int
    center: Tuple[int, int]
    timestamp: float
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary"""
        return {
            'bbox': self.bbox,
            'confidence': self.confidence,
            'class': self.class_name,
            'class_id': self.class_id,
            'center': self.center,
            'timestamp': self.timestamp
        }

class ResourceManager:
    """Manages GPU resources and model loading"""
    
    def __init__(self, max_gpu_memory_gb: float = 6.0, max_concurrent_models: int = 2):
        self.max_gpu_memory = max_gpu_memory_gb * 1024**3  # Convert to bytes
        self.max_concurrent_models = max_concurrent_models
        self.loaded_models = {}
        self.model_usage = {}
        self.lock = threading.Lock()
        
    def can_load_model(self, model_name: str) -> bool:
        """Check if we can load another model"""
        with self.lock:
            if len(self.loaded_models) >= self.max_concurrent_models:
                return model_name in self.loaded_models
            
            # Check GPU memory if CUDA is available
            if torch.cuda.is_available():
                try:
                    memory_used = torch.cuda.memory_allocated()
                    return memory_used < self.max_gpu_memory * 0.8  # 80% threshold
                except:
                    pass
            
            return True
    
    def register_model(self, model_name: str, model: Any):
        """Register a loaded model"""
        with self.lock:
            self.loaded_models[model_name] = model
            self.model_usage[model_name] = time.time()
    
    def get_model(self, model_name: str) -> Optional[Any]:
        """Get a loaded model"""
        with self.lock:
            if model_name in self.loaded_models:
                self.model_usage[model_name] = time.time()
                return self.loaded_models[model_name]
            return None
    
    def cleanup_unused_models(self, timeout_seconds: float = 300):
        """Remove models not used recently"""
        with self.lock:
            current_time = time.time()
            to_remove = []
            
            for model_name, last_used in self.model_usage.items():
                if current_time - last_used > timeout_seconds:
                    to_remove.append(model_name)
            
            for model_name in to_remove:
                if model_name in self.loaded_models:
                    del self.loaded_models[model_name]
                    del self.model_usage[model_name]
                    logger.info(f"Unloaded unused model: {model_name}")
                    
                    # Force garbage collection
                    if torch.cuda.is_available():
                        torch.cuda.empty_cache()

class YOLOv12Detector:
    """YOLOv12 object detector with ROS2 integration and resource management"""
    
    def __init__(self, model_name: str = "yolo11n", confidence_threshold: float = 0.5,
                 nms_threshold: float = 0.4, resource_manager: Optional[ResourceManager] = None,
                 enable_ros2: bool = True):
        # Use YOLO11 (latest available) as YOLOv12 equivalent
        self.model_name = model_name
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        self.resource_manager = resource_manager or ResourceManager()
        self.model = None
        self.model_loaded = False
        self.load_lock = threading.Lock()
        
        # ROS2 integration
        self.enable_ros2 = enable_ros2 and ROS2_AVAILABLE
        self.ros2_node = None
        self.detection_publisher = None
        
        # Performance tracking
        self.inference_times = []
        self.last_cleanup = time.time()
        self.total_detections = 0
        
        logger.info(f"YOLOv12Detector initialized for {model_name} on {self.device}")
        if self.enable_ros2:
            logger.info("ROS2 integration enabled")
        
    def initialize_ros2(self, node_name: str = "yolo_detector"):
        """Initialize ROS2 node and publishers"""
        if not self.enable_ros2:
            return False
        
        try:
            if not rclpy.ok():
                rclpy.init()
            
            self.ros2_node = rclpy.create_node(node_name)
            
            # Create publishers
            self.detection_publisher = self.ros2_node.create_publisher(
                Detection2DArray, '/howyouseeme/detections', 10)
            
            logger.info("ROS2 node initialized for object detection")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize ROS2: {e}")
            self.enable_ros2 = False
            return False
    
    def _load_model(self) -> bool:
        """Load YOLO model with resource management"""
        with self.load_lock:
            if self.model_loaded and self.model is not None:
                return True
            
            # Check if model is already loaded by resource manager
            existing_model = self.resource_manager.get_model(self.model_name)
            if existing_model is not None:
                self.model = existing_model
                self.model_loaded = True
                return True
            
            # Check if we can load a new model
            if not self.resource_manager.can_load_model(self.model_name):
                logger.warning(f"Cannot load {self.model_name} - resource limits exceeded")
                return False
            
            try:
                logger.info(f"Loading {self.model_name}...")
                
                # Use ultralytics YOLO instead of torch.hub for better performance
                from ultralytics import YOLO
                
                # Load model (will cache locally after first download)
                self.model = YOLO(f'{self.model_name}.pt')
                
                # Configure model for inference
                self.model.overrides['conf'] = self.confidence_threshold
                self.model.overrides['iou'] = self.nms_threshold
                self.model.overrides['verbose'] = False
                
                # Register with resource manager
                self.resource_manager.register_model(self.model_name, self.model)
                self.model_loaded = True
                
                logger.info(f"Successfully loaded {self.model_name}")
                return True
                
            except Exception as e:
                logger.error(f"Failed to load {self.model_name}: {e}")
                self.model = None
                self.model_loaded = False
                return False
    
    def detect_objects(self, rgb_frame: np.ndarray, roi: Optional[Tuple[int, int, int, int]] = None) -> List[Detection]:
        """Detect objects in RGB frame"""
        if not self._load_model():
            return []
        
        try:
            start_time = time.time()
            
            # Apply ROI if specified
            if roi is not None:
                x, y, w, h = roi
                frame_roi = rgb_frame[y:y+h, x:x+w]
                offset_x, offset_y = x, y
            else:
                frame_roi = rgb_frame
                offset_x, offset_y = 0, 0
            
            # Run inference with ultralytics YOLO
            results = self.model(frame_roi, verbose=False)
            
            # Parse results (ultralytics format)
            detections = []
            if results and len(results) > 0:
                result = results[0]  # First (and only) image result
                
                if result.boxes is not None:
                    boxes = result.boxes.xyxy.cpu().numpy()  # x1, y1, x2, y2
                    confidences = result.boxes.conf.cpu().numpy()
                    class_ids = result.boxes.cls.cpu().numpy()
                    
                    for i, (box, conf, cls_id) in enumerate(zip(boxes, confidences, class_ids)):
                        if conf >= self.confidence_threshold:
                            x1, y1, x2, y2 = map(int, box)
                            
                            # Adjust coordinates for ROI offset
                            x1 += offset_x
                            y1 += offset_y
                            x2 += offset_x
                            y2 += offset_y
                            
                            class_name = self.model.names[int(cls_id)]
                            
                            detection = Detection(
                                bbox=[x1, y1, x2 - x1, y2 - y1],
                                confidence=float(conf),
                                class_name=class_name,
                                class_id=int(cls_id),
                                center=((x1 + x2) // 2, (y1 + y2) // 2),
                                timestamp=time.time()
                            )
                            
                            detections.append(detection)
            
            # Track performance
            inference_time = time.time() - start_time
            self.inference_times.append(inference_time)
            if len(self.inference_times) > 100:
                self.inference_times.pop(0)
            
            # Periodic cleanup
            if time.time() - self.last_cleanup > 300:  # 5 minutes
                self.resource_manager.cleanup_unused_models()
                self.last_cleanup = time.time()
            
            return detections
            
        except Exception as e:
            logger.error(f"Detection error: {e}")
            return []
    
    def detect_persons(self, rgb_frame: np.ndarray) -> List[Detection]:
        """Detect only persons (optimized for human tracking)"""
        all_detections = self.detect_objects(rgb_frame)
        return [det for det in all_detections if det.class_name == 'person']
    
    def get_performance_stats(self) -> Dict[str, float]:
        """Get performance statistics"""
        if not self.inference_times:
            return {}
        
        times = np.array(self.inference_times)
        return {
            'avg_inference_time': float(np.mean(times)),
            'min_inference_time': float(np.min(times)),
            'max_inference_time': float(np.max(times)),
            'fps_estimate': 1.0 / np.mean(times) if np.mean(times) > 0 else 0.0,
            'total_detections': len(self.inference_times)
        }
    
    def visualize_detections(self, frame: np.ndarray, detections: List[Detection], 
                           show_confidence: bool = True) -> np.ndarray:
        """Draw detection boxes on frame"""
        vis_frame = frame.copy()
        
        for det in detections:
            x, y, w, h = det.bbox
            conf = det.confidence
            class_name = det.class_name
            
            # Choose color based on class
            if class_name == 'person':
                color = (0, 255, 0)  # Green for persons
            elif class_name in ['car', 'truck', 'bus', 'motorcycle']:
                color = (255, 0, 0)  # Blue for vehicles
            else:
                color = (0, 255, 255)  # Yellow for other objects
            
            # Draw bounding box
            cv2.rectangle(vis_frame, (x, y), (x + w, y + h), color, 2)
            
            # Draw label
            if show_confidence:
                label = f"{class_name}: {conf:.2f}"
            else:
                label = class_name
            
            # Calculate text size for background
            (text_width, text_height), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
            
            # Draw background rectangle for text
            cv2.rectangle(vis_frame, (x, y - text_height - 10), 
                         (x + text_width, y), color, -1)
            
            # Draw text
            cv2.putText(vis_frame, label, (x, y - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            # Draw center point
            center_x, center_y = det.center
            cv2.circle(vis_frame, (center_x, center_y), 3, color, -1)
        
        return vis_frame
    
    def cleanup(self):
        """Clean up resources"""
        self.model = None
        self.model_loaded = False
        if torch.cuda.is_available():
            torch.cuda.empty_cache()

class MultiObjectDetector:
    """Manages multiple detection models for different use cases"""
    
    def __init__(self, resource_manager: Optional[ResourceManager] = None):
        self.resource_manager = resource_manager or ResourceManager()
        self.detectors = {}
        
        # Initialize different detectors for different use cases
        self.person_detector = YOLODetector("yolov5s", confidence_threshold=0.6, 
                                          resource_manager=self.resource_manager)
        self.general_detector = YOLODetector("yolov5m", confidence_threshold=0.5,
                                           resource_manager=self.resource_manager)
    
    def detect_humans(self, rgb_frame: np.ndarray) -> List[Detection]:
        """Optimized human detection"""
        return self.person_detector.detect_persons(rgb_frame)
    
    def detect_all_objects(self, rgb_frame: np.ndarray) -> List[Detection]:
        """General object detection"""
        return self.general_detector.detect_objects(rgb_frame)
    
    def get_system_stats(self) -> Dict[str, Any]:
        """Get system-wide detection statistics"""
        return {
            'person_detector': self.person_detector.get_performance_stats(),
            'general_detector': self.general_detector.get_performance_stats(),
            'gpu_available': torch.cuda.is_available(),
            'gpu_memory_used': torch.cuda.memory_allocated() if torch.cuda.is_available() else 0,
            'loaded_models': len(self.resource_manager.loaded_models)
        }


# Test the detector
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    
    print("Testing YOLO Object Detector...")
    
    # Initialize detector
    detector = YOLODetector("yolov5s")
    cap = cv2.VideoCapture(0)
    
    frame_count = 0
    start_time = time.time()
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Detect objects
        detections = detector.detect_objects(frame)
        
        # Visualize
        vis_frame = detector.visualize_detections(frame, detections)
        
        # Show stats
        frame_count += 1
        if frame_count % 30 == 0:  # Every 30 frames
            elapsed = time.time() - start_time
            fps = frame_count / elapsed
            stats = detector.get_performance_stats()
            
            print(f"Frame {frame_count}: {len(detections)} objects detected")
            print(f"FPS: {fps:.1f}, Avg inference: {stats.get('avg_inference_time', 0):.3f}s")
            
            for det in detections[:3]:  # Show first 3
                print(f"  - {det.class_name}: {det.confidence:.2f} at {det.center}")
        
        # Add FPS counter to image
        elapsed = time.time() - start_time
        fps = frame_count / elapsed if elapsed > 0 else 0
        cv2.putText(vis_frame, f"FPS: {fps:.1f}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow('Object Detection', vis_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    
    # Final stats
    final_stats = detector.get_performance_stats()
    print(f"\nFinal Statistics:")
    for key, value in final_stats.items():
        print(f"  {key}: {value}")
    
    detector.cleanup()
    print("Detection test completed!")