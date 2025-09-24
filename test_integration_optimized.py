"""
Optimized Integration test for HowYouSeeMe Phase 1 components
High-performance version with reduced computational overhead
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

import time
import cv2
import numpy as np
import logging
from typing import Dict, Any

from perception.sensor_interface import KinectV2Interface
from perception.slam.slam_interface import BasicSLAM
from perception.detection.object_detector import YOLODetector, ResourceManager

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class OptimizedHowYouSeeMeIntegration:
    """Optimized integration test class for Phase 1 components"""
    
    def __init__(self):
        self.kinect = None
        self.slam = None
        self.detector = None
        self.resource_manager = ResourceManager(max_gpu_memory_gb=4.0, max_concurrent_models=1)
        
        # Statistics
        self.frame_count = 0
        self.start_time = time.time()
        self.processing_times = []
        
        # Performance optimization settings
        self.detection_interval = 10  # Run detection every 10th frame
        self.slam_interval = 2       # Run SLAM every 2nd frame
        self.visualization_interval = 3  # Update visualization every 3rd frame
        
        # Cached results
        self.last_detections = []
        self.last_slam_result = None
        
    def initialize_components(self) -> bool:
        """Initialize all perception components"""
        logger.info("Initializing optimized HowYouSeeMe components...")
        
        try:
            # Initialize Kinect interface
            self.kinect = KinectV2Interface(
                use_modern=True,
                preferred_pipeline="opengl"  # Force OpenGL for best performance
            )
            if not self.kinect.start():
                logger.error("Failed to initialize Kinect")
                return False
            
            logger.info(f"Kinect initialized: {self.kinect.get_camera_info()}")
            
            # Get camera info for SLAM
            camera_info = self.kinect.get_camera_info()
            rgb_intrinsics = camera_info['rgb_intrinsics']
            
            # Initialize SLAM with reduced features
            self.slam = BasicSLAM(rgb_intrinsics)
            logger.info("SLAM initialized")
            
            # Initialize object detector with optimized settings
            self.detector = YOLODetector(
                model_name="yolov5n",  # Use nano model for speed
                confidence_threshold=0.6,  # Higher threshold for fewer false positives
                resource_manager=self.resource_manager
            )
            logger.info("Object detector initialized")
            
            return True
            
        except Exception as e:
            logger.error(f"Component initialization failed: {e}")
            return False
    
    def process_frame(self, rgb_frame: np.ndarray, depth_frame: np.ndarray = None) -> Dict[str, Any]:
        """Process a single frame through the optimized perception pipeline"""
        start_time = time.time()
        
        # SLAM processing (every slam_interval frames)
        if self.frame_count % self.slam_interval == 0:
            self.last_slam_result = self.slam.process_frame(rgb_frame, depth_frame)
        
        slam_result = self.last_slam_result or {
            'keypoints': [],
            'is_tracking': False,
            'num_features': 0,
            'pose': None
        }
        
        # Object detection (every detection_interval frames)
        if self.frame_count % self.detection_interval == 0:
            self.last_detections = self.detector.detect_objects(rgb_frame)
        
        detections = self.last_detections
        
        # Calculate processing time
        processing_time = time.time() - start_time
        self.processing_times.append(processing_time)
        if len(self.processing_times) > 50:  # Smaller buffer
            self.processing_times.pop(0)
        
        return {
            'slam': slam_result,
            'detections': detections,
            'processing_time': processing_time,
            'timestamp': time.time()
        }
    
    def visualize_results(self, frame: np.ndarray, results: Dict[str, Any]) -> np.ndarray:
        """Create lightweight visualization of processing results"""
        # Only update visualization every visualization_interval frames
        if self.frame_count % self.visualization_interval != 0:
            return frame
        
        vis_frame = frame.copy()
        
        # Draw SLAM features (reduced)
        slam_result = results['slam']
        if slam_result and slam_result.get('keypoints'):
            keypoints = slam_result['keypoints'][:100]  # Limit to 100 keypoints for performance
            for kp in keypoints:
                x, y = int(kp.pt[0]), int(kp.pt[1])
                cv2.circle(vis_frame, (x, y), 2, (0, 255, 255), -1)
        
        # Draw object detections
        detections = results['detections']
        for det in detections:
            x, y, w, h = det.bbox
            cv2.rectangle(vis_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Simple label
            label = f"{det.class_name}: {det.confidence:.2f}"
            cv2.putText(vis_frame, label, (x, y - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Add lightweight status overlay
        self._add_lightweight_status(vis_frame, results)
        
        return vis_frame
    
    def _add_lightweight_status(self, frame: np.ndarray, results: Dict[str, Any]):
        """Add lightweight status information overlay"""
        slam_result = results['slam']
        detections = results['detections']
        
        # Calculate FPS
        elapsed = time.time() - self.start_time
        fps = self.frame_count / elapsed if elapsed > 0 else 0
        
        # Simple status text
        status_lines = [
            f"FPS: {fps:.1f}",
            f"SLAM: {'OK' if slam_result and slam_result.get('is_tracking') else 'LOST'}",
            f"Objects: {len(detections)}"
        ]
        
        # Draw status with minimal overhead
        y_offset = 30
        for line in status_lines:
            cv2.putText(frame, line, (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            y_offset += 25
    
    def run_test(self, max_frames: int = 1000, show_visualization: bool = True):
        """Run the optimized integration test"""
        logger.info(f"Starting optimized integration test for {max_frames} frames...")
        
        if not self.initialize_components():
            logger.error("Failed to initialize components")
            return False
        
        try:
            while self.frame_count < max_frames:
                # Get frame from Kinect
                rgb_data, depth_data = self.kinect.get_synchronized_frames()
                if rgb_data is None:
                    time.sleep(0.001)  # Minimal sleep
                    continue
                
                rgb_frame = rgb_data['frame']
                depth_frame = depth_data['frame'] if depth_data else None
                
                # Process frame through optimized pipeline
                results = self.process_frame(rgb_frame, depth_frame)
                
                # Update frame count
                self.frame_count += 1
                
                # Log progress less frequently
                if self.frame_count % 60 == 0:  # Every 60 frames
                    self._log_progress(results)
                
                # Lightweight visualization
                if show_visualization and self.frame_count % self.visualization_interval == 0:
                    vis_frame = self.visualize_results(rgb_frame, results)
                    cv2.imshow('HowYouSeeMe Optimized Test', vis_frame)
                    
                    # Non-blocking key check
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
                    elif key == ord('r'):
                        if self.slam:
                            self.slam.reset()
                        logger.info("SLAM reset")
            
            # Final statistics
            self._print_final_stats()
            return True
            
        except KeyboardInterrupt:
            logger.info("Test interrupted by user")
            return True
        except Exception as e:
            logger.error(f"Test failed: {e}")
            return False
        finally:
            self.cleanup()
    
    def _log_progress(self, results: Dict[str, Any]):
        """Log progress information"""
        slam_result = results['slam']
        detections = results['detections']
        
        elapsed = time.time() - self.start_time
        fps = self.frame_count / elapsed
        avg_processing = np.mean(self.processing_times) if self.processing_times else 0
        
        logger.info(f"Frame {self.frame_count}: FPS={fps:.1f}, "
                   f"Processing={avg_processing*1000:.1f}ms, "
                   f"SLAM={'OK' if slam_result and slam_result.get('is_tracking') else 'LOST'}, "
                   f"Objects={len(detections)}")
    
    def _print_final_stats(self):
        """Print final test statistics"""
        elapsed = time.time() - self.start_time
        avg_fps = self.frame_count / elapsed
        avg_processing = np.mean(self.processing_times) if self.processing_times else 0
        
        print("\n" + "="*50)
        print("OPTIMIZED INTEGRATION TEST RESULTS")
        print("="*50)
        print(f"Total frames processed: {self.frame_count}")
        print(f"Total time: {elapsed:.2f} seconds")
        print(f"Average FPS: {avg_fps:.2f}")
        print(f"Average processing time: {avg_processing*1000:.2f}ms")
        print(f"Detection interval: every {self.detection_interval} frames")
        print(f"SLAM interval: every {self.slam_interval} frames")
        
        if self.detector:
            det_stats = self.detector.get_performance_stats()
            if det_stats:
                print(f"Detection FPS estimate: {det_stats.get('fps_estimate', 0):.1f}")
        
        print("="*50)
    
    def cleanup(self):
        """Clean up resources"""
        logger.info("Cleaning up resources...")
        
        if self.kinect:
            self.kinect.stop()
        
        if self.detector:
            self.detector.cleanup()
        
        cv2.destroyAllWindows()


def main():
    """Main test function"""
    print("HowYouSeeMe Optimized Integration Test")
    print("=====================================")
    print("This optimized test features:")
    print("- Reduced computational overhead")
    print("- Selective processing intervals")
    print("- Lightweight visualization")
    print("- YOLOv5n (nano) for speed")
    print()
    print("Controls:")
    print("- 'q': quit test")
    print("- 'r': reset SLAM")
    print()
    
    # Create and run optimized test
    test = OptimizedHowYouSeeMeIntegration()
    
    try:
        success = test.run_test(max_frames=1000, show_visualization=True)
        if success:
            print("Optimized integration test completed successfully!")
        else:
            print("Optimized integration test failed!")
    except Exception as e:
        print(f"Test error: {e}")
        logger.error(f"Test error: {e}")
    finally:
        test.cleanup()


if __name__ == "__main__":
    main()