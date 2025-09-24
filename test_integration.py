"""
Integration test for HowYouSeeMe Phase 1 components
Tests Kinect interface, SLAM, and object detection together
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

class HowYouSeeMeIntegration:
    """Integration test class for Phase 1 components"""
    
    def __init__(self):
        self.kinect = None
        self.slam = None
        self.detector = None
        self.resource_manager = ResourceManager(max_gpu_memory_gb=4.0, max_concurrent_models=2)
        
        # Statistics
        self.frame_count = 0
        self.start_time = time.time()
        self.processing_times = []
        
    def initialize_components(self) -> bool:
        """Initialize all perception components"""
        logger.info("Initializing HowYouSeeMe components...")
        
        try:
            # Initialize Kinect interface
            self.kinect = KinectV2Interface(use_protonect=True)
            if not self.kinect.start():
                logger.warning("Kinect failed, falling back to webcam")
                self.kinect = None
                return False
            
            # Get camera info for SLAM
            camera_info = self.kinect.get_camera_info()
            rgb_intrinsics = camera_info['rgb_intrinsics']
            
            # Initialize SLAM
            self.slam = BasicSLAM(rgb_intrinsics)
            logger.info("SLAM initialized")
            
            # Initialize object detector
            self.detector = YOLODetector(
                model_name="yolov5s",
                confidence_threshold=0.5,
                resource_manager=self.resource_manager
            )
            logger.info("Object detector initialized")
            
            return True
            
        except Exception as e:
            logger.error(f"Component initialization failed: {e}")
            return False
    
    def process_frame(self, rgb_frame: np.ndarray, depth_frame: np.ndarray = None) -> Dict[str, Any]:
        """Process a single frame through the perception pipeline"""
        start_time = time.time()
        
        # SLAM processing
        slam_result = self.slam.process_frame(rgb_frame, depth_frame)
        
        # Object detection
        detections = self.detector.detect_objects(rgb_frame)
        
        # Calculate processing time
        processing_time = time.time() - start_time
        self.processing_times.append(processing_time)
        if len(self.processing_times) > 100:
            self.processing_times.pop(0)
        
        return {
            'slam': slam_result,
            'detections': detections,
            'processing_time': processing_time,
            'timestamp': time.time()
        }
    
    def visualize_results(self, frame: np.ndarray, results: Dict[str, Any]) -> np.ndarray:
        """Create visualization of all processing results"""
        vis_frame = frame.copy()
        
        # Draw SLAM features
        slam_result = results['slam']
        if slam_result['keypoints']:
            vis_frame = cv2.drawKeypoints(vis_frame, slam_result['keypoints'], None, 
                                        color=(0, 255, 255), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
        # Draw object detections
        detections = results['detections']
        vis_frame = self.detector.visualize_detections(vis_frame, detections)
        
        # Add status information
        self._add_status_overlay(vis_frame, results)
        
        return vis_frame
    
    def _add_status_overlay(self, frame: np.ndarray, results: Dict[str, Any]):
        """Add status information overlay to frame"""
        slam_result = results['slam']
        detections = results['detections']
        
        # Background for text
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (400, 200), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
        
        # Text information
        y_offset = 30
        line_height = 25
        
        # FPS and timing
        elapsed = time.time() - self.start_time
        fps = self.frame_count / elapsed if elapsed > 0 else 0
        avg_processing = np.mean(self.processing_times) if self.processing_times else 0
        
        cv2.putText(frame, f"FPS: {fps:.1f}", (15, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        y_offset += line_height
        
        cv2.putText(frame, f"Processing: {avg_processing*1000:.1f}ms", (15, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        y_offset += line_height
        
        # SLAM status
        slam_status = "TRACKING" if slam_result['is_tracking'] else "LOST"
        slam_color = (0, 255, 0) if slam_result['is_tracking'] else (0, 0, 255)
        cv2.putText(frame, f"SLAM: {slam_status}", (15, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, slam_color, 1)
        y_offset += line_height
        
        cv2.putText(frame, f"Features: {slam_result['num_features']}", (15, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        y_offset += line_height
        
        # Detection info
        cv2.putText(frame, f"Objects: {len(detections)}", (15, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        y_offset += line_height
        
        # Show detected classes
        if detections:
            classes = list(set([det.class_name for det in detections]))
            class_text = ", ".join(classes[:3])  # Show first 3 classes
            if len(classes) > 3:
                class_text += "..."
            cv2.putText(frame, f"Classes: {class_text}", (15, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    def run_test(self, max_frames: int = 1000, show_visualization: bool = True):
        """Run the integration test"""
        logger.info(f"Starting integration test for {max_frames} frames...")
        
        if not self.initialize_components():
            logger.error("Failed to initialize components")
            return False
        
        try:
            while self.frame_count < max_frames:
                # Get frame from Kinect
                if self.kinect:
                    rgb_data, depth_data = self.kinect.get_synchronized_frames()
                    if rgb_data is None:
                        time.sleep(0.01)
                        continue
                    
                    rgb_frame = rgb_data['frame']
                    depth_frame = depth_data['frame'] if depth_data else None
                else:
                    # Fallback: use webcam
                    ret, rgb_frame = cv2.VideoCapture(0).read()
                    if not ret:
                        continue
                    depth_frame = None
                
                # Process frame through pipeline
                results = self.process_frame(rgb_frame, depth_frame)
                
                # Update frame count
                self.frame_count += 1
                
                # Log progress
                if self.frame_count % 30 == 0:
                    self._log_progress(results)
                
                # Visualization
                if show_visualization:
                    vis_frame = self.visualize_results(rgb_frame, results)
                    cv2.imshow('HowYouSeeMe Integration Test', vis_frame)
                    
                    # Show SLAM trajectory
                    if self.slam:
                        traj_img = self.slam.visualize_trajectory()
                        cv2.imshow('SLAM Trajectory', traj_img)
                    
                    # Check for quit
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
                    elif key == ord('r'):
                        if self.slam:
                            self.slam.reset()
                        logger.info("SLAM reset")
                
                # Small delay to prevent overwhelming
                time.sleep(0.01)
            
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
        
        logger.info(f"Frame {self.frame_count}: FPS={fps:.1f}, "
                   f"SLAM={'OK' if slam_result['is_tracking'] else 'LOST'}, "
                   f"Features={slam_result['num_features']}, "
                   f"Objects={len(detections)}")
        
        # Show detected objects
        if detections:
            for det in detections[:3]:  # Show first 3
                logger.info(f"  - {det.class_name}: {det.confidence:.2f}")
    
    def _print_final_stats(self):
        """Print final test statistics"""
        elapsed = time.time() - self.start_time
        avg_fps = self.frame_count / elapsed
        avg_processing = np.mean(self.processing_times) if self.processing_times else 0
        
        print("\n" + "="*50)
        print("INTEGRATION TEST RESULTS")
        print("="*50)
        print(f"Total frames processed: {self.frame_count}")
        print(f"Total time: {elapsed:.2f} seconds")
        print(f"Average FPS: {avg_fps:.2f}")
        print(f"Average processing time: {avg_processing*1000:.2f}ms")
        
        if self.slam:
            trajectory = self.slam.get_trajectory()
            print(f"SLAM trajectory points: {len(trajectory)}")
        
        if self.detector:
            det_stats = self.detector.get_performance_stats()
            print(f"Detection stats: {det_stats}")
        
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
    print("HowYouSeeMe Phase 1 Integration Test")
    print("====================================")
    print("This test integrates:")
    print("- Kinect v2 sensor interface")
    print("- Basic SLAM with ORB features")
    print("- YOLO object detection")
    print()
    print("Controls:")
    print("- 'q': quit test")
    print("- 'r': reset SLAM")
    print()
    
    # Create and run test
    test = HowYouSeeMeIntegration()
    
    try:
        success = test.run_test(max_frames=500, show_visualization=True)
        if success:
            print("Integration test completed successfully!")
        else:
            print("Integration test failed!")
    except Exception as e:
        print(f"Test error: {e}")
        logger.error(f"Test error: {e}")
    finally:
        test.cleanup()


if __name__ == "__main__":
    main()