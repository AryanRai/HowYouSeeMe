"""
Kinect v2 Sensor Interface for HowYouSeeMe
Provides RGB-D data acquisition using modern pylibfreenect2-py310
"""

import cv2
import numpy as np
from typing import Tuple, Optional, Dict, Any
import threading
import queue
import time
import logging
import subprocess
import os

# Try to import modern Kinect interface
try:
    from .modern_kinect_interface import ModernKinectInterface, KinectFrame
    MODERN_KINECT_AVAILABLE = True
except ImportError:
    try:
        from modern_kinect_interface import ModernKinectInterface, KinectFrame
        MODERN_KINECT_AVAILABLE = True
    except ImportError:
        MODERN_KINECT_AVAILABLE = False

logger = logging.getLogger(__name__)

class KinectV2Interface:
    """
    Kinect v2 interface for RGB-D data acquisition
    
    This class now uses the modern pylibfreenect2-py310 bindings when available,
    with fallback to the legacy implementation for compatibility.
    """
    
    def __init__(self, device_id: int = 0, use_modern: bool = True, preferred_pipeline: str = "auto"):
        self.device_id = device_id
        self.use_modern = use_modern and MODERN_KINECT_AVAILABLE
        self.preferred_pipeline = preferred_pipeline
        
        # Initialize the appropriate interface
        if self.use_modern:
            logger.info("Using modern pylibfreenect2-py310 interface")
            self.modern_kinect = ModernKinectInterface(
                preferred_pipeline=preferred_pipeline,
                enable_rgb=True,
                enable_depth=True,
                enable_ir=False
            )
            # Get intrinsics from modern interface
            camera_info = self.modern_kinect.get_camera_info()
            self.rgb_intrinsics = camera_info['rgb_intrinsics']
            self.depth_intrinsics = camera_info['depth_intrinsics']
        else:
            logger.info("Using legacy interface (modern pylibfreenect2 not available)")
            self.modern_kinect = None
            # Legacy setup
            self.rgb_queue = queue.Queue(maxsize=5)
            self.depth_queue = queue.Queue(maxsize=5)
            self.running = False
            self.capture_thread = None
            self.protonect_process = None
            
            # Camera intrinsics (Kinect v2 defaults)
            self.rgb_intrinsics = {
                'fx': 1081.37, 'fy': 1081.37,
                'cx': 959.5, 'cy': 539.5,
                'width': 1920, 'height': 1080
            }
            
            self.depth_intrinsics = {
                'fx': 365.456, 'fy': 365.456,
                'cx': 257.128, 'cy': 210.468,
                'width': 512, 'height': 424
            }
        
    def start(self) -> bool:
        """Start Kinect data capture"""
        try:
            if self.use_modern and self.modern_kinect:
                return self.modern_kinect.start_streaming()
            else:
                # Legacy implementation
                if self._check_protonect():
                    return self._start_protonect_capture()
                else:
                    return self._start_opencv_capture()
                
        except Exception as e:
            logger.error(f"Failed to start Kinect: {e}")
            return False
    
    def _check_protonect(self) -> bool:
        """Check if Protonect is available"""
        try:
            result = subprocess.run(['which', 'Protonect'], 
                                  capture_output=True, text=True)
            return result.returncode == 0
        except:
            return False
    
    def _start_protonect_capture(self) -> bool:
        """Start capture using Protonect (preferred method)"""
        try:
            # Set up environment for libfreenect2
            env = os.environ.copy()
            freenect2_lib = os.path.expanduser("~/freenect2/lib")
            if os.path.exists(freenect2_lib):
                env['LD_LIBRARY_PATH'] = f"{freenect2_lib}:{env.get('LD_LIBRARY_PATH', '')}"
            
            # Try CUDA pipeline first, fallback to others
            pipelines = ['cuda', 'cl', 'opengl', 'cpu']
            
            for pipeline in pipelines:
                try:
                    env['LIBFREENECT2_PIPELINE'] = pipeline
                    # Test if pipeline works
                    test_cmd = ['Protonect', '-frames', '1', '-noviewer']
                    result = subprocess.run(test_cmd, env=env, capture_output=True, 
                                          text=True, timeout=10)
                    
                    if result.returncode == 0:
                        logger.info(f"Using {pipeline} pipeline for Kinect processing")
                        self.pipeline = pipeline
                        break
                except subprocess.TimeoutExpired:
                    continue
                except Exception as e:
                    logger.warning(f"Pipeline {pipeline} failed: {e}")
                    continue
            else:
                logger.error("No working Kinect pipeline found")
                return False
            
            self.running = True
            self.capture_thread = threading.Thread(target=self._protonect_capture_loop)
            self.capture_thread.start()
            return True
            
        except Exception as e:
            logger.error(f"Failed to start Protonect capture: {e}")
            return False
    
    def _start_opencv_capture(self) -> bool:
        """Fallback to OpenCV capture (webcam simulation)"""
        try:
            self.rgb_cap = cv2.VideoCapture(self.device_id)
            self.rgb_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            self.rgb_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
            
            if not self.rgb_cap.isOpened():
                logger.error("Failed to open camera")
                return False
                
            logger.warning("Using OpenCV capture (no depth data available)")
            self.running = True
            self.capture_thread = threading.Thread(target=self._opencv_capture_loop)
            self.capture_thread.start()
            return True
            
        except Exception as e:
            logger.error(f"Failed to start OpenCV capture: {e}")
            return False
    
    def _protonect_capture_loop(self):
        """Capture loop using Protonect output"""
        # This is a simplified version - in practice, you'd need to:
        # 1. Parse Protonect output or use libfreenect2 Python bindings
        # 2. Extract RGB and depth frames from the data stream
        # 3. Handle synchronization between RGB and depth
        
        # For now, simulate with webcam + synthetic depth
        # Initialize webcam for fallback
        self.rgb_cap = cv2.VideoCapture(self.device_id)
        self.rgb_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.rgb_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        
        if self.rgb_cap.isOpened():
            self._opencv_capture_loop()
        else:
            logger.error("Failed to open camera for Protonect fallback")
    
    def _opencv_capture_loop(self):
        """Main capture loop using OpenCV"""
        frame_id = 0
        
        while self.running:
            try:
                ret, rgb_frame = self.rgb_cap.read()
                if not ret:
                    logger.warning("Failed to capture frame")
                    continue
                
                timestamp = time.time()
                frame_id += 1
                
                # Create synthetic depth for testing (replace with real depth from Kinect)
                height, width = rgb_frame.shape[:2]
                depth_frame = self._generate_synthetic_depth(width, height)
                
                # Put frames in queues (non-blocking)
                rgb_data = {
                    'frame': rgb_frame,
                    'timestamp': timestamp,
                    'frame_id': frame_id,
                    'intrinsics': self.rgb_intrinsics
                }
                
                depth_data = {
                    'frame': depth_frame,
                    'timestamp': timestamp,
                    'frame_id': frame_id,
                    'intrinsics': self.depth_intrinsics
                }
                
                self._put_frame_safe(self.rgb_queue, rgb_data)
                self._put_frame_safe(self.depth_queue, depth_data)
                
                time.sleep(1/30)  # 30 FPS
                
            except Exception as e:
                logger.error(f"Capture loop error: {e}")
                break
    
    def _put_frame_safe(self, frame_queue: queue.Queue, frame_data: Dict):
        """Safely put frame in queue, dropping oldest if full"""
        try:
            frame_queue.put_nowait(frame_data)
        except queue.Full:
            try:
                frame_queue.get_nowait()  # Drop oldest
                frame_queue.put_nowait(frame_data)
            except queue.Empty:
                pass
    
    def _generate_synthetic_depth(self, width: int, height: int) -> np.ndarray:
        """Generate synthetic depth data for testing"""
        # Create a simple depth gradient for testing
        depth = np.zeros((height, width), dtype=np.uint16)
        for y in range(height):
            depth[y, :] = int(1000 + (y / height) * 3000)  # 1-4 meter range
        return depth
    
    def get_rgb_frame(self) -> Optional[Dict]:
        """Get latest RGB frame"""
        if self.use_modern and self.modern_kinect:
            frame = self.modern_kinect.get_frame()
            if frame and frame.rgb is not None:
                return {
                    'frame': frame.rgb,
                    'timestamp': frame.timestamp,
                    'frame_id': frame.frame_id,
                    'intrinsics': self.rgb_intrinsics
                }
            return None
        else:
            # Legacy implementation
            try:
                return self.rgb_queue.get_nowait()
            except queue.Empty:
                return None
    
    def get_depth_frame(self) -> Optional[Dict]:
        """Get latest depth frame"""
        if self.use_modern and self.modern_kinect:
            frame = self.modern_kinect.get_frame()
            if frame and frame.depth is not None:
                return {
                    'frame': frame.depth,
                    'timestamp': frame.timestamp,
                    'frame_id': frame.frame_id,
                    'intrinsics': self.depth_intrinsics
                }
            return None
        else:
            # Legacy implementation
            try:
                return self.depth_queue.get_nowait()
            except queue.Empty:
                return None
    
    def get_synchronized_frames(self) -> Tuple[Optional[Dict], Optional[Dict]]:
        """Get synchronized RGB and depth frames"""
        if self.use_modern and self.modern_kinect:
            frame = self.modern_kinect.get_frame()
            if frame:
                rgb_data = None
                depth_data = None
                
                if frame.rgb is not None:
                    rgb_data = {
                        'frame': frame.rgb,
                        'timestamp': frame.timestamp,
                        'frame_id': frame.frame_id,
                        'intrinsics': self.rgb_intrinsics
                    }
                
                if frame.depth is not None:
                    depth_data = {
                        'frame': frame.depth,
                        'timestamp': frame.timestamp,
                        'frame_id': frame.frame_id,
                        'intrinsics': self.depth_intrinsics
                    }
                
                return rgb_data, depth_data
            return None, None
        else:
            # Legacy implementation
            rgb_frame = self.get_rgb_frame()
            depth_frame = self.get_depth_frame()
            return rgb_frame, depth_frame
    
    def get_camera_info(self) -> Dict[str, Any]:
        """Get camera calibration information"""
        if self.use_modern and self.modern_kinect:
            return self.modern_kinect.get_camera_info()
        else:
            # Legacy implementation
            return {
                'rgb_intrinsics': self.rgb_intrinsics,
                'depth_intrinsics': self.depth_intrinsics,
                'pipeline': getattr(self, 'pipeline', 'opencv'),
                'device_id': self.device_id
            }
    
    def stop(self):
        """Stop capture"""
        if self.use_modern and self.modern_kinect:
            self.modern_kinect.stop_streaming()
        else:
            # Legacy implementation
            self.running = False
            
            if self.capture_thread:
                self.capture_thread.join(timeout=5)
            
            if hasattr(self, 'rgb_cap'):
                self.rgb_cap.release()
            
            if self.protonect_process:
                self.protonect_process.terminate()
                self.protonect_process.wait()
    
    def is_connected(self) -> bool:
        """Check if Kinect is connected"""
        if self.use_modern and self.modern_kinect:
            return self.modern_kinect.is_connected()
        else:
            return self._check_protonect()
    
    def __enter__(self):
        """Context manager entry"""
        if self.start():
            return self
        else:
            raise RuntimeError("Failed to start Kinect interface")
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.stop()


# Test the interface
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    
    print("Testing Kinect v2 Interface...")
    
    with KinectV2Interface() as kinect:
        print("Kinect started successfully")
        print(f"Camera info: {kinect.get_camera_info()}")
        
        for i in range(100):  # Capture 100 frames
            rgb_data, depth_data = kinect.get_synchronized_frames()
            
            if rgb_data and depth_data:
                print(f"Frame {i}: RGB {rgb_data['timestamp']:.3f}, "
                      f"Depth {depth_data['timestamp']:.3f}")
                
                # Optional: Display frames
                if i % 10 == 0:  # Every 10th frame
                    cv2.imshow('RGB', rgb_data['frame'])
                    cv2.imshow('Depth', (depth_data['frame'] / 16).astype(np.uint8))
                    cv2.waitKey(1)
            
            time.sleep(0.1)
        
        cv2.destroyAllWindows()
        print("Test completed!")