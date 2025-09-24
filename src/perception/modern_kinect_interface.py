#!/usr/bin/env python3
"""
Modern Kinect v2 Interface using pylibfreenect2-py310
Provides high-performance RGB-D data acquisition with GPU acceleration
"""

import numpy as np
import cv2
import logging
import threading
import queue
import time
from typing import Optional, Tuple, Dict, Any, List
from dataclasses import dataclass
from pathlib import Path

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@dataclass
class KinectFrame:
    """Container for Kinect frame data"""
    rgb: Optional[np.ndarray] = None
    depth: Optional[np.ndarray] = None
    ir: Optional[np.ndarray] = None
    timestamp: float = 0.0
    frame_id: int = 0

class ModernKinectInterface:
    """
    Modern Kinect v2 interface using pylibfreenect2-py310
    
    Features:
    - Python 3.10+ compatibility
    - GPU acceleration (CUDA, OpenCL, OpenGL)
    - Automatic pipeline selection
    - Thread-safe frame acquisition
    - Proper resource management
    - Intelligent frame dropping for performance
    """
    
    def __init__(self, 
                 preferred_pipeline: str = "auto",
                 enable_rgb: bool = True,
                 enable_depth: bool = True,
                 enable_ir: bool = False,
                 enable_frame_dropping: bool = True,
                 max_frame_age_ms: float = 100.0,
                 target_fps: float = 15.0):
        """
        Initialize modern Kinect interface
        
        Args:
            preferred_pipeline: 'auto', 'cuda', 'opencl', 'opengl', 'cpu'
            enable_rgb: Enable RGB stream
            enable_depth: Enable depth stream  
            enable_ir: Enable IR stream
            enable_frame_dropping: Enable intelligent frame dropping
            max_frame_age_ms: Maximum age of frames before dropping (ms)
            target_fps: Target FPS for frame dropping logic
        """
        self.preferred_pipeline = preferred_pipeline
        self.enable_rgb = enable_rgb
        self.enable_depth = enable_depth
        self.enable_ir = enable_ir
        
        # Frame dropping configuration
        self.enable_frame_dropping = enable_frame_dropping
        self.max_frame_age_ms = max_frame_age_ms
        self.target_fps = target_fps
        self.min_frame_interval = 1.0 / target_fps if target_fps > 0 else 0.0
        
        # Kinect objects (initialized in start())
        self.fn = None
        self.device = None
        self.listener = None
        self.pipeline = None
        
        # Frame management with dropping logic
        self.frame_queue = queue.Queue(maxsize=10)  # Larger queue for dropping
        self.frame_count = 0
        self.dropped_frame_count = 0
        self.is_streaming = False
        self.last_frame_time = 0.0
        
        # Threading
        self._capture_thread = None
        self._stop_event = threading.Event()
        
        # Performance tracking
        self.frame_times = []
        self.processing_times = []
        
        # Kinect v2 specifications
        self.rgb_width = 1920
        self.rgb_height = 1080
        self.depth_width = 512
        self.depth_height = 424
        
        # Camera intrinsics (Kinect v2 factory calibration)
        self.rgb_intrinsics = {
            'fx': 1081.37, 'fy': 1081.37,
            'cx': 959.5, 'cy': 539.5,
            'width': self.rgb_width,
            'height': self.rgb_height
        }
        
        self.depth_intrinsics = {
            'fx': 365.456, 'fy': 365.456,
            'cx': 257.128, 'cy': 210.468,
            'width': self.depth_width,
            'height': self.depth_height
        }
        
        logger.info(f"Modern Kinect interface initialized (pipeline: {preferred_pipeline})")
    
    def check_pylibfreenect2_available(self) -> bool:
        """Check if pylibfreenect2 is available and working"""
        try:
            import pylibfreenect2
            
            # Test basic functionality
            fn = pylibfreenect2.Freenect2()
            num_devices = fn.enumerateDevices()
            
            logger.info(f"pylibfreenect2 available, {num_devices} devices detected")
            return True
            
        except ImportError as e:
            logger.error(f"pylibfreenect2 not available: {e}")
            return False
        except Exception as e:
            logger.error(f"pylibfreenect2 test failed: {e}")
            return False
    
    def get_available_pipelines(self) -> List[str]:
        """Get list of available GPU pipelines"""
        if not self.check_pylibfreenect2_available():
            return []
        
        import pylibfreenect2
        
        pipelines = []
        
        # Test each pipeline (check if they exist first)
        pipeline_classes = []
        
        if hasattr(pylibfreenect2, 'CudaPacketPipeline'):
            pipeline_classes.append(('cuda', pylibfreenect2.CudaPacketPipeline))
        if hasattr(pylibfreenect2, 'OpenCLPacketPipeline'):
            pipeline_classes.append(('opencl', pylibfreenect2.OpenCLPacketPipeline))
        if hasattr(pylibfreenect2, 'OpenGLPacketPipeline'):
            pipeline_classes.append(('opengl', pylibfreenect2.OpenGLPacketPipeline))
        if hasattr(pylibfreenect2, 'CpuPacketPipeline'):
            pipeline_classes.append(('cpu', pylibfreenect2.CpuPacketPipeline))
        
        for name, pipeline_class in pipeline_classes:
            try:
                pipeline = pipeline_class()
                pipelines.append(name)
                logger.debug(f"✅ {name.upper()} pipeline available")
            except Exception as e:
                logger.debug(f"❌ {name.upper()} pipeline not available: {e}")
        
        return pipelines
    
    def _select_best_pipeline(self) -> Optional[object]:
        """Select the best available pipeline"""
        if not self.check_pylibfreenect2_available():
            return None
        
        import pylibfreenect2
        
        # Pipeline preference order (fastest to slowest) - check availability first
        pipeline_order = []
        
        # Check for CUDA support (may not be available in all builds)
        if hasattr(pylibfreenect2, 'CudaPacketPipeline'):
            pipeline_order.append(('cuda', pylibfreenect2.CudaPacketPipeline))
        # Check for OpenCL support
        if hasattr(pylibfreenect2, 'OpenCLPacketPipeline'):
            pipeline_order.append(('opencl', pylibfreenect2.OpenCLPacketPipeline))
        # OpenGL is usually available and provides good GPU acceleration
        if hasattr(pylibfreenect2, 'OpenGLPacketPipeline'):
            pipeline_order.append(('opengl', pylibfreenect2.OpenGLPacketPipeline))
        # CPU fallback
        if hasattr(pylibfreenect2, 'CpuPacketPipeline'):
            pipeline_order.append(('cpu', pylibfreenect2.CpuPacketPipeline))
        
        # If specific pipeline requested, try it first
        if self.preferred_pipeline != "auto":
            for name, pipeline_class in pipeline_order:
                if name == self.preferred_pipeline.lower():
                    try:
                        pipeline = pipeline_class()
                        logger.info(f"✅ Using requested {name.upper()} pipeline")
                        return pipeline
                    except Exception as e:
                        logger.warning(f"Requested {name.upper()} pipeline failed: {e}")
                        break
        
        # Try pipelines in order of preference
        for name, pipeline_class in pipeline_order:
            try:
                pipeline = pipeline_class()
                logger.info(f"✅ Using {name.upper()} pipeline")
                return pipeline
            except Exception as e:
                logger.debug(f"{name.upper()} pipeline not available: {e}")
        
        logger.error("No working pipeline found!")
        return None
    
    def start_streaming(self) -> bool:
        """Start streaming from Kinect v2"""
        if self.is_streaming:
            logger.warning("Already streaming")
            return True
        
        try:
            import pylibfreenect2
            
            # Initialize Freenect2
            self.fn = pylibfreenect2.Freenect2()
            
            # Check for devices
            num_devices = self.fn.enumerateDevices()
            if num_devices == 0:
                logger.error("No Kinect v2 devices found")
                return False
            
            logger.info(f"Found {num_devices} Kinect v2 device(s)")
            
            # Select pipeline
            self.pipeline = self._select_best_pipeline()
            if not self.pipeline:
                logger.error("No working pipeline available")
                return False
            
            # Open device
            self.device = self.fn.openDefaultDevice(self.pipeline)
            if not self.device:
                logger.error("Failed to open Kinect device")
                return False
            
            # Set up frame listener
            frame_types = 0
            if self.enable_rgb:
                frame_types |= pylibfreenect2.FrameType.Color
            if self.enable_depth:
                frame_types |= pylibfreenect2.FrameType.Depth
            if self.enable_ir:
                frame_types |= pylibfreenect2.FrameType.Ir
            
            self.listener = pylibfreenect2.SyncMultiFrameListener(frame_types)
            
            # Register listeners
            if self.enable_rgb:
                self.device.setColorFrameListener(self.listener)
            if self.enable_depth or self.enable_ir:
                self.device.setIrAndDepthFrameListener(self.listener)
            
            # Start device
            self.device.start()
            
            # Start capture thread
            self._stop_event.clear()
            self._capture_thread = threading.Thread(target=self._capture_loop)
            self._capture_thread.daemon = True
            self._capture_thread.start()
            
            self.is_streaming = True
            logger.info("✅ Kinect v2 streaming started")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start streaming: {e}")
            self._cleanup()
            return False
    
    def stop_streaming(self):
        """Stop streaming from Kinect v2"""
        if not self.is_streaming:
            return
        
        logger.info("Stopping Kinect v2 streaming...")
        
        # Signal stop
        self._stop_event.set()
        
        # Wait for capture thread
        if self._capture_thread:
            self._capture_thread.join(timeout=5)
        
        # Cleanup resources
        self._cleanup()
        
        self.is_streaming = False
        logger.info("✅ Kinect v2 streaming stopped")
    
    def _capture_loop(self):
        """Main capture loop with intelligent frame dropping"""
        import pylibfreenect2
        
        try:
            while not self._stop_event.is_set():
                # Create FrameMap for receiving frames
                frames = pylibfreenect2.FrameMap()
                
                # Wait for new frame (with timeout)
                if not self.listener.waitForNewFrame(frames, 1000):  # 1 second timeout
                    continue
                
                try:
                    current_time = time.time()
                    
                    # Frame dropping logic
                    if self.enable_frame_dropping:
                        # Check if we should drop this frame based on timing
                        if self._should_drop_frame(current_time):
                            self.dropped_frame_count += 1
                            continue
                    
                    # Create frame data
                    frame_data = KinectFrame()
                    frame_data.timestamp = current_time
                    frame_data.frame_id = self.frame_count
                    self.frame_count += 1
                    self.last_frame_time = current_time
                    
                    # Extract RGB frame using integer frame type (Color = 1)
                    if self.enable_rgb:
                        try:
                            color_frame = frames[1]  # FrameType.Color = 1
                            color_array = color_frame.asarray()
                            # Convert BGRA to RGB (remove alpha and swap channels)
                            if color_array.shape[2] == 4:  # BGRA
                                frame_data.rgb = cv2.cvtColor(color_array, cv2.COLOR_BGRA2RGB)
                            else:  # Already RGB
                                frame_data.rgb = color_array
                        except KeyError:
                            # No color frame available
                            pass
                    
                    # Extract depth frame using integer frame type (Depth = 4)
                    if self.enable_depth:
                        try:
                            depth_frame = frames[4]  # FrameType.Depth = 4
                            frame_data.depth = depth_frame.asarray()
                        except KeyError:
                            # No depth frame available
                            pass
                    
                    # Extract IR frame using integer frame type (Ir = 2)
                    if self.enable_ir:
                        try:
                            ir_frame = frames[2]  # FrameType.Ir = 2
                            frame_data.ir = ir_frame.asarray()
                        except KeyError:
                            # No IR frame available
                            pass
                    
                    # Only queue frames that have data
                    if frame_data.rgb is not None or frame_data.depth is not None or frame_data.ir is not None:
                        # Intelligent queue management with frame dropping
                        self._queue_frame_with_dropping(frame_data)
                    
                finally:
                    # Always release frames
                    self.listener.release(frames)
                    
        except Exception as e:
            logger.error(f"Capture loop error: {e}")
    
    def _should_drop_frame(self, current_time: float) -> bool:
        """Determine if current frame should be dropped based on timing and queue state"""
        # Check minimum frame interval (target FPS)
        if self.last_frame_time > 0:
            time_since_last = current_time - self.last_frame_time
            if time_since_last < self.min_frame_interval:
                return True
        
        # Check queue fullness - drop if queue is getting full
        queue_size = self.frame_queue.qsize()
        if queue_size > 7:  # Drop if queue > 70% full
            return True
        
        return False
    
    def _queue_frame_with_dropping(self, frame_data: KinectFrame):
        """Queue frame with intelligent dropping logic"""
        current_time = time.time()
        
        # Clean old frames from queue first
        self._clean_old_frames(current_time)
        
        try:
            # Try to put frame in queue
            self.frame_queue.put_nowait(frame_data)
        except queue.Full:
            # Queue is full - implement dropping strategy
            dropped_count = 0
            
            # Drop oldest frames until we have space or queue is empty
            while not self.frame_queue.empty() and dropped_count < 5:
                try:
                    old_frame = self.frame_queue.get_nowait()
                    dropped_count += 1
                    self.dropped_frame_count += 1
                except queue.Empty:
                    break
            
            # Try to queue the new frame again
            try:
                self.frame_queue.put_nowait(frame_data)
            except queue.Full:
                # Still full, drop this frame
                self.dropped_frame_count += 1
    
    def _clean_old_frames(self, current_time: float):
        """Remove frames that are too old from the queue"""
        if not self.enable_frame_dropping:
            return
        
        max_age_seconds = self.max_frame_age_ms / 1000.0
        frames_to_keep = []
        dropped_count = 0
        
        # Check all frames in queue
        while not self.frame_queue.empty():
            try:
                frame = self.frame_queue.get_nowait()
                frame_age = current_time - frame.timestamp
                
                if frame_age <= max_age_seconds:
                    frames_to_keep.append(frame)
                else:
                    dropped_count += 1
                    self.dropped_frame_count += 1
            except queue.Empty:
                break
        
        # Put back the frames we want to keep
        for frame in frames_to_keep:
            try:
                self.frame_queue.put_nowait(frame)
            except queue.Full:
                # If queue is full, drop the oldest frames
                self.dropped_frame_count += 1
    
    def get_frame(self) -> Optional[KinectFrame]:
        """Get the latest frame with age checking"""
        if not self.is_streaming:
            logger.warning("Not streaming - call start_streaming() first")
            return None
        
        current_time = time.time()
        
        # Clean old frames first
        if self.enable_frame_dropping:
            self._clean_old_frames(current_time)
        
        try:
            frame = self.frame_queue.get_nowait()
            
            # Check if frame is too old
            if self.enable_frame_dropping:
                frame_age = current_time - frame.timestamp
                max_age_seconds = self.max_frame_age_ms / 1000.0
                
                if frame_age > max_age_seconds:
                    self.dropped_frame_count += 1
                    return None
            
            return frame
        except queue.Empty:
            return None
    
    def get_latest_frame(self) -> Optional[KinectFrame]:
        """Get the most recent frame, dropping all older ones"""
        if not self.is_streaming:
            return None
        
        latest_frame = None
        dropped_count = 0
        
        # Get all frames and keep only the latest
        while True:
            try:
                frame = self.frame_queue.get_nowait()
                if latest_frame is not None:
                    dropped_count += 1
                latest_frame = frame
            except queue.Empty:
                break
        
        self.dropped_frame_count += dropped_count
        return latest_frame
    
    def get_camera_info(self) -> Dict[str, Any]:
        """Get camera calibration information and performance stats"""
        # Only get available pipelines if not streaming to avoid creating new instances
        if not self.is_streaming:
            available_pipelines = self.get_available_pipelines()
        else:
            available_pipelines = ['unknown']  # Don't check while streaming
        
        current_pipeline = 'None'
        if self.pipeline:
            current_pipeline = self.pipeline.__class__.__name__
        
        # Calculate frame dropping statistics
        total_frames = self.frame_count + self.dropped_frame_count
        drop_rate = (self.dropped_frame_count / total_frames * 100) if total_frames > 0 else 0.0
        
        return {
            'rgb_intrinsics': self.rgb_intrinsics,
            'depth_intrinsics': self.depth_intrinsics,
            'available_pipelines': available_pipelines,
            'current_pipeline': current_pipeline,
            'streaming': self.is_streaming,
            'frame_count': self.frame_count,
            'dropped_frame_count': self.dropped_frame_count,
            'drop_rate_percent': drop_rate,
            'frame_dropping_enabled': self.enable_frame_dropping,
            'target_fps': self.target_fps,
            'max_frame_age_ms': self.max_frame_age_ms,
            'queue_size': self.frame_queue.qsize()
        }
    
    def is_connected(self) -> bool:
        """Check if Kinect v2 is connected"""
        try:
            if not self.check_pylibfreenect2_available():
                return False
            
            import pylibfreenect2
            fn = pylibfreenect2.Freenect2()
            return fn.enumerateDevices() > 0
            
        except Exception:
            return False
    
    def _cleanup(self):
        """Clean up resources"""
        try:
            if self.device:
                self.device.stop()
                self.device.close()
                self.device = None
            
            self.listener = None
            self.pipeline = None
            self.fn = None
            
        except Exception as e:
            logger.warning(f"Cleanup error: {e}")
    
    def __enter__(self):
        """Context manager entry"""
        if self.start_streaming():
            return self
        else:
            raise RuntimeError("Failed to start Kinect streaming")
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.stop_streaming()

def test_modern_kinect():
    """Test the modern Kinect interface"""
    logger.info("Testing Modern Kinect v2 Interface with pylibfreenect2-py310")
    logger.info("=" * 60)
    
    # Test 1: Check pylibfreenect2 availability
    kinect = ModernKinectInterface()
    
    if not kinect.check_pylibfreenect2_available():
        logger.error("❌ pylibfreenect2 not available")
        logger.info("Please install it with: pip install git+https://github.com/cerealkiller2527/pylibfreenect2-py310.git")
        return False
    
    logger.info("✅ pylibfreenect2 is available")
    
    # Test 2: Check available pipelines
    pipelines = kinect.get_available_pipelines()
    logger.info(f"🚀 Available pipelines: {pipelines}")
    
    if not pipelines:
        logger.error("❌ No pipelines available")
        return False
    
    # Test 3: Check device connection
    if not kinect.is_connected():
        logger.error("❌ No Kinect v2 devices found")
        logger.info("Make sure your Kinect v2 is connected and has proper permissions")
        return False
    
    logger.info("✅ Kinect v2 device detected")
    
    # Test 4: Camera info
    camera_info = kinect.get_camera_info()
    logger.info(f"📷 RGB: {camera_info['rgb_intrinsics']['width']}x{camera_info['rgb_intrinsics']['height']}")
    logger.info(f"📷 Depth: {camera_info['depth_intrinsics']['width']}x{camera_info['depth_intrinsics']['height']}")
    
    # Test 5: Streaming test
    logger.info("🎬 Testing streaming (10 seconds)...")
    try:
        with kinect:
            logger.info("✅ Streaming started")
            
            frames_received = 0
            start_time = time.time()
            
            for i in range(100):  # Try to get 100 frames
                frame = kinect.get_frame()
                if frame:
                    frames_received += 1
                    if frames_received % 10 == 0:
                        logger.info(f"📸 Frame {frame.frame_id}: RGB={frame.rgb is not None}, "
                                  f"Depth={frame.depth is not None}, IR={frame.ir is not None}")
                
                time.sleep(0.1)
                
                if time.time() - start_time > 10:  # 10 second limit
                    break
            
            elapsed = time.time() - start_time
            fps = frames_received / elapsed
            
            logger.info(f"✅ Streaming test completed")
            logger.info(f"📊 Received {frames_received} frames in {elapsed:.1f}s ({fps:.1f} FPS)")
            
            if frames_received > 0:
                logger.info("🎉 Modern Kinect interface is working perfectly!")
                return True
            else:
                logger.warning("⚠️  No frames received - check Kinect connection")
                return False
            
    except Exception as e:
        logger.error(f"❌ Streaming test failed: {e}")
        return False

if __name__ == "__main__":
    test_modern_kinect()