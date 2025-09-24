"""
Minimal Kinect-only test for maximum performance
Tests just the Kinect streaming without SLAM or detection overhead
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

import time
import cv2
import numpy as np
import logging
from perception.sensor_interface import KinectV2Interface

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def test_kinect_performance():
    """Test raw Kinect performance without any processing"""
    print("Kinect Performance Test")
    print("======================")
    print("Testing raw Kinect streaming performance")
    print("Press 'q' to quit")
    print()
    
    # Initialize Kinect
    kinect = KinectV2Interface(use_modern=True, preferred_pipeline="opengl")
    
    if not kinect.start():
        print("Failed to initialize Kinect")
        return
    
    print(f"Kinect info: {kinect.get_camera_info()}")
    
    frame_count = 0
    start_time = time.time()
    
    try:
        while True:
            # Get frames
            rgb_data, depth_data = kinect.get_synchronized_frames()
            
            if rgb_data is None:
                time.sleep(0.001)
                continue
            
            frame_count += 1
            
            # Show FPS every 60 frames
            if frame_count % 60 == 0:
                elapsed = time.time() - start_time
                fps = frame_count / elapsed
                print(f"Frame {frame_count}: {fps:.1f} FPS")
            
            # Minimal visualization
            if frame_count % 3 == 0:  # Every 3rd frame
                rgb_frame = rgb_data['frame']
                
                # Add FPS counter
                elapsed = time.time() - start_time
                fps = frame_count / elapsed if elapsed > 0 else 0
                cv2.putText(rgb_frame, f"FPS: {fps:.1f}", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # Resize for display (optional)
                display_frame = cv2.resize(rgb_frame, (960, 540))
                cv2.imshow('Kinect Raw Performance', display_frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    
    except KeyboardInterrupt:
        print("Test interrupted")
    
    finally:
        # Final stats
        elapsed = time.time() - start_time
        final_fps = frame_count / elapsed
        
        print(f"\nFinal Results:")
        print(f"Total frames: {frame_count}")
        print(f"Total time: {elapsed:.2f}s")
        print(f"Average FPS: {final_fps:.1f}")
        
        kinect.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    test_kinect_performance()