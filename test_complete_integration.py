#!/usr/bin/env python3
"""
Complete Integration Test for HowYouSeeMe with Working Kinect
Tests the full pipeline with real RGB-D data from Kinect v2
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

import time
import cv2
import numpy as np
import logging
from typing import Dict, Any, Optional

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

def test_kinect_interface():
    """Test the updated Kinect interface"""
    print("🎮 Testing Updated Kinect Interface")
    print("=" * 50)
    
    try:
        from perception.modern_kinect_interface import ModernKinectInterface
        
        # Initialize with working settings
        kinect = ModernKinectInterface(
            preferred_pipeline="opengl",
            enable_rgb=True,
            enable_depth=True,
            enable_ir=False
        )
        
        print("✅ Kinect interface created")
        
        # Check connection
        if not kinect.is_connected():
            print("❌ Kinect not connected")
            return False
        
        print("✅ Kinect connected")
        
        # Get camera info
        camera_info = kinect.get_camera_info()
        print(f"🚀 Available pipelines: {camera_info['available_pipelines']}")
        print(f"📷 RGB: {camera_info['rgb_intrinsics']['width']}x{camera_info['rgb_intrinsics']['height']}")
        print(f"📷 Depth: {camera_info['depth_intrinsics']['width']}x{camera_info['depth_intrinsics']['height']}")
        
        # Test streaming
        print("🎬 Starting streaming...")
        if not kinect.start_streaming():
            print("❌ Failed to start streaming")
            return False
        
        print("✅ Streaming started")
        
        # Capture frames
        frame_count = 0
        rgb_frames = 0
        depth_frames = 0
        start_time = time.time()
        
        print("📸 Capturing frames for 10 seconds...")
        
        while time.time() - start_time < 10:  # 10 seconds
            frame = kinect.get_frame()
            
            if frame:
                frame_count += 1
                
                if frame.rgb is not None:
                    rgb_frames += 1
                if frame.depth is not None:
                    depth_frames += 1
                
                # Save first frame
                if frame_count == 1:
                    if frame.rgb is not None:
                        cv2.imwrite('test_rgb_frame.jpg', cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2BGR))
                        print("💾 Saved test_rgb_frame.jpg")
                    
                    if frame.depth is not None:
                        depth_8bit = (frame.depth / 16).astype(np.uint8)
                        cv2.imwrite('test_depth_frame.jpg', depth_8bit)
                        print("💾 Saved test_depth_frame.jpg")
                
                if frame_count % 30 == 0:
                    elapsed = time.time() - start_time
                    fps = frame_count / elapsed
                    print(f"📊 Frame {frame_count}: {fps:.1f} FPS, "
                          f"RGB: {frame.rgb is not None}, Depth: {frame.depth is not None}")
            
            time.sleep(0.01)  # Small delay
        
        elapsed = time.time() - start_time
        fps = frame_count / elapsed if elapsed > 0 else 0
        
        print(f"\n📊 Results:")
        print(f"   Total frames: {frame_count}")
        print(f"   RGB frames: {rgb_frames}")
        print(f"   Depth frames: {depth_frames}")
        print(f"   Average FPS: {fps:.1f}")
        
        kinect.stop_streaming()
        print("✅ Streaming stopped")
        
        return frame_count > 0 and rgb_frames > 0 and depth_frames > 0
        
    except Exception as e:
        print(f"❌ Kinect test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_sensor_interface():
    """Test the updated sensor interface wrapper"""
    print("\n🔌 Testing Sensor Interface Wrapper")
    print("=" * 50)
    
    try:
        from perception.sensor_interface import KinectV2Interface
        
        # Initialize with modern interface
        kinect = KinectV2Interface(
            use_modern=True,
            preferred_pipeline="opengl"
        )
        
        print("✅ Sensor interface created")
        
        # Check connection
        if not kinect.is_connected():
            print("❌ Kinect not connected")
            return False
        
        print("✅ Kinect connected")
        
        # Get camera info
        camera_info = kinect.get_camera_info()
        print(f"📷 Current pipeline: {camera_info.get('current_pipeline', 'Unknown')}")
        
        # Start streaming
        if not kinect.start():
            print("❌ Failed to start streaming")
            return False
        
        print("✅ Streaming started")
        
        # Test frame capture
        frame_count = 0
        sync_frames = 0
        
        print("📸 Testing synchronized frame capture...")
        
        for i in range(50):  # Test 50 frames
            rgb_data, depth_data = kinect.get_synchronized_frames()
            
            if rgb_data and depth_data:
                frame_count += 1
                sync_frames += 1
                
                if frame_count == 1:
                    print(f"✅ First synchronized frame:")
                    print(f"   RGB: {rgb_data['frame'].shape}")
                    print(f"   Depth: {depth_data['frame'].shape}")
                    print(f"   Timestamp diff: {abs(rgb_data['timestamp'] - depth_data['timestamp']):.3f}s")
                
                if frame_count % 10 == 0:
                    print(f"📊 Synchronized frame {frame_count}")
            
            time.sleep(0.1)
        
        print(f"\n📊 Sensor Interface Results:")
        print(f"   Synchronized frames: {sync_frames}/50")
        print(f"   Success rate: {sync_frames/50:.1%}")
        
        kinect.stop()
        print("✅ Sensor interface stopped")
        
        return sync_frames > 25  # At least 50% success rate
        
    except Exception as e:
        print(f"❌ Sensor interface test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_slam_integration():
    """Test SLAM integration with real Kinect data"""
    print("\n🗺️  Testing SLAM Integration")
    print("=" * 50)
    
    try:
        from perception.sensor_interface import KinectV2Interface
        from perception.slam.slam_interface import BasicSLAM
        
        # Initialize components
        kinect = KinectV2Interface(use_modern=True, preferred_pipeline="opengl")
        if not kinect.start():
            print("❌ Failed to start Kinect")
            return False
        
        # Initialize SLAM with real camera parameters
        camera_info = kinect.get_camera_info()
        slam = BasicSLAM(camera_info['rgb_intrinsics'])
        print("✅ SLAM initialized with real camera parameters")
        
        # Process frames
        frame_count = 0
        tracking_frames = 0
        feature_counts = []
        
        print("🎯 Processing real RGB-D frames with SLAM...")
        
        for i in range(100):  # Process 100 frames
            rgb_data, depth_data = kinect.get_synchronized_frames()
            
            if rgb_data and depth_data:
                rgb_frame = rgb_data['frame']
                depth_frame = depth_data['frame']
                
                # Process with SLAM
                slam_result = slam.process_frame(rgb_frame, depth_frame)
                frame_count += 1
                
                if slam_result['is_tracking']:
                    tracking_frames += 1
                
                feature_counts.append(slam_result['num_features'])
                
                if frame_count % 20 == 0:
                    tracking_rate = tracking_frames / frame_count
                    avg_features = np.mean(feature_counts[-20:])
                    print(f"📊 Frame {frame_count}: Tracking={slam_result['is_tracking']}, "
                          f"Features={slam_result['num_features']:.0f}, Rate={tracking_rate:.1%}")
                
                # Save visualization of first tracking frame
                if slam_result['is_tracking'] and tracking_frames == 1:
                    vis_frame = rgb_frame.copy()
                    if slam_result['keypoints']:
                        vis_frame = cv2.drawKeypoints(vis_frame, slam_result['keypoints'], None,
                                                    color=(0, 255, 0), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                    cv2.imwrite('slam_tracking_frame.jpg', cv2.cvtColor(vis_frame, cv2.COLOR_RGB2BGR))
                    print("💾 Saved slam_tracking_frame.jpg")
            
            time.sleep(0.05)  # 20 FPS
        
        tracking_rate = tracking_frames / frame_count if frame_count > 0 else 0
        avg_features = np.mean(feature_counts) if feature_counts else 0
        
        print(f"\n📊 SLAM Results:")
        print(f"   Frames processed: {frame_count}")
        print(f"   Tracking frames: {tracking_frames}")
        print(f"   Tracking rate: {tracking_rate:.1%}")
        print(f"   Average features: {avg_features:.1f}")
        
        # Get trajectory
        trajectory = slam.get_trajectory()
        print(f"   Trajectory points: {len(trajectory)}")
        
        kinect.stop()
        return tracking_rate > 0.3  # At least 30% tracking
        
    except Exception as e:
        print(f"❌ SLAM integration test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_visualization():
    """Test real-time visualization with Kinect data"""
    print("\n🎨 Testing Real-time Visualization")
    print("=" * 50)
    
    try:
        from perception.sensor_interface import KinectV2Interface
        from perception.slam.slam_interface import BasicSLAM
        
        # Initialize
        kinect = KinectV2Interface(use_modern=True)
        if not kinect.start():
            print("❌ Failed to start Kinect")
            return False
        
        camera_info = kinect.get_camera_info()
        slam = BasicSLAM(camera_info['rgb_intrinsics'])
        
        print("✅ Components initialized")
        print("🎬 Starting real-time visualization (press 'q' to quit, 'r' to reset SLAM)")
        
        frame_count = 0
        start_time = time.time()
        
        while True:
            rgb_data, depth_data = kinect.get_synchronized_frames()
            
            if rgb_data and depth_data:
                rgb_frame = rgb_data['frame']
                depth_frame = depth_data['frame']
                frame_count += 1
                
                # Process with SLAM
                slam_result = slam.process_frame(rgb_frame, depth_frame)
                
                # Create visualization
                vis_frame = rgb_frame.copy()
                
                # Draw SLAM features
                if slam_result['keypoints']:
                    color = (0, 255, 0) if slam_result['is_tracking'] else (0, 0, 255)
                    vis_frame = cv2.drawKeypoints(vis_frame, slam_result['keypoints'], None,
                                                color=color, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                
                # Add status overlay
                elapsed = time.time() - start_time
                fps = frame_count / elapsed if elapsed > 0 else 0
                
                # Status text
                status_text = [
                    f"FPS: {fps:.1f}",
                    f"Frame: {frame_count}",
                    f"SLAM: {'TRACKING' if slam_result['is_tracking'] else 'LOST'}",
                    f"Features: {slam_result['num_features']}",
                    f"Pipeline: {camera_info.get('current_pipeline', 'Unknown')}"
                ]
                
                # Draw status
                y_offset = 30
                for text in status_text:
                    cv2.putText(vis_frame, text, (10, y_offset), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.putText(vis_frame, text, (10, y_offset), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 1)
                    y_offset += 30
                
                # Convert RGB to BGR for display
                vis_frame_bgr = cv2.cvtColor(vis_frame, cv2.COLOR_RGB2BGR)
                
                # Show frames
                cv2.imshow('HowYouSeeMe - RGB + SLAM', vis_frame_bgr)
                
                # Show depth
                depth_vis = cv2.applyColorMap((depth_frame / 16).astype(np.uint8), cv2.COLORMAP_JET)
                cv2.imshow('HowYouSeeMe - Depth', depth_vis)
                
                # Show SLAM trajectory
                if frame_count % 10 == 0:  # Update every 10 frames
                    traj_img = slam.visualize_trajectory()
                    cv2.imshow('SLAM Trajectory', traj_img)
                
                # Handle keys
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("🛑 Quit requested")
                    break
                elif key == ord('r'):
                    slam.reset()
                    print("🔄 SLAM reset")
                
                # Auto-quit after 30 seconds for testing
                if elapsed > 30:
                    print("⏰ 30 second test completed")
                    break
        
        cv2.destroyAllWindows()
        kinect.stop()
        
        print(f"✅ Visualization test completed ({frame_count} frames)")
        return True
        
    except Exception as e:
        print(f"❌ Visualization test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Run complete integration tests"""
    print("🎯 HowYouSeeMe Complete Integration Test")
    print("=" * 60)
    print("Testing the complete pipeline with real Kinect v2 data:")
    print("- Modern Kinect interface with working frame capture")
    print("- Sensor interface wrapper")
    print("- SLAM integration with real RGB-D data")
    print("- Real-time visualization")
    print()
    
    tests = [
        ("Modern Kinect Interface", test_kinect_interface),
        ("Sensor Interface Wrapper", test_sensor_interface),
        ("SLAM Integration", test_slam_integration),
        ("Real-time Visualization", test_visualization)
    ]
    
    results = {}
    
    for test_name, test_func in tests:
        print(f"\n🧪 Running: {test_name}")
        try:
            results[test_name] = test_func()
        except Exception as e:
            print(f"❌ Test '{test_name}' crashed: {e}")
            results[test_name] = False
        
        time.sleep(2)  # Brief pause between tests
    
    # Summary
    print("\n" + "=" * 60)
    print("🎯 COMPLETE INTEGRATION TEST RESULTS")
    print("=" * 60)
    
    passed = 0
    total = len(tests)
    
    for test_name, result in results.items():
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{status} {test_name}")
        if result:
            passed += 1
    
    print(f"\nOverall: {passed}/{total} tests passed ({passed/total:.1%})")
    
    if passed == total:
        print("🎉 COMPLETE SUCCESS! Your HowYouSeeMe system is fully operational!")
        print("🚀 Ready for advanced computer vision development!")
    elif passed >= 3:
        print("🎊 Excellent! Core functionality is working perfectly!")
    elif passed >= 2:
        print("👍 Good progress! Most components are working.")
    else:
        print("⚠️  Some issues remain. Check the failed tests above.")
    
    print(f"\n📁 Generated files:")
    print("   - test_rgb_frame.jpg (RGB capture)")
    print("   - test_depth_frame.jpg (Depth capture)")
    print("   - slam_tracking_frame.jpg (SLAM features)")
    
    return passed >= 3  # Success if at least 3/4 tests pass

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)