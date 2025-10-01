# HowYouSeeMe ROS2 Setup Guide

## 🚀 Quick Setup

### 1. Run the Setup Script
```bash
./setup_kinect2_ros2.sh
```

This script will:
- ✅ Install all required ROS2 packages
- ✅ Install libfreenect2 (with CUDA option)
- ✅ Clone and build kinect2_ros2 package
- ✅ Create HowYouSeeMe ROS2 integration
- ✅ Build all packages
- ✅ Create launch scripts
- ✅ Set up RViz configuration

### 2. Test the Installation
```bash
./test_kinect2_ros2.sh
```

### 3. Source the Environment
```bash
source ~/.bashrc
# OR open a new terminal
```

## 🎮 Usage

### Option 1: Manual Launch (Recommended for Testing)

**Terminal 1 - Start Kinect Bridge:**
```bash
~/howyouseeme_scripts/start_kinect_bridge.sh
```

**Terminal 2 - Start HowYouSeeMe Processing:**
```bash
~/howyouseeme_scripts/start_howyouseeme.sh
```

**Terminal 3 - Monitor Performance:**
```bash
# Check topics
ros2 topic list | grep kinect2

# Monitor frame rate
ros2 topic hz /kinect2/hd/image_color

# View images
ros2 run rqt_image_view rqt_image_view /kinect2/hd/image_color
```

**Terminal 4 - Visualize in RViz2:**
```bash
rviz2 -d ~/howyouseeme_scripts/config/howyouseeme.rviz
```

### Option 2: Complete System Launch
```bash
~/howyouseeme_scripts/start_complete_system.sh
```

## 📊 Expected Performance

### ROS2 Topics Published:
- `/kinect2/hd/image_color` - RGB images (1920x1080)
- `/kinect2/hd/image_depth_rect` - Depth images (512x424)
- `/kinect2/hd/camera_info` - Camera calibration
- `/howyouseeme/pose` - SLAM pose estimates

### Performance Metrics:
- **Frame Rate**: 10-15 FPS (depending on processing load)
- **Latency**: Low latency for real-time applications
- **SLAM**: Pose tracking with our optimized pipeline
- **Detection**: Object detection every 10th frame

## 🔧 Troubleshooting

### Common Issues:

1. **"No Kinect device found"**
   ```bash
   # Check USB connection
   lsusb | grep 045e
   
   # Check permissions
   sudo usermod -a -G plugdev $USER
   # Then logout and login again
   ```

2. **"kinect2_bridge fails to start"**
   ```bash
   # Check libfreenect2 installation
   ldconfig -p | grep freenect2
   
   # Try rebuilding
   cd ~/ros2_ws
   colcon build --packages-select kinect2_bridge --cmake-clean-cache
   ```

3. **"No topics published"**
   ```bash
   # Check if bridge is running
   ros2 node list | grep kinect2
   
   # Check for errors
   ros2 topic echo /rosout
   ```

4. **"HowYouSeeMe components not found"**
   - The subscriber will work in basic mode without SLAM/detection
   - Make sure the HowYouSeeMe source code is in the correct path
   - Check the path in the subscriber node

### Performance Optimization:

1. **Reduce image resolution** (edit kinect2_bridge config)
2. **Adjust processing intervals** in the subscriber
3. **Use CUDA libfreenect2** for better performance
4. **Monitor CPU/GPU usage** with `htop` and `nvidia-smi`

## 📁 File Structure

After setup, you'll have:

```
~/ros2_ws/
├── src/
│   ├── kinect2_ros2/          # Kinect2 ROS2 bridge
│   └── howyouseeme_ros2/      # Our integration package
├── build/
├── install/
└── log/

~/howyouseeme_scripts/
├── start_kinect_bridge.sh     # Launch Kinect bridge
├── start_howyouseeme.sh       # Launch our subscriber
├── start_complete_system.sh   # Launch everything
└── config/
    └── howyouseeme.rviz       # RViz configuration
```

## 🎯 Next Steps

1. **Test basic functionality** with the provided scripts
2. **Calibrate your Kinect** if needed:
   ```bash
   ros2 run kinect2_calibration kinect2_calibration_node
   ```
3. **Integrate with RTABMap** for advanced SLAM:
   ```bash
   sudo apt install ros-jazzy-rtabmap-ros
   ```
4. **Add navigation** with Nav2:
   ```bash
   sudo apt install ros-jazzy-navigation2
   ```

## 🚀 Success Indicators

You'll know everything is working when:
- ✅ `ros2 topic list` shows kinect2 topics
- ✅ `ros2 topic hz /kinect2/hd/image_color` shows ~15 Hz
- ✅ RViz2 displays the camera feed
- ✅ HowYouSeeMe subscriber logs show processing statistics
- ✅ No error messages in the terminals

**Ready to explore computer vision with ROS2!** 🎉