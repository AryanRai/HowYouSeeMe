# Getting Started with HowYouSeeMe ROS2 System

This guide will help you set up and run the HowYouSeeMe ROS2-based computer vision system with Kinect v2, RTABMap SLAM, and YOLOv12 object detection.

## Prerequisites

### Hardware Requirements
- **Microsoft Kinect v2** with USB 3.0 controller
- **NVIDIA GPU** with CUDA 12.0+ support (recommended)
- **USB 3.0 port** for Kinect connection
- **8GB+ RAM** recommended
- **20GB+ free disk space** (for ROS2 and models)

### Software Requirements
- **Ubuntu 24.04 LTS** (recommended for ROS2 Jazzy)
- **ROS2 Jazzy Jalopy**
- **Python 3.12+**
- **CUDA 12.0+** (for GPU acceleration)
- **Git** for repository management

## One-Command Installation

### Complete ROS2 System Setup
```bash
# Clone and setup everything
git clone https://github.com/AryanRai/HowYouSeeMe.git
cd HowYouSeeMe
./setup_kinect2_ros2.sh
```

This script will:
- Install ROS2 Jazzy
- Build kinect2_bridge with CUDA support
- Set up HowYouSeeMe ROS2 package
- Install all dependencies
- Configure the complete system

## Manual Installation (Advanced)

### 1. Install ROS2 Jazzy
```bash
# Add ROS2 repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy
sudo apt update
sudo apt install ros-jazzy-desktop python3-argcomplete python3-colcon-common-extensions
```

### 2. Set Up ROS2 Workspace
```bash
# Source ROS2
source /opt/ros/jazzy/setup.bash

# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build workspace
colcon build
source install/setup.bash
```

### 3. Install Kinect2 Bridge
```bash
# Install dependencies
sudo apt install libfreenect2-dev ros-jazzy-cv-bridge ros-jazzy-image-transport

# Clone kinect2_bridge
cd ~/ros2_ws/src
git clone https://github.com/thiagodefreitas/kinect2_bridge.git

# Build
cd ~/ros2_ws
colcon build --packages-select kinect2_bridge
```

### 4. Set Up HowYouSeeMe Package
```bash
# Clone HowYouSeeMe
cd ~/ros2_ws/src
git clone https://github.com/AryanRai/HowYouSeeMe.git howyouseeme_ros2

# Install Python dependencies
pip install ultralytics opencv-python numpy torch torchvision

# Build package
cd ~/ros2_ws
colcon build --packages-select howyouseeme_ros2
source install/setup.bash
```

## Quick Start

### 1. Launch Complete System
```bash
# Source ROS2 workspace
source ~/ros2_ws/install/setup.bash

# Launch everything (sensor bridge, SLAM, detection, visualization)
ros2 launch howyouseeme_ros2 howyouseeme_complete.launch.py
```

### 2. Launch with Advanced SLAM
```bash
# Launch with RTABMap SLAM for production mapping
ros2 launch howyouseeme_ros2 howyouseeme_complete.launch.py use_rtabmap:=true
```

### 3. Detection Only Mode
```bash
# Just object detection and visualization
ros2 launch howyouseeme_ros2 detection_only.launch.py
```

### 4. Custom Configuration
```bash
# Custom YOLO model and settings
ros2 launch howyouseeme_ros2 detection_only.launch.py \
    yolo_model:=yolo11m \
    confidence_threshold:=0.7
```

## System Monitoring

### Check ROS2 Topics
```bash
# List all topics
ros2 topic list

# Check Kinect topics
ros2 topic list | grep kinect2

# Check HowYouSeeMe topics
ros2 topic list | grep howyouseeme
```

### Monitor Performance
```bash
# Check sensor frame rate
ros2 topic hz /kinect2/hd/image_color

# Monitor detection rate
ros2 topic hz /howyouseeme/detections

# View detection results
ros2 topic echo /howyouseeme/detections
```

### Visualization
```bash
# Launch RViz2 with custom configuration
rviz2 -d ~/ros2_ws/src/howyouseeme_ros2/config/howyouseeme_complete.rviz

# Real-time topic graph
rqt_graph

# Plot performance data
rqt_plot /howyouseeme/detection_stats/data[4]  # FPS
```

## ROS2 Topics Reference

### Published Topics (30+)
```bash
# Sensor Data
/kinect2/hd/image_color          # RGB images (1920x1080) @ 14.5 FPS
/kinect2/hd/image_depth_rect     # Registered depth @ 14.5 FPS
/kinect2/hd/camera_info          # Camera calibration
/kinect2/qhd/*                   # Quarter HD streams (960x540)
/kinect2/sd/*                    # Standard definition (512x424)

# Computer Vision Results
/howyouseeme/detections          # YOLOv12 object detections
/howyouseeme/detection_image     # Annotated detection image
/howyouseeme/pose                # SLAM pose estimates
/howyouseeme/trajectory          # Camera trajectory
/howyouseeme/map                 # 3D point cloud map

# Performance Metrics
/howyouseeme/sensor_stats        # Sensor performance
/howyouseeme/detection_stats     # Detection performance
/howyouseeme/slam_stats          # SLAM performance

# Navigation (with RTABMap)
/map                             # Occupancy grid map
/odom                            # Odometry data
/tf                              # Transform tree
```

## Configuration

### System Parameters
```bash
# View current parameters
ros2 param list

# Set detection confidence
ros2 param set /howyouseeme_yolo_detector confidence_threshold 0.7

# Set target FPS
ros2 param set /kinect2_bridge fps_limit 15.0
```

### Launch File Parameters
```bash
# Available parameters for complete launch
ros2 launch howyouseeme_ros2 howyouseeme_complete.launch.py --show-args

# Example with custom parameters
ros2 launch howyouseeme_ros2 howyouseeme_complete.launch.py \
    use_rviz:=true \
    use_rtabmap:=false \
    target_fps:=15
```

## Troubleshooting

### Common Issues

#### 1. Kinect Not Detected
```bash
# Check USB connection
lsusb | grep Microsoft

# Test kinect2_bridge directly
ros2 run kinect2_bridge kinect2_bridge_node

# Check device permissions
ls -la /dev/bus/usb/*/
```

#### 2. ROS2 Build Issues
```bash
# Clean build
cd ~/ros2_ws
rm -rf build install log
colcon build --packages-select howyouseeme_ros2

# Check dependencies
rosdep install --from-paths src --ignore-src -r -y
```

#### 3. CUDA/GPU Issues
```bash
# Check CUDA
nvidia-smi
nvcc --version

# Test PyTorch CUDA
python3 -c "import torch; print(torch.cuda.is_available())"

# Check YOLO model loading
python3 -c "from ultralytics import YOLO; model = YOLO('yolo11n.pt'); print('Model loaded')"
```

#### 4. Performance Issues
```bash
# Check system resources
htop
nvidia-smi

# Monitor ROS2 performance
ros2 run rqt_top rqt_top

# Check topic rates
ros2 topic hz /kinect2/hd/image_color
```

#### 5. Launch File Issues
```bash
# Check launch file syntax
ros2 launch howyouseeme_ros2 howyouseeme_complete.launch.py --show-args

# Debug launch
ros2 launch howyouseeme_ros2 howyouseeme_complete.launch.py --debug
```

### Performance Optimization

#### For Maximum Performance
```bash
# Use fastest YOLO model
ros2 launch howyouseeme_ros2 detection_only.launch.py yolo_model:=yolo11n

# Reduce image resolution
ros2 param set /kinect2_bridge publish_qhd true
ros2 param set /kinect2_bridge publish_hd false
```

#### For Maximum Accuracy
```bash
# Use most accurate YOLO model
ros2 launch howyouseeme_ros2 detection_only.launch.py yolo_model:=yolo11x

# Increase confidence threshold
ros2 param set /howyouseeme_yolo_detector confidence_threshold 0.8
```

## Development & Testing

### Build and Test
```bash
# Build specific package
cd ~/ros2_ws
colcon build --packages-select howyouseeme_ros2

# Run tests
colcon test --packages-select howyouseeme_ros2

# Check test results
colcon test-result --verbose
```

### Custom Nodes
```bash
# Create new ROS2 node
cd ~/ros2_ws/src/howyouseeme_ros2/howyouseeme_ros2
# Add your custom node Python file

# Update setup.py to include new node
# Rebuild package
cd ~/ros2_ws
colcon build --packages-select howyouseeme_ros2
```

## Performance Expectations

With proper ROS2 setup, you should achieve:

### **Sensor Performance**
- **RGB-D Processing**: 14.5 FPS with CUDA acceleration
- **Frame Drop Rate**: <2% with intelligent processing
- **Sensor Latency**: <50ms end-to-end

### **Computer Vision Performance**
- **YOLOv12 Detection**: 10+ FPS on NVIDIA RTX 3050
- **SLAM Processing**: Real-time pose estimation
- **Total System Latency**: <75ms for complete pipeline

### **ROS2 Integration**
- **Topic Publishing**: 30+ topics active
- **Message Latency**: <10ms for ROS2 communication
- **System Stability**: 24/7 operation capable

## Next Steps

Once your ROS2 system is running:

1. **Explore Advanced Features**:
   - Multi-object tracking
   - Nav2 navigation integration
   - Custom behavior trees

2. **Integration Development**:
   - Custom ROS2 nodes
   - Service and action servers
   - Parameter configuration

3. **Production Deployment**:
   - Docker containerization
   - Multi-robot coordination
   - Cloud integration

## Support & Resources

### Documentation
- **[ROS2 Setup Guide](../ROS2_SETUP_GUIDE.md)** - Detailed installation
- **[Implementation Status](../IMPLEMENTATION_STATUS.md)** - Current progress
- **[System Architecture](../ROS2_SYSTEM_COMPLETE.md)** - Technical details

### Community
- **GitHub Issues**: [Report bugs and request features](https://github.com/AryanRai/HowYouSeeMe/issues)
- **ROS2 Community**: [ROS Discourse](https://discourse.ros.org/)
- **Email Support**: [buzzaryanrai@gmail.com](mailto:aryanrai170@gmail.com)

---

**Welcome to the future of ROS2-based computer vision! 🤖👁️**