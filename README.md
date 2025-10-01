# HowYouSeeMe - Production ROS2 Computer Vision System

> **A production-ready ROS2-based computer vision system for real-time perception using Microsoft Kinect v2, featuring RTABMap SLAM, YOLOv12 object detection, Nav2 navigation, and comprehensive robotics ecosystem integration.**

[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)
[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![Kinect](https://img.shields.io/badge/Kinect-v2-orange)](ROS2_SETUP_GUIDE.md)
[![CUDA](https://img.shields.io/badge/CUDA-12.0+-green)](https://developer.nvidia.com/cuda-toolkit)
[![Performance](https://img.shields.io/badge/Performance-14.5_FPS-red)](ROS2_SYSTEM_COMPLETE.md)
[![Topics](https://img.shields.io/badge/ROS2_Topics-30+-purple)](IMPLEMENTATION_STATUS.md)

## Overview

HowYouSeeMe is a **production-ready ROS2-based computer vision system** that provides real-time spatial awareness, advanced SLAM, and object recognition. Built entirely around **ROS2 Jazzy**, it leverages the complete robotics ecosystem including Nav2 navigation, RTABMap SLAM, and TF2 transforms while maintaining exceptional performance through CUDA acceleration and intelligent processing.

### 🚀 **One-Command Launch**
```bash
# Complete system with SLAM and navigation
ros2 launch howyouseeme_ros2 howyouseeme_complete.launch.py use_rtabmap:=true

# Detection and visualization only
ros2 launch howyouseeme_ros2 detection_only.launch.py
```

### 🏗️ ROS2-Centric Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│ Kinect v2       │───▶│ kinect2_bridge   │───▶│ ROS2 Topics     │
│ (CUDA Accel)    │    │ (14.5 FPS)       │    │ /kinect2/hd/*   │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                                │                       │
                                ▼                       ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│ RTABMap SLAM    │◀───│ HowYouSeeMe      │◀───│ YOLOv12         │
│ (3D Mapping)    │    │ ROS2 Pipeline    │    │ Detection       │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│ Nav2 Navigation │    │ TF2 Transforms   │    │ RViz2           │
│ (Path Planning) │    │ (Pose Tracking)  │    │ Visualization   │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

### Core ROS2 Components

1. **📡 High-Performance Sensor Bridge**
   - **kinect2_bridge** with CUDA acceleration (14.5 FPS)
   - **30+ ROS2 topics** published (HD, QHD, SD streams)
   - **Real-time RGB-D** processing with intelligent frame management
   - **TF2 transforms** for spatial coordinate management

2. **🧠 Advanced Computer Vision Pipeline**
   - **RTABMap SLAM** for production-grade 3D mapping and localization
   - **YOLOv12 (YOLO11)** object detection with GPU acceleration
   - **Multi-object tracking** with Kalman filtering
   - **ROS2 vision_msgs** for standardized detection publishing

3. **🤖 Complete Robotics Integration**
   - **Nav2 navigation stack** for autonomous path planning
   - **RViz2 visualization** with custom configurations
   - **Behavior trees** integration for complex autonomous behaviors
   - **Standard ROS2 ecosystem** compatibility (sensor_msgs, geometry_msgs)

## 📡 ROS2 Topics & Data Flow

### **Published Topics (30+)**
```bash
# Sensor Data
/kinect2/hd/image_color          # RGB images (1920x1080) @ 14.5 FPS
/kinect2/hd/image_depth_rect     # Registered depth @ 14.5 FPS
/kinect2/hd/camera_info          # Camera calibration
/kinect2/qhd/*                   # Quarter HD streams (960x540)
/kinect2/sd/*                    # Standard definition (512x424)

# Computer Vision Results
/howyouseeme/detections          # YOLOv12 object detections
/howyouseeme/pose                # SLAM pose estimates
/howyouseeme/map                 # 3D point cloud map
/howyouseeme/tracking            # Multi-object tracking

# Navigation & Mapping
/map                             # Occupancy grid map
/odom                            # Odometry data
/tf                              # Transform tree
/cmd_vel                         # Velocity commands
```

### **ROS2 System Architecture**
```
┌─────────────────────────────────────────────────────────────┐
│                 HowYouSeeMe ROS2 System                    │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │ kinect2     │  │ RTABMap     │  │ Nav2        │        │
│  │ bridge      │─▶│ SLAM        │─▶│ Navigation  │        │
│  │ (Sensor)    │  │ (Mapping)   │  │ (Planning)  │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
│         │                 │                 │              │
│         ▼                 ▼                 ▼              │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │ YOLOv12     │  │ TF2         │  │ RViz2       │        │
│  │ Detection   │  │ Transforms  │  │ Visualization│       │
│  │ (Vision)    │  │ (Coords)    │  │ (Monitor)   │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## 🤖 Robotics Ecosystem Integration

HowYouSeeMe is designed as a **core perception module** for autonomous robotics:

- **🗺️ SLAM & Mapping**: RTABMap provides production-grade 3D mapping and localization
- **🎯 Navigation**: Nav2 stack enables autonomous path planning and obstacle avoidance
- **👁️ Computer Vision**: YOLOv12 delivers state-of-the-art object detection and tracking
- **🔗 ROS2 Ecosystem**: Full compatibility with standard robotics tools and frameworks

## 🚀 Quick Start

### Prerequisites
- **OS**: Ubuntu 24.04 LTS
- **ROS2**: Jazzy Jalopy
- **Hardware**: Microsoft Kinect v2 with USB 3.0
- **GPU**: NVIDIA with CUDA 12.0+ (recommended)
- **RAM**: 8GB+ recommended

### One-Command Setup
```bash
# Complete ROS2 system setup
./setup_kinect2_ros2.sh
```

### Launch System

#### **Option 1: Complete System (Recommended)**
```bash
# Launch everything: sensor bridge, SLAM, detection, visualization
ros2 launch howyouseeme_ros2 howyouseeme_complete.launch.py
```

#### **Option 2: With Advanced SLAM**
```bash
# Launch with RTABMap SLAM for production mapping
ros2 launch howyouseeme_ros2 howyouseeme_complete.launch.py use_rtabmap:=true
```

#### **Option 3: Detection Only**
```bash
# Just object detection and visualization
ros2 launch howyouseeme_ros2 detection_only.launch.py
```

#### **Option 4: Custom Configuration**
```bash
# Custom YOLO model and confidence
ros2 launch howyouseeme_ros2 detection_only.launch.py \
    yolo_model:=yolo11m \
    confidence_threshold:=0.7
```

### Monitor System Performance

```bash
# Check all published topics
ros2 topic list | grep -E "(kinect2|howyouseeme)"

# Monitor sensor performance
ros2 topic hz /kinect2/hd/image_color

# View detection results
ros2 topic echo /howyouseeme/detections

# Monitor system with RQT
rqt_graph  # Topic graph visualization
rqt_plot   # Real-time data plotting
```

### Manual Launch (for debugging)

```bash
# Terminal 1: Kinect sensor bridge
ros2 run kinect2_bridge kinect2_bridge_node

# Terminal 2: HowYouSeeMe processing pipeline
ros2 run howyouseeme_ros2 kinect_subscriber

# Terminal 3: YOLOv12 detection node
ros2 run howyouseeme_ros2 yolo_detector_node

# Terminal 4: Visualization
rviz2 -d ~/ros2_ws/src/howyouseeme_ros2/config/howyouseeme_complete.rviz
```

## 📚 Documentation

### **Essential ROS2 Guides**
- **[🚀 ROS2 Setup Guide](ROS2_SETUP_GUIDE.md)** - Complete installation and configuration
- **[📊 System Status](IMPLEMENTATION_STATUS.md)** - Current implementation progress
- **[⚡ Performance Analysis](ROS2_SYSTEM_COMPLETE.md)** - Detailed performance metrics
- **[🎮 Getting Started](docs/Getting_Started.md)** - Quick start tutorial

### **Technical Documentation**
- **[🔧 Kinect Integration](docs/KinectV2RosHumble.md)** - Kinect v2 ROS2 bridge setup
- **[🧠 Computer Vision](src/perception/detection/object_detector.py)** - YOLOv12 implementation
- **[🗺️ SLAM Integration](src/perception/slam/slam_interface.py)** - RTABMap SLAM bridge
- **[📡 ROS2 Nodes](~/ros2_ws/src/howyouseeme_ros2/)** - Complete ROS2 package structure

## ✨ Key Features

### 🔍 **Production-Ready Computer Vision**
- **RTABMap SLAM**: Production-grade 3D mapping and localization with loop closure
- **YOLOv12 Detection**: Latest YOLO11 architecture with 80+ object classes
- **Multi-Object Tracking**: Kalman filter-based tracking with ID persistence
- **Real-Time Processing**: 14.5 FPS RGB-D with intelligent frame management
- **GPU Acceleration**: CUDA-optimized processing pipeline

### 🤖 **Complete ROS2 Integration**
- **30+ ROS2 Topics**: Comprehensive sensor data and processing results
- **Nav2 Navigation**: Autonomous path planning and obstacle avoidance
- **TF2 Transforms**: Complete spatial coordinate management
- **Standard Messages**: sensor_msgs, geometry_msgs, vision_msgs compatibility
- **Launch System**: Flexible deployment configurations

### 🚀 **High-Performance Architecture**
- **CUDA Acceleration**: 6x performance improvement with GPU processing
- **Intelligent Processing**: Adaptive frame dropping with 1.1% drop rate
- **Multi-Resolution**: HD, QHD, and SD streams for different use cases
- **Resource Management**: Optimized memory and CPU utilization
- **Real-Time Monitoring**: Comprehensive performance metrics and visualization

## 🏗️ Development Status

### ✅ Completed
- [x] **Foundation**: Kinect v2 integration and basic RGB-D processing
- [x] **Documentation**: Comprehensive project planning and architecture
- [x] **Ecosystem Design**: Integration strategy with DroidCore platform

### 🚧 In Development
- [ ] **Computer Vision Pipeline**: SLAM + YOLO + Segmentation integration
- [ ] **Memory System**: Redis-based RAG implementation
- [ ] **MCP Server**: Model Context Protocol interface
- [ ] **Ally Integration**: Tool calling and cognitive processing

### 🔮 Planned
- [ ] **Advanced Features**: Multi-camera support and sensor fusion
- [ ] **Performance Optimization**: Real-time 30fps processing
- [ ] **Deployment Tools**: Containerization and scaling solutions
- [ ] **Mobile Support**: Remote monitoring and control interfaces

## 🛠️ Development & Testing

### Build ROS2 Workspace
```bash
# Build the complete ROS2 package
cd ~/ros2_ws
colcon build --packages-select howyouseeme_ros2

# Source the workspace
source install/setup.bash

# Verify installation
ros2 pkg list | grep howyouseeme
```

### Testing & Validation

#### **System Testing**
```bash
# Test Kinect bridge
ros2 run kinect2_bridge kinect2_bridge_node

# Test YOLOv12 detection
ros2 run howyouseeme_ros2 yolo_detector_node

# Test complete pipeline
ros2 launch howyouseeme_ros2 howyouseeme_complete.launch.py
```

#### **Performance Monitoring**
```bash
# Launch performance monitoring
ros2 launch howyouseeme_ros2 performance_monitor.launch.py

# Check topic frequencies
ros2 topic hz /kinect2/hd/image_color
ros2 topic hz /howyouseeme/detections

# Monitor system resources
htop  # CPU and memory usage
nvidia-smi  # GPU utilization
```

#### **Visualization & Debugging**
```bash
# Launch RViz2 with custom configuration
rviz2 -d ~/ros2_ws/src/howyouseeme_ros2/config/howyouseeme_complete.rviz

# View topic graph
rqt_graph

# Plot real-time data
rqt_plot /howyouseeme/detections/detections[0]/results[0]/hypothesis/score
```

### Implementation Status

#### ✅ **Phase 1: Core ROS2 System (COMPLETED)**
- [x] **ROS2 Jazzy Integration**: Complete workspace and package setup
- [x] **High-Performance Sensor Bridge**: kinect2_bridge with CUDA (14.5 FPS)
- [x] **YOLOv12 Object Detection**: Latest YOLO11 architecture with GPU acceleration
- [x] **RTABMap SLAM Integration**: Production-grade 3D mapping and localization
- [x] **Launch System**: Comprehensive launch files for different scenarios
- [x] **Visualization**: RViz2 configurations and real-time monitoring

#### 🚧 **Phase 2: Advanced Features (IN PROGRESS)**
- [ ] **Nav2 Navigation Stack**: Path planning and autonomous navigation (80%)
- [ ] **Multi-Object Tracking**: Kalman filter-based tracking with ID persistence (70%)
- [ ] **Enhanced Visualization**: Advanced RViz2 plugins and dashboards (60%)
- [ ] **Semantic Segmentation**: Pixel-level scene understanding (40%)

#### 📋 **Phase 3: Production Features (PLANNED)**
- [ ] **Multi-Sensor Fusion**: IMU, GPS, and audio integration
- [ ] **Autonomous Behaviors**: Behavior trees and state machines
- [ ] **Web Interface**: ROS2 web bridge for remote monitoring
- [ ] **Cloud Integration**: Edge computing and data synchronization
- [ ] **Mobile Robot Platform**: Complete autonomous robot integration

### ROS2 Project Structure

```
HowYouSeeMe/
├── 📁 ~/ros2_ws/src/howyouseeme_ros2/    # Main ROS2 package ✅
│   ├── launch/                           # ROS2 launch files ✅
│   │   ├── howyouseeme_complete.launch.py    # Complete system ✅
│   │   ├── detection_only.launch.py          # Detection only ✅
│   │   └── performance_monitor.launch.py     # Performance monitoring ✅
│   ├── config/                           # ROS2 configurations ✅
│   │   └── howyouseeme_complete.rviz         # RViz2 config ✅
│   ├── howyouseeme_ros2/                 # Python package ✅
│   │   ├── kinect_subscriber.py              # Main ROS2 node ✅
│   │   ├── yolo_detector_node.py             # YOLOv12 detection ✅
│   │   └── performance_monitor.py            # Performance monitoring ✅
│   ├── package.xml                       # ROS2 package manifest ✅
│   └── setup.py                          # Python package setup ✅
├── 📁 src/perception/                    # ROS2-centric perception ✅
│   ├── ros2_sensor_interface.py          # ROS2 sensor bridge ✅
│   ├── detection/                        # Object detection ✅
│   │   └── ros2_object_detector.py       # ROS2 YOLOv12 detector ✅
│   ├── slam/                             # SLAM integration ✅
│   │   └── ros2_slam_interface.py        # ROS2 SLAM bridge ✅
│   ├── face_analysis/                    # Face detection 🚧
│   ├── tracking/                         # Multi-object tracking 🚧
│   └── navigation/                       # Nav2 integration 📋
├── 📁 docs/                              # Documentation ✅
│   ├── ROS2_SETUP_GUIDE.md              # Complete ROS2 setup ✅
│   ├── Getting_Started.md               # Quick start guide ✅
│   └── KinectV2RosHumble.md             # Kinect ROS2 integration ✅
├── 📁 config/                            # System configuration ✅
│   └── config.yaml                       # Main config ✅
├── 📁 scripts/                           # Setup and utility scripts ✅
│   ├── setup_kinect2_ros2.sh            # One-command setup ✅
│   ├── start_kinect_bridge.sh           # Kinect bridge launcher ✅
│   └── start_howyouseeme.sh             # Pipeline launcher ✅
├── IMPLEMENTATION_STATUS.md              # Current progress ✅
├── ROS2_SYSTEM_COMPLETE.md              # System completion status ✅
└── README.md                            # This file ✅
```

**Legend**: ✅ Complete | 🚧 In Progress | 📋 Planned

### 🎯 **Use Cases**

#### **Research & Development**
- Computer vision algorithm development and testing
- SLAM and navigation research with real-time data
- Multi-sensor fusion experiments and validation
- ROS2 robotics education and coursework

#### **Production Robotics**
- Autonomous mobile robots with navigation
- Industrial inspection and quality control
- Service robots for hospitality and healthcare
- Warehouse automation and logistics

#### **Performance Achievements**
- **Real-time Processing**: 14.5 FPS RGB-D processing (target achieved)
- **Detection Performance**: YOLOv12 at 10+ FPS with GPU acceleration
- **System Latency**: <75ms end-to-end processing
- **Memory Efficiency**: ~4GB RAM for complete pipeline
- **ROS2 Integration**: 30+ topics with standard message compatibility

## 🤝 Contributing

We welcome contributions! Please see our development plan and:

1. **Fork** the repository
2. **Create** a feature branch for your component
3. **Follow** the architecture outlined in the implementation plan
4. **Test** your changes thoroughly
5. **Submit** a pull request with clear documentation

### Development Guidelines
- **Python**: Follow PEP 8 with type hints
- **Documentation**: Update relevant docs for any changes
- **Testing**: Include unit tests for new functionality
- **Integration**: Ensure compatibility with ecosystem components

## 📄 License

[MIT License](LICENSE) - see LICENSE file for details.

## 🆘 Support

- **📧 Email**: [buzzaryanrai@gmail.com](mailto:aryanrai170@gmail.com)
- **🐛 Issues**: [GitHub Issues](https://github.com/AryanRai/HowYouSeeMe/issues)
- **📖 Documentation**: [docs/](docs/) folder for comprehensive guides
- **💬 Discussions**: Join the DroidCore ecosystem discussions

---

**Built with ❤️ for the future of AI-powered robotics and world understanding.**
