# Kinect2 ROS2 Bridge Setup

## Overview

We've integrated the `kinect2_ros2` bridge, which is a proper ROS2 port of the original `iai_kinect2` package. This provides a more complete and robust implementation compared to our simple publisher.

## Repository

- **Source**: https://github.com/krepa098/kinect2_ros2
- **Based on**: Fork of iai_kinect2 for OpenCV 4 and ROS2 Humble
- **Compatible with**: ROS2 Jazzy (tested)

## Features

### What Works:
- ✅ `kinect2_bridge` - Full bridge with multiple resolutions
- ✅ `kinect2_registration` - Depth registration (CPU only)
- ✅ `kinect2_calibration` - Calibration tools
- ✅ Multiple resolution streams (SD, QHD, HD)
- ✅ Point cloud generation
- ✅ TF publishing
- ✅ Bilateral and edge-aware filtering

### What's Different from Original:
- ❌ No CUDA/OpenCL support (CPU only)
- ❌ No kinect2_viewer (use RViz2 instead)
- ✅ Native ROS2 implementation
- ✅ Modern CMake and ament build system

## Installation

### 1. Prerequisites

libfreenect2 must be installed (already done in our setup):
```bash
# Located at: /home/aryan/Documents/GitHub/HowYouSeeMe/libfreenect2/freenect2
```

### 2. Clone and Build

```bash
cd ~/Documents/GitHub/HowYouSeeMe/ros2_ws/src
git clone https://github.com/krepa098/kinect2_ros2.git

cd ~/Documents/GitHub/HowYouSeeMe/ros2_ws
source /opt/ros/jazzy/setup.bash

# Build with freenect2 path
colcon build --packages-select kinect2_registration kinect2_bridge \
  --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -Dfreenect2_DIR=/home/aryan/Documents/GitHub/HowYouSeeMe/libfreenect2/freenect2/lib/cmake/freenect2
```

### 3. Configuration

The freenect2Config.cmake was updated to point to our libfreenect2 installation:
```cmake
SET(freenect2_ROOT /home/aryan/Documents/GitHub/HowYouSeeMe/libfreenect2/freenect2)
```

## Usage

### Test the Bridge

```bash
./test_kinect2_ros2.sh
```

This launches the kinect2_bridge and publishes topics under `/kinect2/`.

### Available Topics

The bridge publishes multiple resolution streams:

**SD (512x424):**
- `/kinect2/sd/image_color`
- `/kinect2/sd/image_depth`
- `/kinect2/sd/camera_info`
- `/kinect2/sd/image_color_rect`
- `/kinect2/sd/image_depth_rect`

**QHD (960x540):**
- `/kinect2/qhd/image_color`
- `/kinect2/qhd/image_depth`
- `/kinect2/qhd/camera_info`
- `/kinect2/qhd/image_color_rect`
- `/kinect2/qhd/image_depth_rect`
- `/kinect2/qhd/points` (point cloud)

**HD (1920x1080):**
- `/kinect2/hd/image_color`
- `/kinect2/hd/image_depth`
- `/kinect2/hd/camera_info`
- `/kinect2/hd/image_color_rect`
- `/kinect2/hd/image_depth_rect`

**IR:**
- `/kinect2/ir/image`
- `/kinect2/ir/camera_info`

### SLAM Integration

Launch SLAM with the new bridge:

```bash
./launch_kinect2_ros2_slam.sh
```

This uses:
- **QHD resolution** (960x540) - good balance of quality and performance
- **Rectified images** (`image_color_rect`, `image_depth_rect`)
- **Proper camera calibration** from the bridge
- **ICP odometry** for robust tracking

## Parameters

### Bridge Parameters

Edit `ros2_ws/src/kinect2_ros2/kinect2_bridge/launch/kinect2_bridge_launch.yaml`:

```yaml
- name: "fps_limit"  
  value: 30.0              # Max FPS (Kinect v2 max is 30)

- name: "depth_method"  
  value: "default"         # Options: default, cpu, opencl, cuda

- name: "reg_method"  
  value: "default"         # Options: default, cpu, opencl

- name: "max_depth"  
  value: 12.0              # Maximum depth in meters

- name: "min_depth"  
  value: 0.1               # Minimum depth in meters

- name: "bilateral_filter"  
  value: true              # Smooth depth image

- name: "edge_aware_filter"  
  value: true              # Preserve edges

- name: "worker_threads"  
  value: 4                 # Processing threads
```

## Comparison: kinect2_ros2 vs kinect2_simple_publisher

| Feature | kinect2_simple_publisher | kinect2_ros2 |
|---------|-------------------------|--------------|
| **Implementation** | Custom, minimal | Full iai_kinect2 port |
| **Resolutions** | SD only | SD, QHD, HD |
| **Calibration** | Manual parameters | Proper calibration files |
| **Registration** | Basic | Advanced with filtering |
| **Point Clouds** | No | Yes (via depth_image_proc) |
| **TF Publishing** | Manual | Built-in |
| **Filtering** | No | Bilateral + edge-aware |
| **Performance** | Good | Better (optimized) |
| **Maintenance** | Custom | Community supported |

## Advantages of kinect2_ros2

1. **Proper Calibration**: Uses calibration files for accurate depth-color registration
2. **Multiple Resolutions**: Choose based on your needs (SD for speed, HD for quality)
3. **Built-in Filtering**: Bilateral and edge-aware filters improve depth quality
4. **Point Clouds**: Automatic point cloud generation
5. **Standard Topics**: Compatible with standard ROS2 tools and packages
6. **Better Registration**: Advanced depth registration algorithms
7. **Community Support**: Based on widely-used iai_kinect2 package

## Calibration

To calibrate your Kinect v2:

```bash
# Chess pattern calibration
ros2 run kinect2_calibration kinect2_calibration_node \
  chess5x7x0.03 record sync

# Circle pattern calibration  
ros2 run kinect2_calibration kinect2_calibration_node \
  circle7x6x0.02 record sync
```

Calibration files are stored in:
```
ros2_ws/src/kinect2_ros2/kinect2_bridge/data/<serial_number>/
```

## Troubleshooting

### Bridge not starting

Check libfreenect2 library path:
```bash
export LD_LIBRARY_PATH=/home/aryan/Documents/GitHub/HowYouSeeMe/libfreenect2/freenect2/lib:$LD_LIBRARY_PATH
```

### No topics published

The bridge only processes data when clients are connected (saves resources). Start a subscriber:
```bash
ros2 topic hz /kinect2/qhd/image_color
```

### Low FPS

- Reduce resolution (use SD instead of QHD/HD)
- Disable filters in launch file
- Reduce worker_threads if CPU limited

### Build errors

Make sure freenect2_DIR is set correctly:
```bash
-Dfreenect2_DIR=/home/aryan/Documents/GitHub/HowYouSeeMe/libfreenect2/freenect2/lib/cmake/freenect2
```

## Performance Tips

1. **Use QHD for SLAM**: Good balance of quality and performance
2. **Use SD for real-time**: Fastest processing
3. **Use HD for quality**: Best image quality but slower
4. **Enable filtering**: Improves depth quality at small performance cost
5. **Adjust worker_threads**: Match your CPU cores

## Next Steps

1. Test the bridge: `./test_kinect2_ros2.sh`
2. Test SLAM: `./launch_kinect2_ros2_slam.sh`
3. Compare odometry quality with previous setup
4. Calibrate your specific Kinect if needed
5. Adjust parameters based on your environment

## References

- [kinect2_ros2 GitHub](https://github.com/krepa098/kinect2_ros2)
- [Original iai_kinect2](https://github.com/code-iai/iai_kinect2)
- [libfreenect2](https://github.com/OpenKinect/libfreenect2)
