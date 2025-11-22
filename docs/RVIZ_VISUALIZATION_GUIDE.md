# RViz Visualization Guide

## Overview

Complete visualization of Kinect + SLAM + CV Pipeline in RViz2.

## Launch Scripts

### 1. Full System with RViz (NEW!)

```bash
./launch_full_system_rviz.sh
```

**Includes**:
- ✅ Kinect v2 RGB-D Camera
- ✅ RTABMap SLAM (3D Mapping)
- ✅ CV Pipeline (SAM2 Segmentation)
- ✅ RViz2 (Complete Visualization)

### 2. SLAM Only with RViz

```bash
./launch_kinect2_ros2_slam_fixed_tf.sh
```

**Includes**:
- ✅ Kinect v2 RGB-D Camera
- ✅ RTABMap SLAM
- ✅ RViz2 (SLAM visualization)

### 3. CV Pipeline Only

```bash
./launch_kinect_cv_pipeline.sh
```

**Includes**:
- ✅ Kinect v2 RGB-D Camera
- ✅ CV Pipeline (SAM2)
- ❌ No RViz (use manually)

## RViz Displays

### Full System Configuration (`full_system_rviz.rviz`)

| Display | Topic | Description |
|---------|-------|-------------|
| **Grid** | - | Reference grid in 3D space |
| **TF** | /tf | Coordinate frames (kinect2_link, map, odom) |
| **Point Cloud** | /kinect2/qhd/points | Live 3D point cloud from Kinect |
| **SLAM Map** | /rtabmap/mapData | 3D map built by SLAM |
| **SLAM Graph** | /rtabmap/mapGraph | SLAM pose graph with loop closures |
| **Camera RGB** | /kinect2/qhd/image_color | Live RGB camera feed |
| **CV Pipeline Output** | /cv_pipeline/visualization | SAM2 segmentation overlay |
| **Odometry Path** | /rtabmap/odom_path | Camera trajectory |

## Using the Full System

### Step 1: Launch

```bash
./launch_full_system_rviz.sh
```

Wait for all components to start (~10 seconds).

### Step 2: RViz Layout

RViz will open with 8 displays:

**Left Panel**: Display controls
- Toggle displays on/off
- Adjust visualization settings

**Main View**: 3D visualization
- Point cloud (live)
- SLAM map (accumulated)
- TF frames
- Odometry path

**Bottom Panels**: Image displays
- Camera RGB (left)
- CV Pipeline Output (right)

### Step 3: Send Segmentation Request

In a new terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

# Segment with SAM2
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:prompt_type=point'"
```

Watch the **CV Pipeline Output** panel for results!

### Step 4: Build SLAM Map

Move the Kinect around slowly:
- Watch the **Point Cloud** update in real-time
- See the **SLAM Map** accumulate
- Observe the **Odometry Path** trace your movement

## RViz Controls

### Navigation

| Action | Control |
|--------|---------|
| Rotate view | Left mouse drag |
| Pan view | Middle mouse drag |
| Zoom | Scroll wheel |
| Reset view | Press 'R' |

### Tools (Top toolbar)

| Tool | Description |
|------|-------------|
| Interact | Default interaction mode |
| Move Camera | Adjust camera position |
| Select | Select objects |
| Focus Camera | Focus on selected object |
| Measure | Measure distances |
| 2D Pose Estimate | Set initial pose |
| 2D Nav Goal | Set navigation goal |
| Publish Point | Publish clicked point |

## Customizing Displays

### Adjust Point Cloud

1. Click **Point Cloud** in left panel
2. Expand properties
3. Adjust:
   - **Size (Pixels)**: Point size (default: 3)
   - **Style**: Flat Squares, Spheres, etc.
   - **Color Transformer**: RGB8, Intensity, etc.

### Adjust SLAM Map

1. Click **SLAM Map** in left panel
2. Adjust:
   - **Cloud decimation**: Lower = more detail (default: 4)
   - **Cloud voxel size**: Smaller = more detail (default: 0.01m)
   - **Cloud max depth**: Max distance (default: 4.0m)

### Show/Hide Displays

Click checkbox next to display name to toggle visibility.

## Viewing CV Pipeline Results

### In RViz

The **CV Pipeline Output** panel shows:
- Original RGB image
- Segmentation overlays (when processing)
- Model name and status

### In Terminal

```bash
# View JSON results
ros2 topic echo /cv_pipeline/results

# View processing status
ros2 topic echo /cv_pipeline/model_request
```

## Performance Tips

### For Smooth Visualization

1. **Reduce Point Cloud Density**:
   - Increase **Cloud decimation** to 8 or 16
   - Increase **Cloud voxel size** to 0.02m

2. **Limit Display Updates**:
   - Set **Frame Rate** to 15 FPS (Global Options)
   - Disable unused displays

3. **Optimize SLAM**:
   - Already optimized in launch script
   - Processing at 1 Hz detection rate

### For Better Quality

1. **Increase Point Cloud Detail**:
   - Decrease **Cloud decimation** to 2
   - Decrease **Cloud voxel size** to 0.005m

2. **Increase Frame Rate**:
   - Set **Frame Rate** to 30 FPS

## Troubleshooting

### RViz Shows Nothing

**Check Fixed Frame**:
1. Global Options → Fixed Frame
2. Should be: `map` (for SLAM) or `kinect2_link` (for camera)

**Check Topics**:
```bash
ros2 topic list | grep -E "kinect2|rtabmap|cv_pipeline"
```

### Point Cloud Not Showing

**Check Topic**:
1. Point Cloud → Topic → Value
2. Should be: `/kinect2/qhd/points`

**Check Kinect**:
```bash
ros2 topic hz /kinect2/qhd/points
```

### CV Pipeline Output Not Showing

**Send a Request First**:
```bash
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:prompt_type=point'"
```

**Check Topic**:
```bash
ros2 topic hz /cv_pipeline/visualization
```

### SLAM Map Not Building

**Move the Camera**:
- SLAM needs motion to work
- Move slowly and smoothly
- Avoid fast rotations

**Check Odometry**:
```bash
ros2 topic hz /rtabmap/odom
```

## Saving RViz Configuration

If you customize the layout:

```bash
# In RViz: File → Save Config As
# Save to: ~/Documents/GitHub/HowYouSeeMe/my_custom.rviz

# Use it:
rviz2 -d ~/Documents/GitHub/HowYouSeeMe/my_custom.rviz
```

## Example Workflow

### 1. Start System
```bash
./launch_full_system_rviz.sh
```

### 2. Wait for Initialization
- Kinect: 5 seconds
- SLAM: 3 seconds
- CV Pipeline: 2 seconds
- RViz: Opens automatically

### 3. Verify in RViz
- ✅ Point cloud visible
- ✅ Camera image showing
- ✅ TF frames displayed

### 4. Test CV Pipeline
```bash
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:prompt_type=point'"
```

### 5. Build SLAM Map
- Move Kinect slowly
- Watch map accumulate
- See loop closures (yellow lines)

### 6. Stop System
Press `Ctrl+C` in launch terminal

## Advanced: Multiple RViz Windows

### Window 1: SLAM View
```bash
rviz2 -d kinect2_slam_rviz.rviz &
```

### Window 2: CV Pipeline View
```bash
rviz2 &
# Manually add Image display
# Topic: /cv_pipeline/visualization
```

## Files

| File | Purpose |
|------|---------|
| `launch_full_system_rviz.sh` | Launch everything with RViz |
| `full_system_rviz.rviz` | Complete RViz configuration |
| `kinect2_slam_rviz.rviz` | SLAM-only configuration |
| `kinect2_rviz_config.rviz` | Basic Kinect visualization |

## Quick Reference

```bash
# Full system
./launch_full_system_rviz.sh

# Send SAM2 request
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:prompt_type=point'"

# View results
ros2 topic echo /cv_pipeline/results

# Check topics
ros2 topic list

# Check rates
ros2 topic hz /kinect2/qhd/image_color
ros2 topic hz /rtabmap/odom
ros2 topic hz /cv_pipeline/visualization
```

---

**Enjoy your complete vision system visualization!** 🎉
