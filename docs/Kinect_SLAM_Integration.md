# Kinect v2 SLAM Integration

## Overview

RTABMap (Real-Time Appearance-Based Mapping) SLAM is now integrated with the Kinect v2 ROS2 system, providing real-time 3D mapping and localization.

## Quick Start

### Launch Kinect + SLAM + RViz
```bash
./launch_kinect_slam.sh
```

This will start:
1. TF publishers (map → odom → camera_link → kinect2_rgb_optical_frame)
2. Kinect v2 publisher
3. RTABMap SLAM node
4. RViz with SLAM visualization

### Test SLAM Setup
```bash
./test_slam.sh
```

## SLAM Topics

### Published by RTABMap

- `/rtabmap/mapData` - Complete SLAM map data
- `/rtabmap/odom` - Visual odometry (camera pose estimation)
- `/rtabmap/cloud_map` - 3D point cloud map
- `/rtabmap/odom_path` - Trajectory path
- `/rtabmap/graph_nodes` - SLAM graph nodes (for visualization)
- `/rtabmap/info` - SLAM statistics and performance metrics

### Subscribed by RTABMap

- `/kinect2/sd/image_color` - RGB images
- `/kinect2/sd/image_depth` - Depth images
- `/kinect2/sd/camera_info` - Camera calibration

## RViz Visualization

The SLAM RViz config (`kinect2_slam_rviz.rviz`) shows:

1. **Grid** - Reference grid in map frame
2. **TF** - Transform tree visualization
3. **RGB Image** - Live camera feed
4. **RTABMap Cloud** - 3D point cloud map (colored)
5. **RTABMap Graph** - SLAM graph nodes and connections
6. **Odometry Path** - Camera trajectory (green line)

## Monitoring SLAM

### Check SLAM Status
```bash
# View SLAM info
ros2 topic echo /rtabmap/info

# Check odometry rate
ros2 topic hz /rtabmap/odom

# View map statistics
ros2 topic echo /rtabmap/mapData --once
```

### Performance Metrics
```bash
# Monitor all SLAM topics
ros2 topic list | grep rtabmap

# Check topic rates
ros2 topic hz /rtabmap/cloud_map
ros2 topic hz /rtabmap/odom
```

## RTABMap Configuration

The launch script uses these parameters:

```bash
rtabmap_args:="--delete_db_on_start"  # Start fresh each time
rgb_topic:=/kinect2/sd/image_color
depth_topic:=/kinect2/sd/image_depth
camera_info_topic:=/kinect2/sd/camera_info
frame_id:=camera_link
approx_sync:=true                      # Sync RGB and depth approximately
wait_imu_to_init:=false               # Don't wait for IMU
qos:=2                                # QoS reliability
```

## Advanced Usage

### Save SLAM Map

To save the map for later use, remove `--delete_db_on_start`:

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
    rgb_topic:=/kinect2/sd/image_color \
    depth_topic:=/kinect2/sd/image_depth \
    camera_info_topic:=/kinect2/sd/camera_info \
    frame_id:=camera_link \
    approx_sync:=true
```

The map will be saved to `~/.ros/rtabmap.db`

### Load Existing Map

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:="--Rtabmap/StartNewMapOnLoopClosure true" \
    rgb_topic:=/kinect2/sd/image_color \
    depth_topic:=/kinect2/sd/image_depth \
    camera_info_topic:=/kinect2/sd/camera_info
```

### Localization Mode

To use an existing map for localization only (no mapping):

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
    localization:=true \
    rgb_topic:=/kinect2/sd/image_color \
    depth_topic:=/kinect2/sd/image_depth \
    camera_info_topic:=/kinect2/sd/camera_info
```

## Integration with HowYouSeeMe

### Subscribe to SLAM Data

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from rtabmap_msgs.msg import Info

class SLAMIntegrationNode(Node):
    def __init__(self):
        super().__init__('slam_integration')
        
        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry, '/rtabmap/odom',
            self.odom_callback, 10)
        
        # Subscribe to point cloud map
        self.cloud_sub = self.create_subscription(
            PointCloud2, '/rtabmap/cloud_map',
            self.cloud_callback, 10)
        
        # Subscribe to SLAM info
        self.info_sub = self.create_subscription(
            Info, '/rtabmap/info',
            self.info_callback, 10)
    
    def odom_callback(self, msg):
        # Get camera pose
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        # Use for navigation, object detection context, etc.
    
    def cloud_callback(self, msg):
        # Process 3D map
        # Use for obstacle detection, path planning, etc.
    
    def info_callback(self, msg):
        # SLAM statistics
        loop_closures = msg.loop_closure_id
        # Monitor SLAM health
```

### Combine with Object Detection

```python
class SLAMObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('slam_object_detection')
        
        # Subscribe to RGB and odometry
        self.rgb_sub = self.create_subscription(
            Image, '/kinect2/sd/image_color',
            self.rgb_callback, 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/rtabmap/odom',
            self.odom_callback, 10)
        
        self.current_pose = None
        self.detector = YOLODetector()
    
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
    
    def rgb_callback(self, msg):
        # Detect objects
        detections = self.detector.detect(msg)
        
        # Add 3D position using current pose
        for det in detections:
            det.world_position = self.transform_to_world(
                det.bbox, self.current_pose)
```

## Troubleshooting

### SLAM Not Starting

Check if RTABMap is installed:
```bash
ros2 pkg list | grep rtabmap
```

Install if missing:
```bash
sudo apt install ros-jazzy-rtabmap-ros
```

### No Map Building

1. **Check topics are publishing:**
   ```bash
   ros2 topic hz /kinect2/sd/image_color
   ros2 topic hz /kinect2/sd/image_depth
   ```

2. **Check TF tree:**
   ```bash
   ros2 run tf2_tools view_frames
   ```

3. **Monitor SLAM info:**
   ```bash
   ros2 topic echo /rtabmap/info
   ```

### Poor SLAM Performance

1. **Move camera slowly** - Fast movements can cause tracking loss
2. **Ensure good lighting** - Poor lighting affects visual odometry
3. **Add visual features** - Blank walls are hard to track
4. **Check frame rates** - Should be >10 FPS for good SLAM

### Loop Closure Not Working

Loop closure requires:
- Revisiting previously mapped areas
- Good visual features
- Sufficient overlap between views

Monitor loop closures:
```bash
ros2 topic echo /rtabmap/info | grep loop_closure_id
```

## Performance Optimization

### Reduce Resolution

For better performance, use lower resolution:
```bash
# Modify Kinect publisher to use QHD instead of HD
# Or adjust RTABMap decimation parameters
```

### Adjust SLAM Parameters

Create custom RTABMap config:
```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:="--Rtabmap/DetectionRate 1 --Vis/MaxFeatures 400"
```

Common parameters:
- `Rtabmap/DetectionRate` - Process every Nth frame (default: 1)
- `Vis/MaxFeatures` - Max visual features (default: 1000)
- `RGBD/OptimizeMaxError` - Optimization threshold

## Files

- `launch_kinect_slam.sh` - Main SLAM launcher
- `kinect2_slam_rviz.rviz` - RViz configuration for SLAM
- `test_slam.sh` - SLAM setup verification

## Next Steps

1. **Test SLAM**: `./test_slam.sh`
2. **Launch system**: `./launch_kinect_slam.sh`
3. **Move camera around** to build map
4. **Monitor in RViz** - Watch point cloud and trajectory
5. **Integrate with your pipeline** - Use odometry and map data

## Resources

- [RTABMap ROS Wiki](http://wiki.ros.org/rtabmap_ros)
- [RTABMap Documentation](https://github.com/introlab/rtabmap)
- [RTABMap Parameters](https://github.com/introlab/rtabmap/wiki/Parameters)
