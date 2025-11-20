# RTAB-Map SLAM Quick Reference

## 🚀 Launch Commands

```bash
# Start SLAM system
./launch_kinect_slam.sh

# Kill all processes
./kill_kinect.sh

# Test Kinect only
./test_kinect_simple.sh
```

## 📊 Understanding the Output

### Good Odometry ✅
```
Odom: quality=95, std dev=7.96m|0.032rad, update time=0.015s
```
- Quality: **70-120** ✅
- Std dev: **< 20m** ✅
- Update: **< 0.02s** ✅

### Poor Odometry ❌
```
Odom: quality=25, std dev=93.39m|0.084rad, update time=0.005s
```
- Quality: **< 50** ❌
- Std dev: **> 40m** ❌
- Needs improvement!

### RTAB-Map Status
```
rtabmap (202): Rate=1.00s, RTAB-Map=0.008s, (local map=6, WM=202)
```
- **local map=6**: Active nodes being visualized (should be < 50)
- **WM=202**: Total map nodes (grows continuously)
- **RTAB-Map=0.008s**: Processing time (should be < 0.1s)

## 🎯 Best Practices

### ✅ DO:
- Point at **textured surfaces** (furniture, walls with pictures)
- Stay **1-2 meters** from objects
- Move **slowly and smoothly** (< 0.5 m/s)
- Keep **some features in view** between frames
- **Revisit areas** for loop closures
- Use **USB 3.0** (blue port)

### ❌ DON'T:
- Point at **blank walls** or **windows**
- Get too **close (< 0.5m)** or **far (> 4m)**
- Move **too fast** or **jerk** the camera
- Use in **direct sunlight** (IR interference)
- Use **USB hubs** (bandwidth issues)

## 🔧 Common Issues

| Issue | Quick Fix |
|-------|-----------|
| Low quality (< 50) | Point at textured surfaces, move slower |
| Node IDs jumping | Normal loop closure, or increase `--Rtabmap/LoopThr` |
| RViz slowing down | Already fixed with local map visualization |
| NaN warnings | Avoid reflective surfaces, check USB connection |
| Map drift | Revisit previous locations for loop closures |

## 📁 Documentation

- **Performance**: `docs/SLAM_Performance_Optimization.md`
- **Troubleshooting**: `docs/SLAM_Troubleshooting.md`
- **Setup**: `docs/Kinect_SLAM_Integration.md`

## 🎮 RViz Controls

- **Orbit**: Left mouse drag
- **Pan**: Middle mouse drag or Shift + left drag
- **Zoom**: Scroll wheel
- **Enable full map**: Check "RTABMap Full Cloud (Disabled)" in Displays panel
  - ⚠️ Warning: Will slow down visualization!

## 🔍 Monitoring Commands

```bash
# Check topics
ros2 topic list | grep kinect2

# Monitor odometry
ros2 topic echo /rtabmap/odom

# Check TF tree
ros2 run tf2_tools view_frames

# Monitor CPU/memory
htop
```

## 🎛️ Quick Parameter Adjustments

Edit `launch_kinect_slam.sh` and adjust:

**For difficult environments:**
```bash
--Icp/Iterations 50                    # More robust (slower)
--Icp/CorrespondenceRatio 0.1          # Accept fewer matches
```

**For better performance:**
```bash
--Icp/Iterations 10                    # Faster (less robust)
--decimation:=4                        # Downsample more
```

**For stricter loop closures:**
```bash
--Rtabmap/LoopThr 0.2                  # Higher = stricter
--Rtabmap/LoopRatio 0.95               # Higher = more confident
```

## 💾 Database Management

```bash
# Database location
~/.ros/rtabmap.db

# View database
rtabmap-databaseViewer ~/.ros/rtabmap.db

# Delete database (fresh start)
rm ~/.ros/rtabmap.db

# Keep database (remove --delete_db_on_start from launch script)
```

## 🆘 Emergency Fixes

**System frozen?**
```bash
./kill_kinect.sh
# Or force kill:
killall -9 rviz2 rtabmap rgbd_odometry kinect2_simple_publisher_node
```

**Kinect not responding?**
```bash
# Unplug and replug USB
# Check with:
lsusb | grep Xbox
```

**ROS2 issues?**
```bash
# Re-source
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```
