# Phase 3: Open3D TSDF Integration

CPU-based volumetric mapping that integrates ORB-SLAM3 poses with Kinect RGB-D into a dense colored point cloud. Zero VRAM usage leaves GPU free for YOLO and SAM2.

## Prerequisites

- Phase 2 ORB-SLAM3 publishing `/orb_slam3/pose` at ~14.5Hz
- Kinect v2 publishing RGB-D
- BlueLily IMU running

## Quick Start

```bash
# 1. Install Open3D
pip install open3d --break-system-packages

# 2. Verify installation
python3 -c "import open3d as o3d; print(o3d.__version__)"  # Needs 0.18+

# 3. Build package
cd ros2_ws
colcon build --packages-select kinect2_slam

# 4. Launch full system
source install/setup.bash
ros2 launch kinect2_slam orb_slam3.launch.py
```

## Architecture

```
Kinect RGB-D → ┐
               ├→ ORB-SLAM3 → /orb_slam3/pose → ┐
BlueLily IMU → ┘                                 ├→ TSDF Integrator → /tsdf/pointcloud → RViz
Kinect RGB-D ────────────────────────────────────┘
```

## Configuration

### Voxel Size Tuning

Edit launch file or pass as argument:

```bash
# High detail (small rooms, high CPU)
ros2 launch kinect2_slam orb_slam3.launch.py voxel_length:=0.02

# Balanced (recommended)
ros2 launch kinect2_slam orb_slam3.launch.py voxel_length:=0.04

# Coarse (large spaces, low CPU)
ros2 launch kinect2_slam orb_slam3.launch.py voxel_length:=0.08
```

**Voxel size guidelines:**
- `0.02` - Very high detail, high CPU load, small rooms only
- `0.04` - Good detail, moderate CPU, recommended default
- `0.08` - Coarse, low CPU, house-scale fast scanning
- `0.10` - Block-like, very low CPU, large outdoor spaces

### Camera Intrinsics

Update in launch file with Phase 1 calibration values:

```python
'fx': 1081.37,  # From kalibr_output/camchain-imucam.yaml
'fy': 1081.37,
'cx': 959.5,
'cy': 539.5,
```

## Running

### Terminal 1: Kinect Publisher
```bash
source ros2_ws/install/setup.bash
ros2 launch kinect2_slam kinect2_publisher.launch.py
```

### Terminal 2: BlueLily IMU
```bash
source ros2_ws/install/setup.bash
ros2 run bluelily_bridge bluelily_imu_node
```

### Terminal 3: ORB-SLAM3 + TSDF
```bash
source ros2_ws/install/setup.bash
ros2 launch kinect2_slam orb_slam3.launch.py
```

## Visualization

### RViz Setup
```bash
ros2 run rviz2 rviz2
```

Add displays:
1. **PointCloud2**
   - Topic: `/tsdf/pointcloud`
   - Fixed Frame: `map`
   - Style: Points
   - Size: 0.01

2. **PoseStamped**
   - Topic: `/orb_slam3/pose`
   - Shows current camera position

### Expected Behavior

- Dense colored point cloud builds up as you move
- Cloud updates at ~1Hz (configurable with `publish_rate`)
- No ghosting or duplicate geometry
- Smooth integration of new observations

## Mesh Export

Export TSDF mesh for offline processing (OpenSplat, etc.):

```bash
ros2 service call /tsdf/export_mesh std_srvs/srv/Trigger
```

Mesh saved to `/tmp/tsdf_mesh.ply` by default. Change with:

```python
'export_path': '/path/to/output.ply'
```

## Performance Monitoring

```bash
# Monitor CPU usage
htop

# Check integration rate
ros2 topic hz /tsdf/pointcloud

# Check point cloud size
ros2 topic echo /tsdf/pointcloud --once | grep "width:"
```

## Resource Usage (4GB Laptop)

| Component | CPU Cores | RAM | VRAM |
|-----------|-----------|-----|------|
| ORB-SLAM3 | 2-3 | ~500MB | ~200MB |
| TSDF Integrator | 1-2 | ~1.5GB | 0MB |
| YOLO11 | 0.5 | ~500MB | ~800MB |
| SAM2 tiny | 0.5 | ~500MB | ~280MB |
| **Total** | **~5** | **~3GB** | **~1.3GB** |

TSDF runs entirely on CPU, leaving GPU free for vision models.

## Troubleshooting

### Point Cloud Not Appearing

**Check topics:**
```bash
ros2 topic list | grep -E "orb_slam3|tsdf"
ros2 topic hz /orb_slam3/pose
ros2 topic hz /tsdf/pointcloud
```

**Check pose validity:**
```bash
ros2 topic echo /orb_slam3/pose --once
```
Position should not be all zeros.

### High CPU Usage

- Increase `voxel_length` (0.04 → 0.08)
- Decrease `publish_rate` (1.0 → 0.5)
- Reduce `depth_trunc` in code (3.0 → 2.0 meters)

### Memory Issues

- Increase `voxel_length` to reduce volume resolution
- Restart TSDF node periodically for long sessions
- Export mesh and restart with fresh volume

### Poor Integration Quality

- Verify ORB-SLAM3 tracking is stable
- Check camera intrinsics match Phase 1 calibration
- Ensure `depth_scale=1000.0` (Kinect depth is in mm)
- Verify transformation matrix inversion in code

## Parameters Reference

| Parameter | Default | Description |
|-----------|---------|-------------|
| `voxel_length` | 0.04 | TSDF voxel size in meters |
| `sdf_trunc` | 0.08 | SDF truncation distance |
| `publish_rate` | 1.0 | Point cloud publish rate (Hz) |
| `export_path` | `/tmp/tsdf_mesh.ply` | Mesh export location |
| `fx`, `fy` | 1081.37 | Camera focal lengths |
| `cx`, `cy` | 959.5, 539.5 | Camera principal point |

## Next Steps

- Phase 4: Semantic projection with SAM2
- Phase 5: Face recognition integration
- OpenSplat processing of exported meshes

## Files Created

- `ros2_ws/src/kinect2_slam/kinect2_slam/tsdf_integrator_node.py` - TSDF node
- `ros2_ws/src/kinect2_slam/launch/orb_slam3.launch.py` - Updated launch file
- `ros2_ws/src/kinect2_slam/setup.py` - Python package setup
- `ros2_ws/src/kinect2_slam/CMakeLists.txt` - Updated build config


## Quick Start

Phase 2 and 3 are now integrated into a single launch system.

Check all dependencies:
```bash
./scripts/check_phase2_3_deps.sh
```

Launch complete system:
```bash
./scripts/run_phase2_3.sh
```

This launches:
1. Kinect v2 bridge
2. BlueLily IMU node
3. ORB-SLAM3 RGB-D+IMU tracking
4. TSDF volumetric integrator

Export mesh during runtime:
```bash
ros2 service call /tsdf/export_mesh std_srvs/srv/Trigger
```

Visualize in RViz:
```bash
rviz2 -d kinect2_slam_rviz.rviz
```

Add displays:
- PoseStamped on `/orb_slam3/pose` - Camera trajectory
- PointCloud2 on `/tsdf/pointcloud` - Dense 3D reconstruction
