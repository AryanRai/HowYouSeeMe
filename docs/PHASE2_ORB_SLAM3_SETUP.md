# Phase 2: ORB-SLAM3 RGB-D+IMU Setup

Replace RTABMap with ORB-SLAM3 for tightly-coupled IMU preintegration. Eliminates ghosting and provides robust visual-inertial odometry.

## Prerequisites

- Phase 1 Kalibr calibration completed (`kalibr_output/camchain-imucam.yaml` exists)
- Kinect v2 publishing RGB-D at ~14.5Hz
- BlueLily IMU publishing at 800Hz on `/imu/data`

## Quick Start

```bash
# 1. Convert Kalibr calibration to ORB-SLAM3 format
python3 scripts/kalibr_to_orb_slam3.py

# 2. Build ORB-SLAM3 and ROS2 wrapper (15-20 minutes)
./scripts/setup_orb_slam3.sh

# 3. Verify installation
./scripts/verify_orb_slam3.sh

# 4. Launch system
ros2 launch kinect2_slam orb_slam3.launch.py
```

## Manual Installation

### Step 1: Install Dependencies

```bash
sudo apt install -y libglew-dev libboost-all-dev libssl-dev
sudo apt install -y libeigen3-dev libopencv-dev
```

### Step 2: Build Pangolin

```bash
git clone https://github.com/stevenlovegrove/Pangolin ~/Pangolin
cd ~/Pangolin && mkdir build && cd build
cmake .. && make -j4
sudo make install
```

If ORB-SLAM3 build fails with "Pangolin not found":
```bash
export Pangolin_DIR=/usr/local/lib/cmake/Pangolin
```

### Step 3: Build ORB-SLAM3

```bash
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3 ~/ORB_SLAM3
cd ~/ORB_SLAM3
chmod +x build.sh && ./build.sh
```

Build takes 15-20 minutes. Verify:
```bash
ls ~/ORB_SLAM3/lib/libORB_SLAM3.so
```

### Step 4: Build ROS2 Wrapper

```bash
cd ros2_ws/src
git clone https://github.com/zang09/ORB-SLAM3-ROS2 orb_slam3_ros2

# Update CMakeLists.txt with ORB_SLAM3 path
sed -i 's|set(ORB_SLAM3_ROOT_DIR.*|set(ORB_SLAM3_ROOT_DIR "$HOME/ORB_SLAM3")|' \
    orb_slam3_ros2/CMakeLists.txt

cd ~/Documents/GitHub/HowYouSeeMe/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select orb_slam3_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Step 5: Configure Calibration

The config file `orb_slam3_configs/Kinect2_RGBD_IMU.yaml` needs your Phase 1 calibration data.

**Automatic (recommended):**
```bash
python3 scripts/kalibr_to_orb_slam3.py
```

**Manual:**
1. Open `kalibr_output/camchain-imucam.yaml`
2. Copy cam0 intrinsics to `Camera.fx`, `Camera.fy`, `Camera.cx`, `Camera.cy`
3. Copy cam0 `T_cam_imu` matrix to `IMU.T_b_c1` (flatten 4x4 matrix row-wise)

## Running ORB-SLAM3

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

### Terminal 3: ORB-SLAM3
```bash
source ros2_ws/install/setup.bash
ros2 launch kinect2_slam orb_slam3.launch.py
```

## Verification

### Check Pose Output
```bash
# Verify publishing rate (~14.5Hz)
ros2 topic hz /orb_slam3/pose

# Check pose values
ros2 topic echo /orb_slam3/pose --once
```

Position should update as you move the robot. No ghosting or duplicate positions.

### RViz Visualization
```bash
ros2 run rviz2 rviz2
```

Add display:
- Type: PoseStamped
- Topic: `/orb_slam3/pose`
- Fixed Frame: `map`

Walk around slowly. Pose should track smoothly without jumps or offsets.

## Troubleshooting

### Tracking Lost Immediately
- **Cause:** Wrong intrinsics or T_cam_imu
- **Fix:** Verify `kalibr_to_orb_slam3.py` ran successfully
- **Check:** `Camera.depthFactor: 1000.0` (Kinect depth is in mm)

### Build Fails: Pangolin Not Found
```bash
export Pangolin_DIR=/usr/local/lib/cmake/Pangolin
cd ~/ORB_SLAM3 && ./build.sh
```

### Pose Not Publishing
- **Check topic remapping:** `ros2 topic list | grep -E "kinect2|imu"`
- **Verify:** RGB at `/kinect2/hd/image_color`, depth at `/kinect2/hd/image_depth_rect`, IMU at `/imu/data`

### IMU Not Fusing
- **Check:** `IMU.Frequency: 800` in config matches BlueLily output rate
- **Verify:** `ros2 topic hz /imu/data` shows ~800Hz

### Poor Tracking Quality
- Increase features: `ORBextractor.nFeatures: 1500`
- Adjust depth threshold: `ThDepth: 50.0`
- Check lighting conditions (ORB features need texture)

## Configuration Tuning

### IMU Noise Parameters
Default values work for BlueLily MPU9250. If tracking is unstable:

```yaml
# Increase if IMU is noisy
IMU.NoiseGyro: 0.002
IMU.NoiseAcc: 0.02

# Increase if drift is high
IMU.GyroWalk: 0.0002
IMU.AccWalk: 0.002
```

### ORB Feature Extraction
```yaml
# More features = better tracking, slower processing
ORBextractor.nFeatures: 1000  # Default
ORBextractor.nFeatures: 1500  # Better tracking
ORBextractor.nFeatures: 2000  # Best quality, may drop frames
```

## Next Steps

Once ORB-SLAM3 is tracking reliably:
- Phase 3: TSDF volumetric mapping
- Phase 4: Semantic projection with SAM2
- Phase 5: Face recognition integration

## Files Created

- `scripts/setup_orb_slam3.sh` - Automated build script
- `scripts/kalibr_to_orb_slam3.py` - Calibration converter
- `scripts/verify_orb_slam3.sh` - Installation verification
- `orb_slam3_configs/Kinect2_RGBD_IMU.yaml` - ORB-SLAM3 config
- `ros2_ws/src/kinect2_slam/launch/orb_slam3.launch.py` - Launch file


## Quick Start

Check dependencies:
```bash
./scripts/check_phase2_3_deps.sh
```

Launch complete system (Kinect + IMU + ORB-SLAM3 + TSDF):
```bash
./scripts/run_phase2_3.sh
```

This single script launches all 4 components in one terminal.

## TF2 Transforms

The ORB-SLAM3 node broadcasts the camera pose as a TF2 transform:
- Frame: `map` → `camera_pose` (or `camera_link`)
- Published on every successful tracking frame
- Used by semantic projection to transform detections to world coordinates

Check TF2 frames:
```bash
ros2 run tf2_ros tf2_monitor --all-frames
ros2 run tf2_ros tf2_echo map camera_pose
```

## Common Runtime Issues

### Kinect not detected
```bash
lsusb | grep -i kinect
# Should show: Microsoft Corp. Xbox NUI Sensor
```

### IMU not connected
```bash
ls /dev/ttyACM*
# Should show: /dev/ttyACM0 or similar
```

### ORB-SLAM3 tracking lost immediately
- Check `orb_slam3_configs/Kinect2_RGBD_IMU.yaml` intrinsics match your camera
- Verify `Camera.depthFactor: 1000.0` (Kinect depth is in mm)
- Ensure good lighting and textured surfaces

### TSDF node crashes with GLIBCXX error
If you see `libstdc++.so.6: version GLIBCXX_3.4.30 not found`:
```bash
conda deactivate
./scripts/run_phase2_3.sh
```
The conda environment's libstdc++ is older than what ROS2 Jazzy needs.

### High CPU usage
Reduce TSDF voxel resolution:
```bash
ros2 launch kinect2_slam orb_slam3.launch.py voxel_length:=0.08
```
