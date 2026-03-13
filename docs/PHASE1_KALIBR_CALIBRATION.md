# Phase 1 — Kalibr IMU-Camera Calibration

**Status**: Ready to execute  
**Prerequisites**: BlueLily bridge audit complete ✅  
**Critical Output**: `camchain-imucam.yaml` with T_cam_imu transform

---

## Why This Matters

ORB-SLAM3's tightly coupled IMU preintegration requires accurate camera-to-IMU extrinsics. A bad calibration causes:
- Silent drift (no error messages)
- Ghosting/duplicate point clouds
- Lost tracking during fast motion
- Incorrect semantic projection in Phase 4

**This is the single most important step.** Everything in Phases 2, 3, and 4 depends on it.

---

## Prerequisites Checklist

### Hardware
- ✅ Kinect v2 connected and working
- ✅ BlueLily IMU connected (/dev/ttyACM0)
- ✅ Both rigidly mounted in robot head (no flex/movement between them)
- 📋 Printed checkerboard target (A3 size, mounted on rigid board)
- 📋 Good even lighting setup (no shadows on board)

### Software
- ✅ BlueLily bridge running and verified (`./scripts/verify_bluelily_bridge.sh`)
- ✅ Kinect bridge publishing at 14.5 FPS
- 📋 Kalibr installed (Docker recommended)
- 📋 rosbag2 installed

### Verification Commands
```bash
# Check IMU is publishing
ros2 topic hz /imu/data  # Should be 100-800 Hz

# Check Kinect RGB is publishing
ros2 topic hz /kinect2/hd/image_color  # Should be ~14.5 Hz

# Check frame IDs are correct
ros2 topic echo /imu/data --once | grep frame_id  # Must be 'imu_link'
ros2 topic echo /kinect2/hd/image_color --once | grep frame_id  # Should be 'kinect2_rgb_optical_frame'
```

---

## Step 1 — Verify Kinect Intrinsics

Kinect v2 has factory calibration, but verify per-unit values:

```bash
# Launch Kinect bridge
ros2 launch kinect2_ros2_cuda kinect2_bridge.launch.py

# In another terminal, check intrinsics
ros2 topic echo /kinect2/hd/camera_info --once
```

### Expected Values (Kinect v2 HD typical)
```yaml
K: [1081.37, 0.0, 959.5,
    0.0, 1081.37, 539.5,
    0.0, 0.0, 1.0]

D: [0.0, 0.0, 0.0, 0.0, 0.0]  # Pre-rectified
```

**Save these exact values** — they go into ORB-SLAM3 config in Phase 2.

If your values differ significantly (>5%), use your actual values in all configs.

---

## Step 2 — Prepare Checkerboard Target

### Print and Mount
1. Print checkerboard pattern (8x6 corners, A3 size)
2. Mount on rigid board (foam board or cardboard)
3. Ensure board is perfectly flat (no warping)

### Measure Precisely
Measure the distance between corner centers (not square edges):

```
rowSpacingMeters: 0.030  # Measure YOUR board exactly
colSpacingMeters: 0.030  # Typically 30mm for A3 8x6
```

**Critical**: Measure with calipers or ruler. 1mm error = bad calibration.

### Pattern Specifications
- **Corners**: 8 columns × 6 rows (internal corners)
- **Squares**: 9 × 7 (one more than corners)
- **Size**: Each square ~30mm × 30mm for A3 paper
- **Colors**: Black and white, high contrast
- **Material**: Matte finish (no glare)

Download pattern: https://github.com/ethz-asl/kalibr/wiki/downloads

---

## Step 3 — Create Kalibr Configuration Files

### 3.1 IMU Configuration

Create `kalibr_configs/bluelily_imu.yaml`:

```yaml
#Accelerometers
accelerometer_noise_density: 0.01      # [m/s^2/sqrt(Hz)] (MPU6500 typical)
accelerometer_random_walk: 0.001       # [m/s^3/sqrt(Hz)]

#Gyroscopes  
gyroscope_noise_density: 0.001         # [rad/s/sqrt(Hz)] (MPU6500 typical)
gyroscope_random_walk: 0.0001          # [rad/s^2/sqrt(Hz)]

rostopic: /imu/data
update_rate: 100.0  # BlueLily firmware configured for 100Hz
```

**Note**: These are MPU6500 typical values. Kalibr will refine them.

### 3.2 Camera Configuration

Create `kalibr_configs/kinect2_cam.yaml`:

```yaml
cam0:
  camera_model: pinhole
  distortion_model: radtan
  distortion_coeffs: [0.0, 0.0, 0.0, 0.0]  # Kinect pre-rectifies
  intrinsics: [1081.37, 1081.37, 959.5, 539.5]  # Replace with YOUR values from Step 1
  resolution: [1920, 1080]
  rostopic: /kinect2/hd/image_color
  timeshift_cam_imu: 0.0  # Initial estimate, Kalibr will optimize
```

**Important**: Replace intrinsics with your actual values from Step 1.

### 3.3 Target Configuration

Create `kalibr_configs/target.yaml`:

```yaml
target_type: 'checkerboard'
targetCols: 8  # Number of internal corners (columns)
targetRows: 6  # Number of internal corners (rows)
rowSpacingMeters: 0.030  # MEASURE YOUR BOARD EXACTLY
colSpacingMeters: 0.030  # MEASURE YOUR BOARD EXACTLY
```

**Critical**: Measure your printed board with calipers. Use exact values.

---

## Step 4 — Record Calibration Bag

### 4.1 Setup
1. Mount robot head on stable base (or hold steady)
2. Position checkerboard in front of camera (~1-2 meters)
3. Ensure good even lighting (no shadows on board)
4. Launch both bridges:

```bash
# Terminal 1: Kinect bridge
ros2 launch kinect2_ros2_cuda kinect2_bridge.launch.py

# Terminal 2: BlueLily bridge
ros2 launch bluelily_bridge bluelily_imu.launch.py

# Terminal 3: Verify both publishing
ros2 topic hz /kinect2/hd/image_color  # ~14.5 Hz
ros2 topic hz /imu/data                # 100-800 Hz
```

### 4.2 Recording Strategy

**Goal**: Excite all 6 degrees of freedom + all IMU axes

Move the checkerboard (or robot head) through:
- ✅ Translation: X (left/right), Y (up/down), Z (forward/back)
- ✅ Rotation: Roll, Pitch, Yaw
- ✅ Include some fast rotations to excite gyroscopes
- ✅ Include some linear accelerations
- ✅ Keep board fully visible throughout (no partial occlusions)
- ✅ Vary distance: 0.5m to 3m from camera

**Duration**: 60-90 seconds minimum

### 4.3 Record

```bash
# Terminal 4: Record bag
ros2 bag record \
  /kinect2/hd/image_color \
  /imu/data \
  -o kalibr_calib
```

**Expected file size**: ~500MB for 60 seconds

### 4.4 Verify Recording

```bash
ros2 bag info kalibr_calib

# Should show:
# - /kinect2/hd/image_color: ~870 messages (14.5 Hz × 60s)
# - /imu/data: ~6000-48000 messages (100-800 Hz × 60s)
```

---

## Step 5 — Run Kalibr

### 5.1 Install Kalibr (Docker)

```bash
# Pull Kalibr Docker image
docker pull stereolabs/kalibr

# Create alias for convenience
alias kalibr='docker run -it --rm \
  -v $(pwd):/data \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  stereolabs/kalibr'
```

### 5.2 Convert ROS2 bag to ROS1 (if needed)

Kalibr requires ROS1 bags. Convert:

```bash
# Install rosbags tool
pip install rosbags

# Convert
rosbags-convert kalibr_calib --dst kalibr_calib_ros1
```

### 5.3 Run Calibration

```bash
cd kalibr_configs

kalibr_calibrate_imu_camera \
  --bag ../kalibr_calib_ros1/kalibr_calib_ros1_0.db3 \
  --cam kinect2_cam.yaml \
  --imu bluelily_imu.yaml \
  --target target.yaml \
  --show-extraction \
  --verbose
```

**Expected runtime**: 10-30 minutes depending on bag length

### 5.4 Monitor Progress

Kalibr will show:
1. Corner detection visualization (green dots on checkerboard)
2. Optimization progress (reprojection error decreasing)
3. Final results with statistics

**Good calibration indicators**:
- Reprojection error < 1.0 pixel
- All corners detected in >80% of frames
- Optimization converges smoothly

---

## Step 6 — Extract Results

### 6.1 Critical Output File

```bash
ls kalibr_configs/

# Look for:
# - camchain-imucam-<timestamp>.yaml  ← THIS IS THE CRITICAL FILE
# - imu-<timestamp>.yaml
# - results-imucam-<timestamp>.txt
# - report-imucam-<timestamp>.pdf
```

### 6.2 Inspect T_cam_imu Transform

Open `camchain-imucam-<timestamp>.yaml`:

```yaml
cam0:
  T_cam_imu:
  - [r00, r01, r02, tx]
  - [r10, r11, r12, ty]
  - [r20, r21, r22, tz]
  - [0.0, 0.0, 0.0, 1.0]
```

This 4×4 matrix is the transform from IMU frame to camera frame.

**Sanity checks**:
- Translation (tx, ty, tz) should be small (<0.2m) if IMU and camera are close
- Rotation should be near identity if both mounted similarly
- No NaN or inf values

### 6.3 Review Calibration Report

Open `report-imucam-<timestamp>.pdf`:

Check:
- ✅ Reprojection error < 1.0 pixel (ideally < 0.5)
- ✅ Gravity magnitude ≈ 9.81 m/s²
- ✅ No large outliers in residual plots
- ✅ Covariance ellipses are small

---

## Step 7 — Update TF2 Static Transform

### 7.1 Convert Rotation Matrix to Quaternion

Create `scripts/kalibr_to_tf2.py`:

```python
#!/usr/bin/env python3
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
import sys

if len(sys.argv) < 2:
    print("Usage: ./kalibr_to_tf2.py camchain-imucam.yaml")
    sys.exit(1)

# Load Kalibr results
with open(sys.argv[1], 'r') as f:
    data = yaml.safe_load(f)

T_cam_imu = np.array(data['cam0']['T_cam_imu'])

# Extract translation
tx, ty, tz = T_cam_imu[0:3, 3]

# Extract rotation and convert to quaternion
R = T_cam_imu[0:3, 0:3]
rot = Rotation.from_matrix(R)
qx, qy, qz, qw = rot.as_quat()

print("=" * 60)
print("TF2 Static Transform Publisher Arguments")
print("=" * 60)
print(f"\nTranslation (meters):")
print(f"  x: {tx:.6f}")
print(f"  y: {ty:.6f}")
print(f"  z: {tz:.6f}")
print(f"\nRotation (quaternion):")
print(f"  qx: {qx:.6f}")
print(f"  qy: {qy:.6f}")
print(f"  qz: {qz:.6f}")
print(f"  qw: {qw:.6f}")
print(f"\nLaunch file snippet:")
print(f"""
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='imu_to_kinect_tf',
    arguments=[
        '{tx:.6f}', '{ty:.6f}', '{tz:.6f}',
        '{qx:.6f}', '{qy:.6f}', '{qz:.6f}', '{qw:.6f}',
        'imu_link', 'kinect2_link'
    ]
)
""")
print("=" * 60)
```

### 7.2 Run Conversion

```bash
chmod +x scripts/kalibr_to_tf2.py
./scripts/kalibr_to_tf2.py kalibr_configs/camchain-imucam-<timestamp>.yaml
```

### 7.3 Update Launch File

Copy the output and update `ros2_ws/src/bluelily_bridge/launch/bluelily_imu.launch.py`:

Replace the placeholder transform with Kalibr-calibrated values.

---

## Step 8 — Verify Calibration

### 8.1 Launch with New Transform

```bash
# Rebuild if needed
cd ros2_ws
colcon build --packages-select bluelily_bridge --symlink-install
source install/setup.bash

# Launch
ros2 launch bluelily_bridge bluelily_imu.launch.py
```

### 8.2 Check TF2 Chain

```bash
# Verify transform exists
ros2 run tf2_ros tf2_echo imu_link kinect2_link

# Should show your calibrated transform values

# Generate full TF tree
ros2 run tf2_tools view_frames
evince frames.pdf
```

Expected chain: `imu_link → kinect2_link → kinect2_rgb_optical_frame`

---

## Outputs of Phase 1

### Critical Files (Save These!)

1. **`camchain-imucam-<timestamp>.yaml`**
   - T_cam_imu transform for ORB-SLAM3 config (Phase 2)
   - Refined IMU noise parameters

2. **`kinect2_intrinsics.txt`**
   - fx, fy, cx, cy for ORB-SLAM3 config (Phase 2)

3. **Updated `bluelily_imu.launch.py`**
   - Calibrated TF2 static transform

4. **`report-imucam-<timestamp>.pdf`**
   - Calibration quality metrics
   - Keep for reference

### Checklist Before Phase 2

- ✅ Reprojection error < 1.0 pixel
- ✅ T_cam_imu has no NaN/inf values
- ✅ TF2 transform updated in launch file
- ✅ `ros2 run tf2_ros tf2_echo imu_link kinect2_link` shows calibrated values
- ✅ All files backed up to `calibration_results/` folder

---

## Common Issues & Solutions

### Issue: Kalibr can't detect corners
**Symptoms**: "No corners detected" or very few frames used  
**Causes**:
- Poor lighting (shadows on board)
- Motion blur (moving too fast)
- Board not flat (warped)
- Wrong target.yaml dimensions

**Solutions**:
- Improve lighting (diffuse, even)
- Move slower during recording
- Remount board on rigid surface
- Verify target.yaml matches printed board

### Issue: High reprojection error (>1.5 pixels)
**Symptoms**: Calibration completes but error is high  
**Causes**:
- Incorrect target spacing in target.yaml
- Board not flat
- Rolling shutter effects
- Insufficient motion variety

**Solutions**:
- Re-measure board with calipers
- Use rigid mounting
- Record new bag with more varied motion
- Ensure board fills 30-80% of frame

### Issue: Optimization doesn't converge
**Symptoms**: Error increases or oscillates  
**Causes**:
- IMU and camera timestamps not synchronized
- Wrong IMU noise parameters
- Insufficient excitation

**Solutions**:
- Check `timeshift_cam_imu` parameter
- Verify IMU publishing at correct rate
- Record new bag with faster rotations
- Adjust IMU noise parameters

### Issue: T_cam_imu has large translation (>0.5m)
**Symptoms**: Transform shows IMU far from camera  
**Causes**:
- IMU and camera not rigidly mounted
- Calibration failed (local minimum)
- Wrong frame assignments

**Solutions**:
- Verify physical mounting is rigid
- Check frame_id in both topics
- Re-run calibration with better data
- Measure physical offset and compare

### Issue: Gravity magnitude not ~9.81 m/s²
**Symptoms**: Report shows gravity ≠ 9.81  
**Causes**:
- IMU accelerometer scale factor wrong
- BlueLily firmware not sending correct units
- IMU not calibrated

**Solutions**:
- Verify BlueLily firmware has ROS2Bridge enabled
- Check units in bridge (should be m/s²)
- Recalibrate IMU in BlueLily firmware

---

## Next Steps

✅ **Phase 1 Complete** when:
- Calibration report shows error < 1.0 pixel
- T_cam_imu saved and understood
- TF2 transform updated and verified

➡️ **Proceed to Phase 2**: ORB-SLAM3 Integration
- Use T_cam_imu in ORB-SLAM3 config
- Use refined IMU noise parameters
- Use Kinect intrinsics in camera config

See: `docs/PHASE2_ORBSLAM3_INTEGRATION.md`
