# ORB-SLAM3 Configuration Files

Configuration files for ORB-SLAM3 RGB-D+IMU mode with Kinect v2 and BlueLily IMU.

## Files

- `Kinect2_RGBD_IMU.yaml` - Main ORB-SLAM3 configuration for RGB-D+IMU fusion

## Usage

1. Run Phase 1 Kalibr calibration to generate `kalibr_output/camchain-imucam.yaml`
2. Convert Kalibr results to ORB-SLAM3 format:
   ```bash
   python3 scripts/kalibr_to_orb_slam3.py
   ```
3. This automatically updates `Kinect2_RGBD_IMU.yaml` with:
   - Camera intrinsics (fx, fy, cx, cy)
   - IMU-to-camera transformation matrix
   - Resolution and frame rate

## Manual Configuration

If you need to manually edit the config:

### Camera Intrinsics
From `kalibr_output/camchain-imucam.yaml` cam0 intrinsics:
- `Camera.fx` - Focal length X
- `Camera.fy` - Focal length Y  
- `Camera.cx` - Principal point X
- `Camera.cy` - Principal point Y

### IMU Transform
From `kalibr_output/camchain-imucam.yaml` cam0 T_cam_imu:
- `IMU.T_b_c1` - 4x4 transformation matrix from IMU body frame to camera frame

### IMU Noise Parameters
Tuned for BlueLily MPU9250 at 800Hz. Adjust if tracking is unstable.
