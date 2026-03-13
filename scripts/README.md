# Scripts Directory

This directory contains all operational scripts for the HowYouSeeMe robot system.

## Main System Launchers

- `launch_robot_head.sh` - Launch complete robot head system (Kinect + BlueLily IMU + SLAM + CV Pipeline + RViz)
- `launch_full_system_rviz.sh` - Launch complete system with SLAM map, point clouds, and SAM2 segmentation visualizations
- `launch_kinect_sam2_server.sh` - Launch Kinect + CV Pipeline Server V2 (multi-model support)

## Component Launchers

- `launch_kinect2_ros2_slam_fixed_tf.sh` - Launch Kinect v2 with RTABMap SLAM using kinect2_ros2 bridge (corrected coordinate frames)
- `launch_kinect2_slam_with_imu.sh` - Launch Kinect v2 with RTABMap SLAM + IMU fusion
- `launch_bluelily_bridge.sh` - Launch BlueLily IMU bridge in ROS2 Humble Docker container
- `launch_rviz_kinect.sh` - Launch RViz with Kinect configuration

## Interactive Menus

- `cv_menu.sh` - Quick launcher for CV Pipeline Menu with system check
- `cv_pipeline_menu.sh` - Interactive CV Pipeline Menu with nested model-specific menus (auto-executes after parameter input)

## CV Pipeline Control

- `start_sam2_stream.sh` - Start SAM2 streaming mode for continuous segmentation
- `stop_sam2_stream.sh` - Stop SAM2/CV Pipeline streaming mode
- `stop_cv_streaming.sh` - Quick script to stop CV Pipeline streaming
- `reset_cv_server.sh` - Force reset CV Pipeline server state

## System Management

- `kill_all.sh` - Robust kill script for ALL Kinect, ROS2, SLAM, and CV pipeline processes
- `kill_kinect.sh` - Kill all Kinect and ROS2 related processes

## Installation Scripts

- `install_hsemotion.sh` - Install HSEmotion (high-speed emotion recognition)
- `install_insightface.sh` - Install InsightFace for face recognition

## Usage

All scripts are executable. Run them from the project root directory:

```bash
./scripts/launch_robot_head.sh
./scripts/cv_menu.sh
```

For scripts that accept parameters, run without arguments to see usage information.
