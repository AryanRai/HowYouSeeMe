#!/bin/bash
# Launch Kinect v2 with RTABMap SLAM + IMU fusion (standalone version)
#
# SLAM Precision Improvements (v3):
#   Roll-flip fix:
#     - Publishes kinect2_slam_body (parent, ROS X-forward/Y-left/Z-up) → kinect2_link
#       so physical roll-right appears as roll-right in the RTABMap viewer.
#   Ghosting reduction:
#     - Reg/Strategy 2 (Visual+ICP loop-closure refinement)
#     - OptimizeMaxError 0.5 m (tightened from 1.0 m)
#   Speed-up:
#     - ICP iterations 50→30, PointToPlaneK 30→20
#     - ScanMaxSize 20k→15k, MaxFeatures 1000→600
#     - DetectionRate 1 Hz→2 Hz

# Set library path for libfreenect2
export LD_LIBRARY_PATH=/home/aryan/Documents/GitHub/HowYouSeeMe/libfreenect2/freenect2/lib:$LD_LIBRARY_PATH

# Source ROS2
source /opt/ros/jazzy/setup.bash
source /home/aryan/Documents/GitHub/HowYouSeeMe/ros2_ws/install/setup.bash

echo "=========================================="
echo "Kinect v2 + RTABMap SLAM + IMU Fusion (v3)"
echo "Using kinect2_ros2_cuda bridge (CPU mode)"
echo "WITH ROLL-CORRECTED COORDINATE FRAMES"
echo "=========================================="
echo ""

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "Shutting down..."
    kill $IMU_PID $KINECT_PID $IMU_FILTER_PID $SLAM_TF_PID $RTABMAP_PID $RVIZ_PID 2>/dev/null
    wait $IMU_PID $KINECT_PID $IMU_FILTER_PID $SLAM_TF_PID $RTABMAP_PID $RVIZ_PID 2>/dev/null
    echo "All processes stopped"
    exit 0
}

trap cleanup SIGINT SIGTERM

# 1. Start IMU node
echo "1. Starting IMU node..."
ros2 run bno055 bno055_node &
IMU_PID=$!
sleep 2

# 2. Start IMU filter for better orientation estimation
echo "2. Starting IMU filter (optional but recommended)..."
ros2 run imu_filter_madgwick imu_filter_madgwick_node \
    --ros-args \
    -p use_mag:=true \
    -p publish_tf:=false \
    -p world_frame:=enu &
IMU_FILTER_PID=$!
sleep 1

# 3. Start Kinect2 ROS2 bridge
echo "3. Starting Kinect2 ROS2 bridge..."
ros2 launch kinect2_bridge kinect2_bridge_launch.yaml &
KINECT_PID=$!
sleep 5

# Check if topics are available
echo "4. Checking topics..."
ros2 topic list | grep -E "kinect2|imu"
echo ""

# 5. Publish roll-correction TF: kinect2_slam_body (parent) → kinect2_link (child)
# Quaternion [qx=-0.5, qy=0.5, qz=-0.5, qw=0.5] converts the Kinect's optical
# convention (X=right, Y=down, Z=forward) to ROS REP-103 (X=forward, Y=left, Z=up).
echo "5. Publishing roll-correction TF: kinect2_slam_body → kinect2_link"
ros2 run tf2_ros static_transform_publisher \
    --frame-id kinect2_slam_body \
    --child-frame-id kinect2_link \
    --x 0 --y 0 --z 0 \
    --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5 &
SLAM_TF_PID=$!
echo "   PID: $SLAM_TF_PID"
sleep 1

# 6. Start RTABMap with ICP-based odometry + IMU fusion (v3 precision+speed mode)
echo "6. Starting RTABMap SLAM with ICP odometry + IMU (v3 precision mode)..."
ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start \
                   --Mem/IncrementalMemory true \
                   --Mem/InitWMWithAllNodes false \
                   --Mem/STMSize 50 \
                   --Mem/BadSignaturesIgnored true \
                   --Rtabmap/DetectionRate 2 \
                   --Reg/Strategy 2 \
                   --RGBD/ProximityBySpace true \
                   --RGBD/ProximityMaxGraphDepth 50 \
                   --RGBD/ProximityPathMaxNeighbors 3 \
                   --RGBD/AngularUpdate 0.03 \
                   --RGBD/LinearUpdate 0.03 \
                   --RGBD/OptimizeFromGraphEnd true \
                   --RGBD/OptimizeMaxError 0.5 \
                   --RGBD/NeighborLinkRefining true \
                   --Optimizer/Robust true \
                   --Optimizer/Iterations 20 \
                   --Vis/MaxFeatures 600 \
                   --Vis/MinInliers 25 \
                   --Vis/InlierDistance 0.05 \
                   --Rtabmap/TimeThr 0 \
                   --Rtabmap/MemoryThr 0 \
                   --Rtabmap/LoopThr 0.11 \
                   --Rtabmap/LoopRatio 0.95 \
                   --Grid/MaxObstacleHeight 2.0 \
                   --Grid/MaxGroundHeight 0.0 \
                   --Grid/CellSize 0.025 \
                   --Grid/RangeMax 4.0 \
                   --Grid/ClusterRadius 0.05 \
                   --Grid/GroundIsObstacle false" \
    rgb_topic:=/kinect2/qhd/image_color_rect \
    depth_topic:=/kinect2/qhd/image_depth_rect \
    camera_info_topic:=/kinect2/qhd/camera_info \
    imu_topic:=/imu/data \
    frame_id:=kinect2_slam_body \
    approx_sync:=true \
    wait_imu_to_init:=true \
    qos:=2 \
    odom_args:="--Odom/Strategy 1 \
                --Odom/ResetCountdown 1 \
                --Odom/FilteringStrategy 1 \
                --Odom/ParticleSize 400 \
                --Odom/GuessMotion true \
                --Odom/Holonomic false \
                --Odom/ScanKeyFrameThr 0.7 \
                --OdomF2M/ScanSubtractRadius 0.025 \
                --OdomF2M/ScanSubtractAngle 45 \
                --OdomF2M/ScanMaxSize 15000 \
                --OdomF2M/ScanRange 4.0 \
                --OdomF2M/MaxSize 2000 \
                --Icp/PointToPlane true \
                --Icp/Iterations 30 \
                --Icp/VoxelSize 0.025 \
                --Icp/Epsilon 0.0001 \
                --Icp/PointToPlaneK 20 \
                --Icp/PointToPlaneRadius 0.3 \
                --Icp/MaxTranslation 0.2 \
                --Icp/MaxCorrespondenceDistance 0.08 \
                --Icp/PM false \
                --Icp/PMOutlierRatio 0.65 \
                --Icp/CorrespondenceRatio 0.35" &
RTABMAP_PID=$!
echo "   PID: $RTABMAP_PID"
sleep 3

# 7. Start RViz with SLAM config
echo "7. Starting RViz2..."
rviz2 -d /home/aryan/Documents/GitHub/HowYouSeeMe/kinect2_slam_rviz.rviz &
RVIZ_PID=$!

echo ""
echo "=========================================="
echo "System Running! (v3 – roll-corrected + IMU)"
echo "=========================================="
echo "IMU Node:        $IMU_PID"
echo "IMU Filter:      $IMU_FILTER_PID"
echo "Kinect2 Bridge:  $KINECT_PID"
echo "Slam-Body TF:    $SLAM_TF_PID"
echo "RTABMap SLAM:    $RTABMAP_PID"
echo "RViz:            $RVIZ_PID"
echo ""
echo "frame_id = kinect2_slam_body (roll-corrected)"
echo "Reg/Strategy 2: Visual+ICP loop-closure refinement"
echo "DetectionRate 2 Hz: faster map building"
echo "IMU fusion enabled for better orientation tracking"
echo ""
echo "Move the Kinect around to build a map!"
echo "IMU will help with fast movements and rotations"
echo "Press Ctrl+C to stop"
echo "=========================================="

# Wait for processes
wait $IMU_PID $KINECT_PID $IMU_FILTER_PID $SLAM_TF_PID $RTABMAP_PID $RVIZ_PID
