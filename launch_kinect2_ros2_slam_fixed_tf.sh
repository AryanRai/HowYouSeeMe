#!/bin/bash
# Launch Kinect v2 with RTABMap SLAM using kinect2_ros2 bridge
# WITH CORRECTED COORDINATE FRAME ORIENTATION
#
# SLAM Precision Improvements (v2):
#   - Finer ICP voxels (2.5 cm) for sub-centimetre registration
#   - Tighter ICP correspondence distance (8 cm)
#   - More ICP iterations (50) and tighter convergence (1e-4)
#   - Robust pose-graph optimizer
#   - RGBD neighbor-link ICP refinement
#   - Larger STM window (50 nodes) for wider loop-closure search
#   - Stricter loop-closure threshold (0.11) + OptimizeMaxError guard

# Set library path for libfreenect2
export LD_LIBRARY_PATH=/home/aryan/Documents/GitHub/HowYouSeeMe/libfreenect2/freenect2/lib:$LD_LIBRARY_PATH

# Source ROS2
source /opt/ros/jazzy/setup.bash
source /home/aryan/Documents/GitHub/HowYouSeeMe/ros2_ws/install/setup.bash

echo "=========================================="
echo "Kinect v2 + RTABMap SLAM System"
echo "Using kinect2_ros2_cuda bridge (CPU mode)"
echo "WITH CORRECTED COORDINATE FRAMES"
echo "=========================================="
echo ""

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "Shutting down..."
    kill $KINECT_PID $RTABMAP_PID $RVIZ_PID 2>/dev/null
    wait $KINECT_PID $RTABMAP_PID $RVIZ_PID 2>/dev/null
    echo "All processes stopped"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Start Kinect2 ROS2 bridge first (it publishes TF)
echo "1. Starting Kinect2 ROS2 bridge..."
ros2 launch kinect2_bridge kinect2_bridge_launch.yaml &
KINECT_PID=$!
sleep 5

# Check if topics are available
echo "2. Checking topics..."
ros2 topic list | grep kinect2
echo ""

# The bridge publishes kinect2_link -> kinect2_rgb_optical_frame
# We'll use kinect2_link as the base frame for SLAM
# RTAB-Map will handle the orientation internally
echo "3. Starting RTABMap SLAM with ICP odometry (precision mode)..."
ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start \
                   --Mem/IncrementalMemory true \
                   --Mem/InitWMWithAllNodes false \
                   --Mem/STMSize 50 \
                   --Mem/BadSignaturesIgnored true \
                   --Rtabmap/DetectionRate 1 \
                   --RGBD/ProximityBySpace true \
                   --RGBD/ProximityMaxGraphDepth 50 \
                   --RGBD/ProximityPathMaxNeighbors 3 \
                   --RGBD/AngularUpdate 0.03 \
                   --RGBD/LinearUpdate 0.03 \
                   --RGBD/OptimizeFromGraphEnd true \
                   --RGBD/OptimizeMaxError 1.0 \
                   --RGBD/NeighborLinkRefining true \
                   --Optimizer/Robust true \
                   --Optimizer/Iterations 20 \
                   --Vis/MaxFeatures 1000 \
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
    frame_id:=kinect2_link \
    approx_sync:=true \
    wait_imu_to_init:=false \
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
                --OdomF2M/ScanMaxSize 20000 \
                --OdomF2M/ScanRange 4.0 \
                --OdomF2M/MaxSize 2000 \
                --Icp/PointToPlane true \
                --Icp/Iterations 50 \
                --Icp/VoxelSize 0.025 \
                --Icp/Epsilon 0.0001 \
                --Icp/PointToPlaneK 30 \
                --Icp/PointToPlaneRadius 0.3 \
                --Icp/MaxTranslation 0.2 \
                --Icp/MaxCorrespondenceDistance 0.08 \
                --Icp/PM false \
                --Icp/PMOutlierRatio 0.65 \
                --Icp/CorrespondenceRatio 0.35" &
RTABMAP_PID=$!

echo "   PID: $RTABMAP_PID"
sleep 3

# Start RViz with SLAM config
echo "4. Starting RViz2..."
rviz2 -d /home/aryan/Documents/GitHub/HowYouSeeMe/kinect2_slam_rviz.rviz &
RVIZ_PID=$!

echo ""
echo "=========================================="
echo "System Running!"
echo "=========================================="
echo "Kinect2 Bridge: $KINECT_PID"
echo "RTABMap SLAM: $RTABMAP_PID"
echo "RViz: $RVIZ_PID"
echo ""
echo "Using kinect2_ros2_cuda bridge with QHD resolution"
echo "CPU registration enabled with proper calibration"
echo "Using kinect2_link as base frame"
echo ""
echo "Move the Kinect around to build a map!"
echo "Press Ctrl+C to stop"
echo "=========================================="

# Wait for processes
wait $KINECT_PID $RTABMAP_PID $RVIZ_PID
