#!/bin/bash
# Launch Complete System: Kinect + SLAM + SAM2 Server + RViz
# Shows SLAM map, point clouds, and SAM2 segmentation visualizations

# Set library path for libfreenect2
export LD_LIBRARY_PATH=/home/aryan/Documents/GitHub/HowYouSeeMe/libfreenect2/freenect2/lib:$LD_LIBRARY_PATH

# Source ROS2
source /opt/ros/jazzy/setup.bash
source /home/aryan/Documents/GitHub/HowYouSeeMe/ros2_ws/install/setup.bash

echo "=========================================="
echo "  Complete Vision System with RViz"
echo "=========================================="
echo ""
echo "Components:"
echo "  1. Kinect v2 RGB-D Camera"
echo "  2. RTABMap SLAM (3D Mapping)"
echo "  3. SAM2 Server (Segmentation)"
echo "  4. RViz2 (Visualization)"
echo ""
echo "=========================================="
echo ""

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "Shutting down all components..."
    kill $KINECT_PID $RTABMAP_PID $SAM2_PID $RVIZ_PID 2>/dev/null
    wait $KINECT_PID $RTABMAP_PID $SAM2_PID $RVIZ_PID 2>/dev/null
    echo "All processes stopped"
    exit 0
}

trap cleanup SIGINT SIGTERM

# 1. Start Kinect2 ROS2 bridge
echo "1/4 Starting Kinect2 bridge..."
ros2 launch kinect2_bridge kinect2_bridge_launch.yaml &
KINECT_PID=$!
echo "    PID: $KINECT_PID"
echo "    Waiting for Kinect to initialize..."
sleep 8

# Check if topics are available
echo ""
echo "2/4 Verifying Kinect topics..."
RETRY=0
MAX_RETRIES=5
while [ $RETRY -lt $MAX_RETRIES ]; do
    if ros2 topic list 2>/dev/null | grep -q "/kinect2/qhd/image_color"; then
        echo "    ✅ Kinect topics available"
        break
    else
        RETRY=$((RETRY+1))
        if [ $RETRY -lt $MAX_RETRIES ]; then
            echo "    ⏳ Waiting for topics... (attempt $RETRY/$MAX_RETRIES)"
            sleep 2
        else
            echo "    ❌ Kinect topics not found after $MAX_RETRIES attempts!"
            echo "    Continuing anyway..."
        fi
    fi
done

# 2. Start RTABMap SLAM
echo ""
echo "3/4 Starting RTABMap SLAM..."
echo "    This may take a few seconds..."
ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start \
                   --Mem/IncrementalMemory true \
                   --Mem/InitWMWithAllNodes false \
                   --Rtabmap/DetectionRate 1 \
                   --RGBD/ProximityBySpace true \
                   --RGBD/ProximityMaxGraphDepth 50 \
                   --RGBD/ProximityPathMaxNeighbors 3 \
                   --RGBD/AngularUpdate 0.05 \
                   --RGBD/LinearUpdate 0.05 \
                   --RGBD/OptimizeFromGraphEnd false \
                   --Mem/STMSize 30 \
                   --Mem/BadSignaturesIgnored true \
                   --Rtabmap/TimeThr 0 \
                   --Rtabmap/MemoryThr 0 \
                   --Rtabmap/LoopThr 0.15 \
                   --Rtabmap/LoopRatio 0.9 \
                   --Grid/MaxObstacleHeight 2.0 \
                   --Grid/MaxGroundHeight 0.0 \
                   --Grid/CellSize 0.05 \
                   --Grid/RangeMax 4.0 \
                   --Grid/ClusterRadius 0.1 \
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
                --OdomF2M/ScanSubtractRadius 0.05 \
                --OdomF2M/ScanMaxSize 15000 \
                --OdomF2M/MaxSize 2000 \
                --Icp/PointToPlane true \
                --Icp/Iterations 30 \
                --Icp/VoxelSize 0.05 \
                --Icp/Epsilon 0.001 \
                --Icp/PointToPlaneK 20 \
                --Icp/PointToPlaneRadius 0.5 \
                --Icp/MaxTranslation 0.3 \
                --Icp/MaxCorrespondenceDistance 0.15 \
                --Icp/PM false \
                --Icp/PMOutlierRatio 0.85 \
                --Icp/CorrespondenceRatio 0.2 \
                --Odom/ScanKeyFrameThr 0.7 \
                --OdomF2M/ScanSubtractAngle 45 \
                --OdomF2M/ScanRange 4.0 \
                --Odom/GuessMotion true \
                --Odom/Holonomic false" &
RTABMAP_PID=$!
echo "    PID: $RTABMAP_PID"
sleep 3

# 3. Start SAM2 Server
echo ""
echo "4/5 Starting SAM2 Server (loading model)..."
echo "    This will take ~2-3 seconds to load the model..."
source /home/aryan/anaconda3/bin/activate howyouseeme
python3 /home/aryan/Documents/GitHub/HowYouSeeMe/ros2_ws/src/cv_pipeline/python/sam2_server.py &
SAM2_PID=$!
echo "    PID: $SAM2_PID"
echo "    Waiting for SAM2 model to load..."
sleep 4

# 4. Start RViz with comprehensive config
echo ""
echo "5/5 Starting RViz2 with full visualization..."
if [ -f "/home/aryan/Documents/GitHub/HowYouSeeMe/full_system_rviz.rviz" ]; then
    rviz2 -d /home/aryan/Documents/GitHub/HowYouSeeMe/full_system_rviz.rviz &
else
    echo "    ⚠️  Custom config not found, using default SLAM config"
    rviz2 -d /home/aryan/Documents/GitHub/HowYouSeeMe/kinect2_slam_rviz.rviz &
fi
RVIZ_PID=$!
echo "    PID: $RVIZ_PID"
sleep 2

echo ""
echo "=========================================="
echo "  🎉 System Ready!"
echo "=========================================="
echo ""
echo "Process IDs:"
echo "  Kinect Bridge:  $KINECT_PID"
echo "  RTABMap SLAM:   $RTABMAP_PID"
echo "  SAM2 Server:    $SAM2_PID"
echo "  RViz2:          $RVIZ_PID"
echo ""
echo "RViz Displays:"
echo "  📷 Camera Image - /kinect2/qhd/image_color"
echo "  🎨 SAM2 Results - /cv_pipeline/visualization"
echo "  🗺️  SLAM Map     - /rtabmap/mapData"
echo "  ☁️  Point Cloud  - /kinect2/qhd/points"
echo "  🧭 TF Frames    - kinect2_link, map, odom"
echo ""
echo "SAM2 Commands:"
echo "  Send request:"
echo "    ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \\"
echo "      \"data: 'sam2:prompt_type=point'\""
echo ""
echo "  View results:"
echo "    ros2 topic echo /cv_pipeline/results --once"
echo ""
echo "Move the Kinect to build a 3D map!"
echo "Press Ctrl+C to stop all processes"
echo "=========================================="

# Wait for processes
wait $KINECT_PID $RTABMAP_PID $SAM2_PID $RVIZ_PID
