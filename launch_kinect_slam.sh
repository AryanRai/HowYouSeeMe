#!/bin/bash
# Launch Kinect v2 with RTABMap SLAM

# Set library path for libfreenect2
export LD_LIBRARY_PATH=/home/aryan/Documents/GitHub/HowYouSeeMe/libfreenect2/freenect2/lib:$LD_LIBRARY_PATH

# Source ROS2
source /opt/ros/jazzy/setup.bash
source /home/aryan/Documents/GitHub/HowYouSeeMe/ros2_ws/install/setup.bash

echo "=========================================="
echo "Kinect v2 + RTABMap SLAM System"
echo "=========================================="
echo ""

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "Shutting down..."
    kill $TF1_PID $TF2_PID $TF3_PID $KINECT_PID $RTABMAP_PID $RVIZ_PID 2>/dev/null
    wait $TF1_PID $TF2_PID $TF3_PID $KINECT_PID $RTABMAP_PID $RVIZ_PID 2>/dev/null
    echo "All processes stopped"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Start static transform publishers
echo "1. Starting TF publishers..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom &
TF1_PID=$!
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom kinect2_link &
TF2_PID=$!
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 kinect2_link kinect2_rgb_optical_frame &
TF3_PID=$!
sleep 1

# Start Kinect publisher
echo "2. Starting Kinect v2 publisher..."
ros2 run kinect2_simple_publisher kinect2_simple_publisher_node &
KINECT_PID=$!
sleep 3

# Check if topics are available
echo "3. Checking topics..."
ros2 topic list | grep kinect2
echo ""

# Start RTABMap
echo "4. Starting RTABMap SLAM..."
ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start" \
    rgb_topic:=/kinect2/sd/image_color \
    depth_topic:=/kinect2/sd/image_depth \
    camera_info_topic:=/kinect2/sd/camera_info \
    frame_id:=kinect2_link \
    approx_sync:=true \
    decimation:=2 \
    wait_imu_to_init:=false \
    qos:=2 &
RTABMAP_PID=$!

echo "   PID: $RTABMAP_PID"
sleep 3

# Start RViz with SLAM config
echo "5. Starting RViz2..."
rviz2 -d /home/aryan/Documents/GitHub/HowYouSeeMe/kinect2_slam_rviz.rviz &
RVIZ_PID=$!

echo ""
echo "=========================================="
echo "System Running!"
echo "=========================================="
echo "TF Publishers: $TF1_PID, $TF2_PID, $TF3_PID"
echo "Kinect Publisher: $KINECT_PID"
echo "RTABMap SLAM: $RTABMAP_PID"
echo "RViz: $RVIZ_PID"
echo ""
echo "Move the Kinect around to build a map!"
echo "Press Ctrl+C to stop"
echo "=========================================="

# Wait for processes
wait $TF1_PID $TF2_PID $TF3_PID $KINECT_PID $RTABMAP_PID $RVIZ_PID
