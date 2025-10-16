#!/bin/bash
# Simple test script to verify Kinect topics without blocking

# Set library path for libfreenect2
export LD_LIBRARY_PATH=/home/aryan/Documents/GitHub/HowYouSeeMe/libfreenect2/freenect2/lib:$LD_LIBRARY_PATH

# Source ROS2
source /opt/ros/jazzy/setup.bash
source /home/aryan/Documents/GitHub/HowYouSeeMe/ros2_ws/install/setup.bash

echo "Starting Kinect publisher in background..."
ros2 run kinect2_simple_publisher kinect2_simple_publisher_node &
KINECT_PID=$!

echo "Waiting for initialization..."
sleep 5

echo ""
echo "=========================================="
echo "ROS2 Topics:"
echo "=========================================="
ros2 topic list | grep kinect2

echo ""
echo "=========================================="
echo "Topic Info - RGB:"
echo "=========================================="
ros2 topic info /kinect2/sd/image_color

echo ""
echo "=========================================="
echo "Topic Info - Depth:"
echo "=========================================="
ros2 topic info /kinect2/sd/image_depth

echo ""
echo "=========================================="
echo "Checking topic rates (5 seconds each)..."
echo "=========================================="
echo ""
echo "RGB rate:"
timeout 5 ros2 topic hz /kinect2/sd/image_color

echo ""
echo "Depth rate:"
timeout 5 ros2 topic hz /kinect2/sd/image_depth

echo ""
echo "=========================================="
echo "Sample RGB message:"
echo "=========================================="
timeout 2 ros2 topic echo /kinect2/sd/image_color --once

echo ""
echo "=========================================="
echo "Test complete! Stopping publisher..."
echo "=========================================="
kill $KINECT_PID
wait $KINECT_PID 2>/dev/null

echo "Done!"
