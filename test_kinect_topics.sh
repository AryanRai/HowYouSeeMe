#!/bin/bash
# Test script to verify Kinect topics are publishing

# Set library path for libfreenect2
export LD_LIBRARY_PATH=/home/aryan/Documents/GitHub/HowYouSeeMe/libfreenect2/freenect2/lib:$LD_LIBRARY_PATH

# Source ROS2
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

echo "Checking ROS2 topics..."
echo "Available Kinect topics:"
ros2 topic list | grep kinect2

echo ""
echo "Checking topic rates (Ctrl+C to stop):"
ros2 topic hz /kinect2/qhd/image_color
