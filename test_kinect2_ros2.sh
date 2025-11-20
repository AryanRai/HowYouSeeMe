#!/bin/bash
# Test the kinect2_ros2 bridge

# Set library path for libfreenect2
export LD_LIBRARY_PATH=/home/aryan/Documents/GitHub/HowYouSeeMe/libfreenect2/freenect2/lib:$LD_LIBRARY_PATH

# Source ROS2
source /opt/ros/jazzy/setup.bash
source /home/aryan/Documents/GitHub/HowYouSeeMe/ros2_ws/install/setup.bash

echo "=========================================="
echo "Testing Kinect2 ROS2 Bridge (CUDA Fork)"
echo "CPU Registration Enabled"
echo "=========================================="
echo ""
echo "Starting kinect2_bridge..."
echo "Topics will be published under /kinect2/"
echo ""
echo "Press Ctrl+C to stop"
echo "=========================================="

# Launch the bridge
ros2 launch kinect2_bridge kinect2_bridge_launch.yaml
