#!/bin/bash
"""
Start Kinect2 Bridge for HowYouSeeMe ROS2 System
"""

echo "🚀 Starting Kinect2 Bridge..."

# Set library path for libfreenect2
export LD_LIBRARY_PATH=$HOME/freenect2/lib:$LD_LIBRARY_PATH

# Source ROS2 and workspace
source /opt/ros/jazzy/setup.bash
cd src/ros2_ws
source install/setup.bash

# Check if Kinect is connected
if lsusb | grep -q "Microsoft Corp. Xbox NUI Sensor"; then
    echo "✅ Kinect v2 detected"
else
    echo "❌ Kinect v2 not found. Please connect your Kinect v2 device."
    exit 1
fi

# Start kinect2_bridge
echo "🔗 Starting kinect2_bridge_node..."
ros2 run kinect2_bridge kinect2_bridge_node