#!/bin/bash
echo "🚀 Starting Kinect2 Bridge..."
source ~/ros2_ws/install/setup.bash
export LD_LIBRARY_PATH=$HOME/libfreenect2-modern/lib:$LD_LIBRARY_PATH
ros2 run kinect2_bridge kinect2_bridge_node
