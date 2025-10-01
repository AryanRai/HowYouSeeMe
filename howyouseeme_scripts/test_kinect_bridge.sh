#!/bin/bash
echo "🧪 Testing Kinect2 Bridge..."
source ~/ros2_ws/install/setup.bash
export LD_LIBRARY_PATH=$HOME/libfreenect2-modern/lib:$LD_LIBRARY_PATH

echo "Starting kinect2_bridge for 10 seconds..."
timeout 10s ros2 run kinect2_bridge kinect2_bridge_node &
BRIDGE_PID=$!

sleep 3

echo "Checking for topics..."
ros2 topic list | grep kinect2 || echo "No kinect2 topics found yet"

echo "Waiting for bridge to finish..."
wait $BRIDGE_PID

echo "Test completed!"
