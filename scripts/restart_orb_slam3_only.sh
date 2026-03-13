#!/bin/bash
# Restart only ORB-SLAM3 node (useful after coordinate frame fixes)

echo "Stopping ORB-SLAM3..."
pkill -f orb_slam3_node

sleep 2

echo "Starting ORB-SLAM3..."
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

ros2 run kinect2_slam orb_slam3_node --ros-args \
    -p voc_file:=$HOME/ORB_SLAM3/Vocabulary/ORBvoc.txt \
    -p settings_file:=$(pwd)/orb_slam3_configs/Kinect2_RGBD_IMU.yaml &

echo ""
echo "✓ ORB-SLAM3 restarted"
echo ""
echo "Watch the ORB-SLAM3 viewer window to verify tracking"
echo "Check RViz to see if camera pose is now correct"
echo ""
