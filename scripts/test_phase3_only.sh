#!/bin/bash
# Test Phase 3 TSDF Integration without ORB-SLAM3
# Uses a mock pose publisher for testing

echo "=== Testing Phase 3 TSDF Integration ==="
echo ""
echo "This test runs TSDF integration with a static pose"
echo "Useful for testing the TSDF pipeline without ORB-SLAM3"
echo ""
echo "Terminal 1: Kinect publisher"
echo "Terminal 2: Mock pose publisher (publishes static pose)"
echo "Terminal 3: TSDF integrator"
echo ""
echo "Press Enter to continue..."
read

# Source ROS
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

# Run TSDF integrator
ros2 run kinect2_slam tsdf_integrator \
    --ros-args \
    -p voxel_length:=0.04 \
    -p sdf_trunc:=0.08 \
    -p publish_rate:=1.0
