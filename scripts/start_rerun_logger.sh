#!/bin/bash
# Start Rerun C++ logger node

source ~/Documents/GitHub/HowYouSeeMe/ros2_ws/install/setup.bash

echo "Starting Rerun C++ logger..."
ros2 run kinect2_slam rerun_logger_cpp_node \
    --ros-args \
    -p recording_name:=howyouseeme \
    -p save_path:=/tmp/howyouseeme.rrd \
    -p rgb_downsample:=2 \
    -p pc_max_points:=50000
