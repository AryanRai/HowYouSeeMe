#!/bin/bash
# Launch RViz with Kinect configuration

source /opt/ros/jazzy/setup.bash
source /home/aryan/Documents/GitHub/HowYouSeeMe/ros2_ws/install/setup.bash

echo "Launching RViz2 with Kinect configuration..."
echo ""
echo "Make sure the Kinect publisher is running first:"
echo "  ros2 run kinect2_simple_publisher kinect2_simple_publisher_node"
echo ""
echo "Or use: ./launch_kinect_complete.sh"
echo ""

rviz2 -d /home/aryan/Documents/GitHub/HowYouSeeMe/kinect2_rviz_config.rviz
