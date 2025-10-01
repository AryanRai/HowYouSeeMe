#!/bin/bash
echo "🧠 Starting HowYouSeeMe ROS2 Subscriber..."
source ~/ros2_ws/install/setup.bash
ros2 run howyouseeme_ros2 kinect_subscriber
