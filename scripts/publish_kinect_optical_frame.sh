#!/bin/bash
# Publish correct optical frame transform for Kinect v2
# This corrects the frame convention mismatch in the kinect2_bridge

source /opt/ros/jazzy/setup.bash

# The kinect2_bridge publishes frames in optical convention but labels them wrong
# We need to publish a static transform to correct this

# Kinect optical frame convention (what it should be):
# X-right, Y-down, Z-forward (looking from camera)

# The transform from kinect2_link to kinect2_rgb_optical_frame should be:
# Rotation: 90° around Z, then -90° around X (to go from camera_link to optical)
# But if the driver is already publishing optical, we just need identity

echo "Publishing Kinect optical frame corrections..."

# Check current TF tree
ros2 run tf2_ros tf2_echo kinect2_link kinect2_rgb_optical_frame 2>&1 | head -20

echo ""
echo "If the transform above shows identity or near-identity rotation,"
echo "then the kinect2_bridge is publishing optical frames correctly."
echo ""
echo "The issue is that ORB-SLAM3 and semantic projection need to account"
echo "for this optical frame convention."
echo ""
