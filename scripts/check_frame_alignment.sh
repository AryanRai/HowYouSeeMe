#!/bin/bash
# Check coordinate frame alignment between Kinect, ORB-SLAM3, and World

echo "========================================="
echo "  Frame Alignment Diagnostic"
echo "========================================="
echo ""

# Source ROS
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

echo "Checking TF2 frames..."
echo ""

# Check if frames exist
echo "1. Available TF2 frames:"
timeout 3 ros2 run tf2_ros tf2_echo map camera_link 2>/dev/null
if [ $? -eq 0 ]; then
    echo "   ✓ map → camera_link transform exists"
else
    echo "   ✗ map → camera_link transform NOT found"
fi

echo ""
echo "2. Kinect optical frame:"
timeout 3 ros2 topic echo /kinect2/hd/camera_info --once 2>/dev/null | grep frame_id
if [ $? -eq 0 ]; then
    echo "   ✓ Kinect publishing camera_info"
else
    echo "   ✗ Kinect NOT publishing"
fi

echo ""
echo "3. ORB-SLAM3 pose:"
timeout 3 ros2 topic echo /orb_slam3/pose --once 2>/dev/null | head -n 5
if [ $? -eq 0 ]; then
    echo "   ✓ ORB-SLAM3 publishing pose"
else
    echo "   ✗ ORB-SLAM3 NOT publishing"
fi

echo ""
echo "4. Semantic markers:"
timeout 3 ros2 topic echo /semantic/markers --once 2>/dev/null | grep "ns:"
if [ $? -eq 0 ]; then
    echo "   ✓ Semantic projection publishing markers"
else
    echo "   ✗ Semantic projection NOT publishing"
fi

echo ""
echo "========================================="
echo "  Frame Convention Reference"
echo "========================================="
echo ""
echo "Standard ROS Optical Frame (OpenCV):"
echo "  X-axis: RIGHT (red arrow)"
echo "  Y-axis: DOWN (green arrow)"
echo "  Z-axis: FORWARD (blue arrow)"
echo ""
echo "Your Kinect Frame (NON-STANDARD):"
echo "  X-axis: LEFT (red arrow) ← FLIPPED!"
echo "  Y-axis: DOWN (green arrow)"
echo "  Z-axis: FORWARD (blue arrow)"
echo ""
echo "ROS World Frame (map):"
echo "  X-axis: FORWARD (red arrow)"
echo "  Y-axis: LEFT (green arrow)"
echo "  Z-axis: UP (blue arrow)"
echo ""
echo "========================================="
echo "  Current Configuration"
echo "========================================="
echo ""
echo "flip_x_axis parameter: true"
echo "  → Negates X coordinate in pixel_to_3d()"
echo "  → Corrects for Kinect's left-pointing X-axis"
echo ""
echo "ORB-SLAM3 transformation:"
echo "  [X_ros, Y_ros, Z_ros] = [Z_orb, -X_orb, -Y_orb]"
echo "  → Converts OpenCV frame to ROS camera_link"
echo ""
echo "========================================="
echo ""
echo "To test alignment:"
echo "  1. Point camera at a known object"
echo "  2. Check RViz marker position"
echo "  3. Verify marker appears at correct location"
echo "  4. If still misaligned, check logs:"
echo "     tail -f /tmp/semantic.log"
echo ""
