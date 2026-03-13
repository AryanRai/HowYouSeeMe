#!/bin/bash
# Test script for complete SLAM + CV Pipeline + Semantic Projection system

set -e

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE_ROOT"

echo "=========================================="
echo "Complete System Test"
echo "=========================================="
echo ""

# Source ROS2
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

echo "1. Checking if Kinect is connected..."
if ros2 topic list | grep -q "/kinect2"; then
    echo "   ✅ Kinect topics found"
else
    echo "   ❌ Kinect not running! Start with: ros2 launch kinect2_bridge kinect2_bridge.launch"
    exit 1
fi

echo ""
echo "2. Checking ORB-SLAM3 pose..."
timeout 2 ros2 topic echo /orb_slam3/pose --once > /dev/null 2>&1 && \
    echo "   ✅ ORB-SLAM3 publishing poses" || \
    echo "   ⚠️  No ORB-SLAM3 poses yet (may need initialization)"

echo ""
echo "3. Checking TF2 transforms..."
if ros2 run tf2_ros tf2_echo map camera_pose 2>&1 | grep -q "At time"; then
    echo "   ✅ Camera pose TF2 transform active"
else
    echo "   ⚠️  Camera pose TF2 not yet available"
fi

echo ""
echo "4. Checking CV Pipeline..."
if ros2 topic list | grep -q "/cv_pipeline/results"; then
    echo "   ✅ CV Pipeline topics exist"
    
    # Check if results are being published
    timeout 2 ros2 topic echo /cv_pipeline/results --once > /dev/null 2>&1 && \
        echo "   ✅ CV Pipeline publishing results" || \
        echo "   ⚠️  No CV results yet (start detection with scripts/cv_pipeline_menu.sh)"
else
    echo "   ❌ CV Pipeline not running!"
fi

echo ""
echo "5. Checking Semantic Projection..."
if ros2 topic list | grep -q "/semantic/markers"; then
    echo "   ✅ Semantic projection topics exist"
    
    # Check if markers are being published
    timeout 2 ros2 topic echo /semantic/markers --once > /dev/null 2>&1 && \
        echo "   ✅ Semantic markers being published" || \
        echo "   ⚠️  No semantic markers yet (need CV detections + valid pose)"
else
    echo "   ❌ Semantic projection not running!"
fi

echo ""
echo "6. Checking world state..."
if [ -f "/tmp/world_state.json" ]; then
    num_objects=$(jq 'length' /tmp/world_state.json 2>/dev/null || echo "0")
    echo "   ✅ World state file exists: $num_objects objects tracked"
else
    echo "   ⚠️  No world state file yet"
fi

echo ""
echo "7. Listing all TF2 frames..."
ros2 run tf2_ros tf2_monitor --all-frames 2>&1 | head -20

echo ""
echo "=========================================="
echo "Test Complete!"
echo "=========================================="
echo ""
echo "To view in RViz:"
echo "  rviz2 -d rviz_configs/tsdf_rviz.rviz"
echo ""
echo "To start CV detection:"
echo "  ./scripts/cv_pipeline_menu.sh"
echo ""
