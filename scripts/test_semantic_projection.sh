#!/bin/bash
# Test semantic projection system

echo "=========================================="
echo "  Semantic Projection Test"
echo "=========================================="
echo ""

# Source ROS
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v conda | tr '\n' ':' | sed 's/:$//')
source /opt/ros/jazzy/setup.bash

echo "Checking required topics..."
echo ""

# Check ORB-SLAM3 pose
if ros2 topic list | grep -q "/orb_slam3/pose"; then
    echo "✓ /orb_slam3/pose found"
    echo "  Rate:"
    timeout 3 ros2 topic hz /orb_slam3/pose 2>/dev/null | head -1 || echo "  (no data yet)"
else
    echo "❌ /orb_slam3/pose NOT found"
fi

echo ""

# Check depth
if ros2 topic list | grep -q "/kinect2/hd/image_depth_rect"; then
    echo "✓ /kinect2/hd/image_depth_rect found"
else
    echo "❌ /kinect2/hd/image_depth_rect NOT found"
fi

echo ""

# Check YOLO results
if ros2 topic list | grep -q "/cv_pipeline/results"; then
    echo "✓ /cv_pipeline/results found"
    echo "  Sample detection:"
    timeout 3 ros2 topic echo /cv_pipeline/results --once 2>/dev/null | head -20 || echo "  (no data yet)"
else
    echo "❌ /cv_pipeline/results NOT found"
    echo "   Start CV Pipeline: ros2 run cv_pipeline cv_pipeline_node"
fi

echo ""

# Check semantic markers
if ros2 topic list | grep -q "/semantic/markers"; then
    echo "✓ /semantic/markers found"
    echo "  Rate:"
    timeout 3 ros2 topic hz /semantic/markers 2>/dev/null | head -1 || echo "  (no data yet)"
else
    echo "❌ /semantic/markers NOT found"
    echo "   Start semantic projection: ./scripts/run_phase4.sh"
fi

echo ""

# Check world state
if ros2 topic list | grep -q "/semantic/world_state"; then
    echo "✓ /semantic/world_state found"
else
    echo "❌ /semantic/world_state NOT found"
fi

echo ""

# Check world state file
if [ -f /tmp/world_state.json ]; then
    echo "✓ /tmp/world_state.json exists"
    echo "  Objects tracked:"
    cat /tmp/world_state.json | python3 -c "import sys, json; data=json.load(sys.stdin); print(f'  {len(data)} objects')"
    echo ""
    echo "  Sample entries:"
    cat /tmp/world_state.json | python3 -m json.tool | head -20
else
    echo "❌ /tmp/world_state.json NOT found"
fi

echo ""
echo "=========================================="
echo "  TF2 Frames"
echo "=========================================="
echo ""
ros2 run tf2_ros tf2_echo map kinect2_rgb_optical_frame 2>&1 | head -15 || echo "TF not available"

echo ""
echo "=========================================="
