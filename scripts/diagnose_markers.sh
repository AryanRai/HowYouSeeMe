#!/bin/bash
# Diagnose why markers aren't showing in RViz

echo "=========================================="
echo "  Marker Visibility Diagnostic"
echo "=========================================="
echo ""

source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

echo "[1] Checking marker topic..."
if ros2 topic list | grep -q "/semantic/markers"; then
    echo "    ✓ Topic exists"
    
    echo ""
    echo "    Checking message rate..."
    timeout 3 ros2 topic hz /semantic/markers 2>&1 | head -5
    
    echo ""
    echo "    Checking latest marker message..."
    timeout 2 ros2 topic echo /semantic/markers --once 2>&1 | head -30
else
    echo "    ✗ Topic doesn't exist - semantic_projection node not running"
    exit 1
fi

echo ""
echo "[2] Checking TF2 frames..."
echo "    Available frames:"
ros2 run tf2_ros tf2_monitor --all-frames 2>&1 | grep "Frame:" | head -10

echo ""
echo "    Checking map → camera_pose transform..."
timeout 2 ros2 run tf2_ros tf2_echo map camera_pose 2>&1 | head -10 || echo "    ✗ Transform not available"

echo ""
echo "    Checking map → camera_link transform..."
timeout 2 ros2 run tf2_ros tf2_echo map camera_link 2>&1 | head -10 || echo "    ✗ Transform not available"

echo ""
echo "[3] Checking RViz configuration..."
if [ -f "rviz_configs/tsdf_rviz.rviz" ]; then
    echo "    ✓ RViz config exists"
    
    echo ""
    echo "    Checking Fixed Frame..."
    grep "Fixed Frame:" rviz_configs/tsdf_rviz.rviz
    
    echo ""
    echo "    Checking MarkerArray config..."
    grep -A 10 "MarkerArray" rviz_configs/tsdf_rviz.rviz | head -15
else
    echo "    ✗ RViz config not found"
fi

echo ""
echo "[4] Checking semantic_projection node logs..."
if [ -f "/tmp/semantic.log" ]; then
    echo "    Last 20 lines of semantic.log:"
    tail -20 /tmp/semantic.log
else
    echo "    ✗ No log file found"
fi

echo ""
echo "[5] Checking CV Pipeline results format..."
echo "    Waiting for one CV result message..."
timeout 3 ros2 topic echo /cv_pipeline/results --once 2>&1 | python3 -m json.tool 2>/dev/null | head -40

echo ""
echo "=========================================="
echo "  Common Issues & Solutions"
echo "=========================================="
echo ""
echo "Issue: Markers not visible in RViz"
echo "Solutions:"
echo "  1. Check Fixed Frame is 'map' in RViz"
echo "  2. Enable 'Semantic Labels' display in RViz"
echo "  3. Check MarkerArray namespaces are enabled"
echo "  4. Verify ORB-SLAM3 has initialized (camera moving)"
echo "  5. Ensure YOLO is detecting objects"
echo ""
echo "Issue: TF transforms missing"
echo "Solutions:"
echo "  1. Rebuild kinect2_slam: colcon build --packages-select kinect2_slam"
echo "  2. Check ORB-SLAM3 is tracking (not 'LOST')"
echo "  3. Verify camera_pose or camera_link frame exists"
echo ""
echo "Issue: No detections"
echo "Solutions:"
echo "  1. Start YOLO from CV Pipeline menu"
echo "  2. Check confidence threshold (default 0.4)"
echo "  3. Verify objects are in camera view"
echo ""
