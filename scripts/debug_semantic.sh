#!/bin/bash
# Debug semantic projection pipeline

echo "=========================================="
echo "  Semantic Projection Debug"
echo "=========================================="
echo ""

echo "[1] Checking if semantic_projection node is running..."
if ps aux | grep -v grep | grep -q semantic_projection; then
    echo "    ✓ Running"
    ps aux | grep semantic_projection | grep -v grep | head -1
else
    echo "    ✗ Not running"
fi

echo ""
echo "[2] Checking if ORB-SLAM3 is publishing poses..."
timeout 2 bash -c 'ros2 topic hz /orb_slam3/pose 2>&1 | head -3' || echo "    ✗ No poses"

echo ""
echo "[3] Checking if YOLO is publishing results..."
timeout 2 bash -c 'ros2 topic hz /cv_pipeline/results 2>&1 | head -3' || echo "    ✗ No YOLO results"

echo ""
echo "[4] Checking if depth is publishing..."
timeout 2 bash -c 'ros2 topic hz /kinect2/hd/image_depth_rect 2>&1 | head -3' || echo "    ✗ No depth"

echo ""
echo "[5] Checking if semantic markers are being published..."
timeout 2 bash -c 'ros2 topic hz /semantic/markers 2>&1 | head -3' || echo "    ✗ No markers"

echo ""
echo "[6] Checking world state file..."
if [ -f /tmp/world_state.json ]; then
    SIZE=$(stat -f%z /tmp/world_state.json 2>/dev/null || stat -c%s /tmp/world_state.json 2>/dev/null)
    echo "    File exists: $SIZE bytes"
    if [ "$SIZE" -gt 2 ]; then
        echo "    Content:"
        cat /tmp/world_state.json | python3 -m json.tool 2>/dev/null | head -20
    else
        echo "    Empty (no objects detected yet)"
    fi
else
    echo "    ✗ File doesn't exist"
fi

echo ""
echo "[7] Checking CV Pipeline visualization..."
timeout 2 bash -c 'ros2 topic hz /cv_pipeline/visualization 2>&1 | head -3' || echo "    ✗ No visualization"

echo ""
echo "=========================================="
echo "  Summary"
echo "=========================================="
echo ""
echo "For semantic projection to work, you need:"
echo "  1. ORB-SLAM3 publishing poses (/orb_slam3/pose)"
echo "  2. YOLO publishing detections (/cv_pipeline/results)"
echo "  3. Kinect publishing depth (/kinect2/hd/image_depth_rect)"
echo ""
echo "If YOLO results are missing, start YOLO from CV Pipeline menu:"
echo "  Select: 3) YOLO11 → 1) Detection → Stream mode"
echo ""
