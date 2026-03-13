#!/bin/bash
# Check if all Phase 2-3 dependencies are ready

echo "=== Checking Phase 2-3 Dependencies ==="
echo ""

ERRORS=0

# Check ORB-SLAM3
echo -n "ORB-SLAM3 library... "
if [ -f "$HOME/ORB_SLAM3/lib/libORB_SLAM3.so" ]; then
    echo "✓"
else
    echo "✗ Missing"
    ERRORS=$((ERRORS + 1))
fi

echo -n "ORB-SLAM3 vocabulary... "
if [ -f "$HOME/ORB_SLAM3/Vocabulary/ORBvoc.txt" ]; then
    echo "✓"
else
    echo "✗ Missing"
    ERRORS=$((ERRORS + 1))
fi

# Check config file
echo -n "Kinect2 config file... "
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"
if [ -f "$WORKSPACE_ROOT/orb_slam3_configs/Kinect2_RGBD_IMU.yaml" ]; then
    echo "✓"
else
    echo "✗ Missing"
    ERRORS=$((ERRORS + 1))
fi

# Check ROS packages
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_ROOT/ros2_ws/install/setup.bash" 2>/dev/null

echo -n "kinect2_bridge package... "
if ros2 pkg list 2>/dev/null | grep -q "^kinect2_bridge$"; then
    echo "✓"
else
    echo "✗ Missing"
    ERRORS=$((ERRORS + 1))
fi

echo -n "bluelily_bridge package... "
if ros2 pkg list 2>/dev/null | grep -q "^bluelily_bridge$"; then
    echo "✓"
else
    echo "✗ Missing"
    ERRORS=$((ERRORS + 1))
fi

echo -n "kinect2_slam package... "
if ros2 pkg list 2>/dev/null | grep -q "^kinect2_slam$"; then
    echo "✓"
else
    echo "✗ Missing"
    ERRORS=$((ERRORS + 1))
fi

# Check Python dependencies
echo -n "Open3D Python package... "
if python3 -c "import open3d" 2>/dev/null; then
    echo "✓"
else
    echo "✗ Missing (run: pip install open3d)"
    ERRORS=$((ERRORS + 1))
fi

echo ""
if [ $ERRORS -eq 0 ]; then
    echo "✓ All dependencies ready!"
    echo ""
    echo "Run: ./scripts/run_phase2_3.sh"
    exit 0
else
    echo "✗ $ERRORS dependencies missing"
    exit 1
fi
