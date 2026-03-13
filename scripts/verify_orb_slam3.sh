#!/bin/bash
# Verify ORB-SLAM3 installation and ROS2 integration

set -e

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ORB_SLAM3_DIR="$HOME/ORB_SLAM3"
ROS2_WS="$WORKSPACE_ROOT/ros2_ws"

echo "=== ORB-SLAM3 Verification ==="

# Check ORB-SLAM3 build
echo ""
echo "[1/5] Checking ORB-SLAM3 build..."
if [ -f "$ORB_SLAM3_DIR/lib/libORB_SLAM3.so" ]; then
    echo "✓ ORB-SLAM3 library found"
else
    echo "✗ ORB-SLAM3 not built. Run: scripts/setup_orb_slam3.sh"
    exit 1
fi

# Check vocabulary file
echo ""
echo "[2/5] Checking vocabulary file..."
if [ -f "$ORB_SLAM3_DIR/Vocabulary/ORBvoc.txt" ]; then
    echo "✓ ORB vocabulary found"
else
    echo "✗ ORB vocabulary missing"
    exit 1
fi

# Check config file
echo ""
echo "[3/5] Checking configuration..."
CONFIG_FILE="$WORKSPACE_ROOT/orb_slam3_configs/Kinect2_RGBD_IMU.yaml"
if [ -f "$CONFIG_FILE" ]; then
    echo "✓ Config file exists: $CONFIG_FILE"
    
    # Check if calibration has been applied
    if grep -q "TODO: Replace" "$CONFIG_FILE"; then
        echo "⚠ Config contains placeholder values"
        echo "  Run: python3 scripts/kalibr_to_orb_slam3.py"
    else
        echo "✓ Config appears calibrated"
    fi
else
    echo "✗ Config file missing"
    exit 1
fi

# Check ROS2 package
echo ""
echo "[4/5] Checking ROS2 package..."
if [ -d "$ROS2_WS/install/orb_slam3_ros2" ]; then
    echo "✓ orb_slam3_ros2 package built"
else
    echo "✗ ROS2 package not built"
    echo "  Run: cd $ROS2_WS && colcon build --packages-select orb_slam3_ros2"
    exit 1
fi

# Check launch file
echo ""
echo "[5/5] Checking launch file..."
LAUNCH_FILE="$ROS2_WS/src/kinect2_slam/launch/orb_slam3.launch.py"
if [ -f "$LAUNCH_FILE" ]; then
    echo "✓ Launch file exists"
else
    echo "✗ Launch file missing"
    exit 1
fi

echo ""
echo "=== Verification Complete ==="
echo ""
echo "To run ORB-SLAM3:"
echo "  Terminal 1: ros2 launch kinect2_slam kinect2_publisher.launch.py"
echo "  Terminal 2: ros2 run bluelily_bridge bluelily_imu_node"
echo "  Terminal 3: ros2 launch kinect2_slam orb_slam3.launch.py"
echo ""
echo "To verify tracking:"
echo "  ros2 topic hz /orb_slam3/pose"
echo "  ros2 topic echo /orb_slam3/pose --once"
