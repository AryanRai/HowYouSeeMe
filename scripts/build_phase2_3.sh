#!/bin/bash
# Build Phase 2 & 3: ORB-SLAM3 + TSDF Integration

set -e

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS2_WS="$WORKSPACE_ROOT/ros2_ws"

echo "=== Building Phase 2 & 3 ==="

# Step 1: Install Open3D
echo ""
echo "[1/3] Installing Open3D..."
pip install open3d --break-system-packages || echo "⚠ Open3D install may have failed, continuing..."

# Step 2: Source ROS
echo ""
echo "[2/3] Sourcing ROS..."
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo "✓ Using ROS Jazzy"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✓ Using ROS Humble"
else
    echo "✗ ROS not found in /opt/ros/"
    exit 1
fi

# Step 3: Build package
echo ""
echo "[3/3] Building kinect2_slam package..."
cd "$ROS2_WS"
colcon build --packages-select kinect2_slam --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ Build complete!"
    echo ""
    echo "To run:"
    echo "  source $ROS2_WS/install/setup.bash"
    echo "  ros2 launch kinect2_slam orb_slam3.launch.py"
else
    echo "✗ Build failed"
    exit 1
fi
