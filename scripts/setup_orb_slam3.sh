#!/bin/bash
# Phase 2: ORB-SLAM3 RGB-D+IMU Setup Script
# Builds ORB-SLAM3 and ROS2 wrapper for Kinect v2 + BlueLily IMU

set -e

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ORB_SLAM3_DIR="$HOME/ORB_SLAM3"
ROS2_WS="$WORKSPACE_ROOT/ros2_ws"

echo "=== Phase 2: ORB-SLAM3 Setup ==="
echo "Workspace: $WORKSPACE_ROOT"
echo "ORB-SLAM3 will be installed to: $ORB_SLAM3_DIR"

# Step 1: Verify dependencies
echo ""
echo "[Step 1/5] Checking dependencies..."
dpkg -l | grep -q libglew-dev || { echo "Missing libglew-dev"; exit 1; }
dpkg -l | grep -q libboost-all-dev || { echo "Missing libboost-all-dev"; exit 1; }
dpkg -l | grep -q libeigen3-dev || { echo "Missing libeigen3-dev"; exit 1; }
dpkg -l | grep -q libopencv-dev || { echo "Missing libopencv-dev"; exit 1; }
echo "✓ All dependencies installed"

# Step 2: Verify Pangolin
echo ""
echo "[Step 2/5] Checking Pangolin..."
if [ -d "/usr/local/lib/cmake/Pangolin" ]; then
    echo "✓ Pangolin already installed"
    export Pangolin_DIR=/usr/local/lib/cmake/Pangolin
else
    echo "✗ Pangolin not found. Install with:"
    echo "  git clone https://github.com/stevenlovegrove/Pangolin ~/Pangolin"
    echo "  cd ~/Pangolin && mkdir build && cd build"
    echo "  cmake .. && make -j4 && sudo make install"
    exit 1
fi

# Step 3: Build ORB-SLAM3
echo ""
echo "[Step 3/5] Building ORB-SLAM3..."
if [ -d "$ORB_SLAM3_DIR" ]; then
    echo "ORB-SLAM3 directory exists. Rebuilding..."
    cd "$ORB_SLAM3_DIR"
else
    echo "Cloning ORB-SLAM3..."
    git clone https://github.com/UZ-SLAMLab/ORB_SLAM3 "$ORB_SLAM3_DIR"
    cd "$ORB_SLAM3_DIR"
    
    # Patch CMakeLists.txt for modern CMake
    echo "Patching CMakeLists.txt for CMake 4.x compatibility..."
    sed -i 's/cmake_minimum_required(VERSION 2.8)/cmake_minimum_required(VERSION 3.5)/' CMakeLists.txt
    sed -i 's/cmake_minimum_required(VERSION 2.8)/cmake_minimum_required(VERSION 3.5)/' Thirdparty/DBoW2/CMakeLists.txt
    sed -i 's/CMAKE_MINIMUM_REQUIRED(VERSION 2.6)/cmake_minimum_required(VERSION 3.5)/' Thirdparty/g2o/CMakeLists.txt
    sed -i 's/cmake_minimum_required(VERSION 2.8.10)/cmake_minimum_required(VERSION 3.5)/' Thirdparty/Sophus/CMakeLists.txt
fi

chmod +x build.sh
echo "Building ORB-SLAM3 (this takes 15-20 minutes)..."
./build.sh

if [ ! -f "$ORB_SLAM3_DIR/lib/libORB_SLAM3.so" ]; then
    echo "✗ ORB-SLAM3 build failed"
    exit 1
fi
echo "✓ ORB-SLAM3 built successfully"

# Step 4: Clone ROS2 wrapper
echo ""
echo "[Step 4/5] Setting up ROS2 wrapper..."
cd "$ROS2_WS/src"
if [ ! -d "orb_slam3_ros2" ]; then
    echo "Cloning ORB-SLAM3 ROS2 wrapper..."
    git clone https://github.com/thien94/orb_slam3_ros_wrapper.git orb_slam3_ros2 || \
    git clone https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker.git orb_slam3_ros2 || \
    echo "⚠ Could not clone ROS2 wrapper. You'll need to create a custom wrapper."
fi

if [ -d "$ROS2_WS/src/orb_slam3_ros2" ]; then
    # Update CMakeLists.txt with ORB_SLAM3 path if it exists
    if [ -f "$ROS2_WS/src/orb_slam3_ros2/CMakeLists.txt" ]; then
        sed -i "s|set(ORB_SLAM3_ROOT_DIR.*|set(ORB_SLAM3_ROOT_DIR \"$ORB_SLAM3_DIR\")|" \
            "$ROS2_WS/src/orb_slam3_ros2/CMakeLists.txt" 2>/dev/null || true
    fi

    # Step 5: Build ROS2 package
    echo ""
    echo "[Step 5/5] Building ROS2 package..."
    cd "$ROS2_WS"
    source /opt/ros/humble/setup.bash
    colcon build --packages-select orb_slam3_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
else
    echo ""
    echo "⚠ ROS2 wrapper not available. You'll need to create a custom node."
    echo "  See ros2_ws/src/kinect2_slam/launch/orb_slam3.launch.py for integration example"
fi

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ ORB-SLAM3 ROS2 setup complete!"
    echo ""
    echo "Next steps:"
    echo "1. Update ORB_SLAM3/config/RGB-D-Inertial/Kinect2_RGBD_IMU.yaml with your calibration"
    echo "2. Run: ros2 launch kinect2_slam orb_slam3.launch.py"
else
    echo "✗ ROS2 package build failed"
    exit 1
fi
