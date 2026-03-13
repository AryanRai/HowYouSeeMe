#!/bin/bash
# Patch ORB-SLAM3 CMakeLists.txt files for modern CMake compatibility

set -e

ORB_SLAM3_DIR="$HOME/ORB_SLAM3"

if [ ! -d "$ORB_SLAM3_DIR" ]; then
    echo "✗ ORB_SLAM3 directory not found: $ORB_SLAM3_DIR"
    exit 1
fi

echo "=== Patching ORB-SLAM3 for CMake 4.x ==="

# Patch DBoW2
echo "Patching Thirdparty/DBoW2..."
sed -i 's/cmake_minimum_required(VERSION 2.8)/cmake_minimum_required(VERSION 3.5)/' \
    "$ORB_SLAM3_DIR/Thirdparty/DBoW2/CMakeLists.txt"

# Patch g2o
echo "Patching Thirdparty/g2o..."
sed -i 's/CMAKE_MINIMUM_REQUIRED(VERSION 2.6)/cmake_minimum_required(VERSION 3.5)/' \
    "$ORB_SLAM3_DIR/Thirdparty/g2o/CMakeLists.txt"

# Patch Sophus
echo "Patching Thirdparty/Sophus..."
sed -i 's/cmake_minimum_required(VERSION 2.8.10)/cmake_minimum_required(VERSION 3.5)/' \
    "$ORB_SLAM3_DIR/Thirdparty/Sophus/CMakeLists.txt"

# Patch main ORB_SLAM3
echo "Patching main ORB_SLAM3..."
sed -i 's/cmake_minimum_required(VERSION 2.8)/cmake_minimum_required(VERSION 3.5)/' \
    "$ORB_SLAM3_DIR/CMakeLists.txt"

echo "✓ All CMakeLists.txt files patched"
echo ""
echo "Now run: cd ~/ORB_SLAM3 && ./build.sh"
