#!/bin/bash
# Fix ORB-SLAM3 C++14 compatibility issues

set -e

ORB_SLAM3_DIR="$HOME/ORB_SLAM3"

echo "=== Fixing ORB-SLAM3 for C++14 ==="

# Update main CMakeLists.txt
echo "Updating C++ standard to 14..."
sed -i 's/set(CMAKE_CXX_STANDARD 11)/set(CMAKE_CXX_STANDARD 14)/' "$ORB_SLAM3_DIR/CMakeLists.txt"

# Also update if it's using C++11 flag directly
sed -i 's/-std=c++11/-std=c++14/g' "$ORB_SLAM3_DIR/CMakeLists.txt"

# Clean and rebuild
echo "Cleaning build directory..."
rm -rf "$ORB_SLAM3_DIR/build"
mkdir -p "$ORB_SLAM3_DIR/build"

echo "Rebuilding with C++14..."
cd "$ORB_SLAM3_DIR"
./build.sh

if [ -f "$ORB_SLAM3_DIR/lib/libORB_SLAM3.so" ]; then
    echo "✓ Build successful!"
else
    echo "✗ Build failed"
    exit 1
fi
