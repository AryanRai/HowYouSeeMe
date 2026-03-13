#!/bin/bash
# Quick status check for ORB-SLAM3 build

ORB_SLAM3_DIR="$HOME/ORB_SLAM3"

echo "=== ORB-SLAM3 Build Status ==="
echo ""

# Check third-party libraries
echo "Third-party libraries:"
if [ -f "$ORB_SLAM3_DIR/Thirdparty/DBoW2/lib/libDBoW2.so" ]; then
    echo "  ✓ DBoW2"
else
    echo "  ✗ DBoW2 (not built)"
fi

if [ -f "$ORB_SLAM3_DIR/Thirdparty/g2o/lib/libg2o.so" ]; then
    echo "  ✓ g2o"
else
    echo "  ⏳ g2o (building or not started)"
fi

if [ -f "$ORB_SLAM3_DIR/Thirdparty/Sophus/lib/libSophus.so" ]; then
    echo "  ✓ Sophus"
else
    echo "  ⏳ Sophus (building or not started)"
fi

echo ""
echo "Main library:"
if [ -f "$ORB_SLAM3_DIR/lib/libORB_SLAM3.so" ]; then
    echo "  ✓ ORB_SLAM3"
    ls -lh "$ORB_SLAM3_DIR/lib/libORB_SLAM3.so"
    echo ""
    echo "BUILD COMPLETE! Next: ./scripts/setup_orb_slam3.sh"
else
    echo "  ⏳ ORB_SLAM3 (building or not started)"
    echo ""
    echo "Build in progress... (typically 15-20 minutes)"
fi
