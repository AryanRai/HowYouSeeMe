#!/bin/bash
# Monitor ORB-SLAM3 build progress

ORB_SLAM3_DIR="$HOME/ORB_SLAM3"

echo "Monitoring ORB-SLAM3 build progress..."
echo "This typically takes 15-20 minutes"
echo ""

while true; do
    clear
    echo "=== ORB-SLAM3 Build Status ==="
    echo ""
    
    # Check if build is complete
    if [ -f "$ORB_SLAM3_DIR/lib/libORB_SLAM3.so" ]; then
        echo "✓ BUILD COMPLETE!"
        echo ""
        echo "Library: $ORB_SLAM3_DIR/lib/libORB_SLAM3.so"
        ls -lh "$ORB_SLAM3_DIR/lib/libORB_SLAM3.so"
        echo ""
        echo "Next step: ./scripts/setup_orb_slam3.sh (to build ROS2 wrapper)"
        break
    fi
    
    # Show what's been built so far
    echo "Third-party libraries:"
    [ -f "$ORB_SLAM3_DIR/Thirdparty/DBoW2/lib/libDBoW2.so" ] && echo "  ✓ DBoW2" || echo "  ⏳ DBoW2"
    [ -f "$ORB_SLAM3_DIR/Thirdparty/g2o/lib/libg2o.so" ] && echo "  ✓ g2o" || echo "  ⏳ g2o"
    [ -f "$ORB_SLAM3_DIR/Thirdparty/Sophus/lib/libSophus.so" ] && echo "  ✓ Sophus" || echo "  ⏳ Sophus"
    
    echo ""
    echo "Main library:"
    [ -f "$ORB_SLAM3_DIR/lib/libORB_SLAM3.so" ] && echo "  ✓ ORB_SLAM3" || echo "  ⏳ ORB_SLAM3 (building...)"
    
    echo ""
    echo "Press Ctrl+C to stop monitoring"
    echo "Checking again in 10 seconds..."
    
    sleep 10
done
