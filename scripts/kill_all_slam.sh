#!/bin/bash
# Kill all SLAM-related processes

echo "=========================================="
echo "  Stopping All SLAM Components"
echo "=========================================="
echo ""

# Function to kill process and report
kill_process() {
    local name=$1
    local pattern=$2
    
    echo -n "Stopping $name... "
    if pkill -f "$pattern" 2>/dev/null; then
        sleep 1
        echo "✓"
    else
        echo "(not running)"
    fi
}

# Kill all components
kill_process "ORB-SLAM3" "orb_slam3_node"
kill_process "TSDF Integrator" "tsdf_integrator"
kill_process "Kinect Bridge" "kinect2_bridge_node"
kill_process "Semantic Projection" "semantic_projection"
kill_process "CV Pipeline (C++)" "cv_pipeline_node"
kill_process "CV Pipeline (Python)" "sam2_server_v2.py"
kill_process "BlueLily IMU" "bluelily_imu_node"
kill_process "RViz2" "rviz2"
kill_process "TF Static Publishers" "static_transform_publisher"
kill_process "Depth Processing" "point_cloud_xyzrgb_node"

echo ""
echo "Checking for remaining processes..."
REMAINING=$(ps aux | grep -E "(orb_slam3|tsdf_integrator|kinect2_bridge|semantic_projection|cv_pipeline|sam2_server)" | grep -v grep | wc -l)

if [ "$REMAINING" -gt 0 ]; then
    echo "⚠️  $REMAINING processes still running:"
    ps aux | grep -E "(orb_slam3|tsdf_integrator|kinect2_bridge|semantic_projection|cv_pipeline|sam2_server)" | grep -v grep | awk '{print "   PID " $2 ": " $11}'
    echo ""
    echo "Force kill? (y/n)"
    read -r response
    if [ "$response" = "y" ]; then
        pkill -9 -f "orb_slam3_node"
        pkill -9 -f "tsdf_integrator"
        pkill -9 -f "kinect2_bridge_node"
        pkill -9 -f "semantic_projection"
        pkill -9 -f "cv_pipeline_node"
        pkill -9 -f "sam2_server_v2.py"
        echo "✓ Force killed all processes"
    fi
else
    echo "✓ All processes stopped cleanly"
fi

echo ""
echo "=========================================="
