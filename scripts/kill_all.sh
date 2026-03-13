#!/bin/bash

# Robust kill script for ALL Kinect, ROS2, SLAM, and CV pipeline processes

echo "=========================================="
echo "  Stopping All Processes"
echo "=========================================="

# Function to kill process by name with retry
kill_process() {
    local name=$1
    local display_name=${2:-$name}
    
    if pgrep -f "$name" > /dev/null 2>&1; then
        echo "  Stopping $display_name..."
        pkill -TERM -f "$name" 2>/dev/null
        sleep 0.5
        
        # Check if still running, force kill
        if pgrep -f "$name" > /dev/null 2>&1; then
            pkill -KILL -f "$name" 2>/dev/null
            sleep 0.3
        fi
        
        # Verify stopped
        if ! pgrep -f "$name" > /dev/null 2>&1; then
            echo "    ✅ Stopped"
        else
            echo "    ⚠️  Still running"
        fi
    fi
}

echo ""
echo "1. Kinect Processes"
echo "-------------------"
kill_process "kinect2_bridge" "Kinect Bridge"
kill_process "kinect2_simple_publisher" "Kinect Publisher"
kill_process "point_cloud_xyzrgb" "Point Cloud"

echo ""
echo "2. SLAM Processes"
echo "-----------------"
kill_process "rtabmap" "RTABMap"
kill_process "rgbd_odometry" "RGBD Odometry"
kill_process "icp_odometry" "ICP Odometry"

echo ""
echo "3. CV Pipeline"
echo "--------------"
kill_process "cv_pipeline_node" "CV Pipeline Node"
kill_process "sam2_server" "SAM2 Server"
kill_process "sam2_worker" "SAM2 Worker"

echo ""
echo "4. Visualization"
echo "----------------"
kill_process "rviz2" "RViz2"
kill_process "image_view" "Image View"
kill_process "image_saver" "Image Saver"

echo ""
echo "5. ROS2 Launch"
echo "--------------"
kill_process "ros2 launch" "ROS2 Launch"

echo ""
echo "6. Python ROS2 Nodes"
echo "--------------------"
pkill -f "python.*ros2" 2>/dev/null && echo "  ✅ Python nodes stopped" || echo "  (none running)"

# Wait for cleanup
sleep 1

# Final verification
echo ""
echo "=========================================="
echo "  Verification"
echo "=========================================="

KINECT=$(pgrep -f "kinect2" 2>/dev/null | wc -l)
SLAM=$(pgrep -f "rtabmap" 2>/dev/null | wc -l)
CV=$(pgrep -f "sam2|cv_pipeline" 2>/dev/null | wc -l)
VIZ=$(pgrep -f "rviz2" 2>/dev/null | wc -l)

echo ""
echo "Remaining processes:"
echo "  Kinect: $KINECT"
echo "  SLAM:   $SLAM"
echo "  CV:     $CV"
echo "  Viz:    $VIZ"

TOTAL=$((KINECT + SLAM + CV + VIZ))

if [ $TOTAL -eq 0 ]; then
    echo ""
    echo "✅ All processes stopped successfully!"
else
    echo ""
    echo "⚠️  $TOTAL process(es) still running:"
    echo ""
    ps aux | grep -E "kinect2|rtabmap|sam2|cv_pipeline|rviz2" | grep -v grep | grep -v "kill_all"
    
    echo ""
    echo "To force kill all:"
    echo "  pkill -9 -f 'kinect2|rtabmap|sam2|cv_pipeline|rviz2'"
fi

echo ""
echo "=========================================="
echo "  Cleanup Complete!"
echo "=========================================="
echo ""
