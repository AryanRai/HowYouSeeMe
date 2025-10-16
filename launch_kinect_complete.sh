#!/bin/bash
# Complete Kinect v2 launch script with RViz and monitoring

# Set library path for libfreenect2
export LD_LIBRARY_PATH=/home/aryan/Documents/GitHub/HowYouSeeMe/libfreenect2/freenect2/lib:$LD_LIBRARY_PATH

# Source ROS2
source /opt/ros/jazzy/setup.bash
source /home/aryan/Documents/GitHub/HowYouSeeMe/ros2_ws/install/setup.bash

echo "=========================================="
echo "Starting Kinect v2 ROS2 System"
echo "=========================================="
echo ""

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "Shutting down..."
    kill $TF1_PID $TF2_PID $TF3_PID $KINECT_PID $RVIZ_PID $MONITOR_PID 2>/dev/null
    wait $TF1_PID $TF2_PID $TF3_PID $KINECT_PID $RVIZ_PID $MONITOR_PID 2>/dev/null
    echo "All processes stopped"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Start static transform publisher (map -> kinect2_link -> kinect2_rgb_optical_frame)
echo "1. Starting TF publishers..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map kinect2_link &
TF1_PID=$!
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 kinect2_link kinect2_rgb_optical_frame &
TF2_PID=$!
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 kinect2_link kinect2_ir_optical_frame &
TF3_PID=$!
echo "   TF PIDs: $TF1_PID, $TF2_PID, $TF3_PID"
sleep 1

# Start Kinect publisher in background
echo "2. Starting Kinect v2 publisher..."
ros2 run kinect2_simple_publisher kinect2_simple_publisher_node &
KINECT_PID=$!
echo "   PID: $KINECT_PID"

# Wait for node to initialize
sleep 3

# Check if topics are being published
echo ""
echo "3. Checking ROS2 topics..."
ros2 topic list | grep kinect2
echo ""

# Show topic info
echo "4. Topic information:"
echo "   - /kinect2/sd/image_color (RGB image)"
echo "   - /kinect2/sd/image_depth (Depth image)"
echo "   - /kinect2/sd/image_ir (IR image)"
echo "   - /kinect2/sd/camera_info (Camera info)"
echo ""

# Start topic monitor in background
echo "5. Starting topic rate monitor..."
(
    sleep 2
    while true; do
        clear
        echo "=========================================="
        echo "Kinect v2 ROS2 Topic Monitor"
        echo "=========================================="
        echo ""
        echo "RGB Topic Rate:"
        timeout 3 ros2 topic hz /kinect2/sd/image_color 2>/dev/null || echo "  Waiting for data..."
        echo ""
        echo "Depth Topic Rate:"
        timeout 3 ros2 topic hz /kinect2/sd/image_depth 2>/dev/null || echo "  Waiting for data..."
        echo ""
        echo "Available topics:"
        ros2 topic list | grep kinect2
        echo ""
        echo "Press Ctrl+C to stop all processes"
        sleep 5
    done
) &
MONITOR_PID=$!

# Start RViz2 with config
echo "6. Starting RViz2..."
sleep 2
rviz2 -d /home/aryan/Documents/GitHub/HowYouSeeMe/kinect2_rviz_config.rviz &
RVIZ_PID=$!
echo "   PID: $RVIZ_PID"

echo ""
echo "=========================================="
echo "System Running!"
echo "=========================================="
echo "TF Publishers PIDs: $TF1_PID, $TF2_PID, $TF3_PID"
echo "Kinect Publisher PID: $KINECT_PID"
echo "RViz PID: $RVIZ_PID"
echo "Monitor PID: $MONITOR_PID"
echo ""
echo "RViz Configuration:"
echo "  - Fixed Frame: kinect2_rgb_optical_frame"
echo "  - RGB Image: /kinect2/sd/image_color"
echo "  - Depth Image: /kinect2/sd/image_depth"
echo "  - IR Image: /kinect2/sd/image_ir (disabled by default)"
echo ""
echo "If images don't show, check:"
echo "  1. Fixed Frame is set to 'kinect2_rgb_optical_frame'"
echo "  2. Image topics are subscribed"
echo ""
echo "Press Ctrl+C to stop all processes"
echo "=========================================="

# Wait for processes
wait $TF1_PID $TF2_PID $TF3_PID $KINECT_PID $RVIZ_PID $MONITOR_PID
