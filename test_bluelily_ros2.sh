#!/bin/bash
# Test BlueLily ROS2 IMU Bridge

echo "========================================="
echo "  BlueLily ROS2 IMU Bridge Test"
echo "========================================="
echo ""

# Check if BlueLily is connected
if [ ! -e "/dev/ttyACM0" ]; then
    echo "❌ BlueLily not found on /dev/ttyACM0"
    echo ""
    echo "Available serial devices:"
    ls -la /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "  None found"
    echo ""
    exit 1
fi

echo "✅ BlueLily found on /dev/ttyACM0"
echo ""

# Fix permissions if needed
if [ ! -w "/dev/ttyACM0" ]; then
    echo "⚠️  No write permission, fixing..."
    sudo chmod 666 /dev/ttyACM0
    echo "✅ Permissions fixed"
    echo ""
fi

# Source ROS2
echo "Sourcing ROS2 workspace..."
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
echo "✅ ROS2 sourced"
echo ""

# Launch BlueLily bridge
echo "Launching BlueLily IMU bridge..."
echo "Press Ctrl+C to stop"
echo ""
echo "========================================="
echo ""

ros2 run bluelily_bridge bluelily_imu_node --ros-args \
    -p port:=/dev/ttyACM0 \
    -p baud_rate:=115200 \
    -p frame_id:=bluelily_imu &

BRIDGE_PID=$!

# Wait for bridge to start
sleep 2

# Monitor IMU data
echo ""
echo "Monitoring IMU data (5 seconds)..."
echo ""

timeout 5 ros2 topic echo /imu/data --once 2>/dev/null

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ IMU data received successfully!"
    echo ""
    echo "Available topics:"
    ros2 topic list | grep -E "(imu|bluelily)"
    echo ""
    echo "To monitor continuously:"
    echo "  ros2 topic echo /imu/data"
    echo "  ros2 topic hz /imu/data"
else
    echo ""
    echo "⚠️  No IMU data received"
    echo "Check the bridge output above for errors"
fi

# Cleanup
echo ""
echo "Stopping bridge..."
kill $BRIDGE_PID 2>/dev/null
wait $BRIDGE_PID 2>/dev/null

echo "✅ Test complete"
