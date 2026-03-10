#!/bin/bash
# Basic Robot Head Test - BlueLily + Kinect Only

echo "========================================="
echo "  Robot Head - Basic Test"
echo "========================================="
echo ""

# Source ROS2
source ros2_ws/install/setup.bash 2>/dev/null

# Kill existing
pkill -f bluelily_imu
pkill -f kinect2_bridge
sleep 1

echo "1. Starting BlueLily IMU..."
sudo chmod 666 /dev/ttyACM0 2>/dev/null
ros2 run bluelily_bridge bluelily_imu_node --ros-args \
    -p port:=/dev/ttyACM0 \
    -p baud_rate:=115200 &
BLUELILY_PID=$!

sleep 2

echo "2. Checking IMU data..."
timeout 3 ros2 topic echo /imu/data --once > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "✅ BlueLily IMU working!"
    echo ""
    echo "IMU Rate:"
    timeout 5 ros2 topic hz /imu/data 2>&1 | grep "average rate"
else
    echo "❌ No IMU data"
fi

echo ""
echo "3. Starting Kinect..."
ros2 launch kinect2_bridge kinect2_bridge.launch.py > /tmp/kinect_test.log 2>&1 &
KINECT_PID=$!

sleep 5

echo "4. Checking Kinect topics..."
ros2 topic list | grep kinect2

echo ""
echo "5. System Status:"
echo "   BlueLily PID: $BLUELILY_PID"
echo "   Kinect PID: $KINECT_PID"
echo ""
echo "Press Ctrl+C to stop"

trap "kill $BLUELILY_PID $KINECT_PID 2>/dev/null; exit 0" INT
wait
