#!/bin/bash
# Kill all Kinect and ROS2 related processes

echo "=========================================="
echo "Stopping all Kinect and ROS2 processes"
echo "=========================================="
echo ""

# Kill Kinect publisher
echo "1. Stopping Kinect publisher..."
pkill -f kinect2_simple_publisher_node
sleep 1

# Kill RViz
echo "2. Stopping RViz2..."
pkill -f rviz2
sleep 1

# Kill any kinect2_bridge processes
echo "3. Stopping kinect2_bridge..."
pkill -f kinect2_bridge_node
sleep 1

# Kill any ros2 launch processes
echo "4. Stopping ros2 launch processes..."
pkill -f "ros2 launch"
sleep 1

# Kill any remaining ROS2 nodes
echo "5. Stopping remaining ROS2 nodes..."
pkill -f "ros2 run"
sleep 1

# Check if any processes are still running
echo ""
echo "Checking for remaining processes..."
REMAINING=$(ps aux | grep -E "(kinect|rviz2|ros2)" | grep -v grep | grep -v kill_kinect)

if [ -z "$REMAINING" ]; then
    echo "✓ All processes stopped successfully"
else
    echo "⚠ Some processes may still be running:"
    echo "$REMAINING"
    echo ""
    echo "Force killing remaining processes..."
    pkill -9 -f kinect2_simple_publisher_node
    pkill -9 -f rviz2
    pkill -9 -f kinect2_bridge_node
    sleep 1
    echo "✓ Force kill complete"
fi

echo ""
echo "=========================================="
echo "Cleanup complete!"
echo "=========================================="
