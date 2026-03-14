#!/bin/bash
# Test the memory system components

echo "=== Testing HowYouSeeMe Memory System ==="
echo ""

# Check directories
echo "1. Checking directories..."
if [ -d "/tmp/stm" ]; then
    echo "   ✓ /tmp/stm exists"
else
    echo "   ✗ /tmp/stm missing (will be created on first run)"
fi

if [ -d ~/howyouseeme_persistent ]; then
    echo "   ✓ ~/howyouseeme_persistent exists"
else
    echo "   ✗ ~/howyouseeme_persistent missing"
fi

echo ""
echo "2. Checking ROS2 nodes..."
source ~/Documents/GitHub/HowYouSeeMe/ros2_ws/install/setup.bash 2>/dev/null

# Check if nodes are registered
if ros2 pkg executables kinect2_slam | grep -q "event_checkpointer"; then
    echo "   ✓ event_checkpointer registered"
else
    echo "   ✗ event_checkpointer not found"
fi

if ros2 pkg executables kinect2_slam | grep -q "async_analyser"; then
    echo "   ✓ async_analyser registered"
else
    echo "   ✗ async_analyser not found"
fi

if ros2 pkg executables kinect2_slam | grep -q "world_synthesiser"; then
    echo "   ✓ world_synthesiser registered"
else
    echo "   ✗ world_synthesiser not found"
fi

if ros2 pkg executables kinect2_slam | grep -q "named_memory"; then
    echo "   ✓ named_memory registered"
else
    echo "   ✗ named_memory not found"
fi

echo ""
echo "3. Checking Python dependencies..."
python3 -c "import numpy" 2>/dev/null && echo "   ✓ numpy" || echo "   ✗ numpy missing"
python3 -c "import cv2" 2>/dev/null && echo "   ✓ opencv (cv2)" || echo "   ✗ opencv missing"
python3 -c "import rclpy" 2>/dev/null && echo "   ✓ rclpy" || echo "   ✗ rclpy missing"

echo ""
echo "4. Checking MCP server..."
if [ -f ~/Documents/GitHub/HowYouSeeMe/ros2_ws/src/kinect2_slam/kinect2_slam/mcp_server.py ]; then
    echo "   ✓ mcp_server.py exists"
    if python3 -c "import mcp" 2>/dev/null; then
        echo "   ✓ mcp library installed"
    else
        echo "   ✗ mcp library not installed"
        echo "     Install with: pip install mcp --break-system-packages"
    fi
else
    echo "   ✗ mcp_server.py not found"
fi

echo ""
echo "=== Test Complete ==="
echo ""
echo "To launch the full system:"
echo "  1. Start SLAM + YOLO: ./scripts/run_complete_slam_system.sh"
echo "  2. Start memory system: ros2 launch kinect2_slam howyouseeme_memory.launch.py"
echo "  3. Start MCP server: python3 ros2_ws/src/kinect2_slam/kinect2_slam/mcp_server.py"
echo ""
