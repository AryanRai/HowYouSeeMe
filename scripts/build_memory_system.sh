#!/bin/bash
# Build and setup the 5-tier memory system

set -e

echo "=== Building HowYouSeeMe Memory System ==="

# Create persistent directory
mkdir -p ~/howyouseeme_persistent
echo "Created ~/howyouseeme_persistent/"

# Create short-term memory directory
mkdir -p /tmp/stm
echo "Created /tmp/stm/"

# Build the package
cd ~/Documents/GitHub/HowYouSeeMe/ros2_ws
source /opt/ros/jazzy/setup.bash

echo ""
echo "Building kinect2_slam package..."
colcon build --packages-select kinect2_slam --symlink-install

echo ""
echo "=== Build Complete ==="
echo ""
echo "Memory system components installed:"
echo "  - event_checkpointer (Tier 1)"
echo "  - async_analyser (Tier 2)"
echo "  - world_synthesiser (Tier 3)"
echo "  - named_memory (Tier 3)"
echo "  - mcp_server (Tier 5 - run separately)"
echo ""
echo "To launch the memory system:"
echo "  source ~/Documents/GitHub/HowYouSeeMe/ros2_ws/install/setup.bash"
echo "  ros2 launch kinect2_slam howyouseeme_memory.launch.py"
echo ""
echo "To run MCP server (in separate terminal):"
echo "  python3 ~/Documents/GitHub/HowYouSeeMe/ros2_ws/src/kinect2_slam/kinect2_slam/mcp_server.py"
echo ""
