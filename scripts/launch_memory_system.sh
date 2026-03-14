#!/bin/bash
# Launch the 5-tier memory system nodes directly

set -e

echo "=== Launching HowYouSeeMe Memory System ==="

# Source ROS2
source /opt/ros/jazzy/setup.bash
source ~/Documents/GitHub/HowYouSeeMe/ros2_ws/install/setup.bash

# Create directories
mkdir -p /tmp/stm
mkdir -p ~/howyouseeme_persistent

echo ""
echo "Starting memory system nodes..."
echo "Press Ctrl+C to stop all nodes"
echo ""

# Launch nodes in background
python3 ~/Documents/GitHub/HowYouSeeMe/ros2_ws/src/kinect2_slam/kinect2_slam/event_checkpointer_node.py &
PID1=$!
echo "Started event_checkpointer (PID: $PID1)"

python3 ~/Documents/GitHub/HowYouSeeMe/ros2_ws/src/kinect2_slam/kinect2_slam/async_analyser_node.py &
PID2=$!
echo "Started async_analyser (PID: $PID2)"

python3 ~/Documents/GitHub/HowYouSeeMe/ros2_ws/src/kinect2_slam/kinect2_slam/world_synthesiser_node.py &
PID3=$!
echo "Started world_synthesiser (PID: $PID3)"

python3 ~/Documents/GitHub/HowYouSeeMe/ros2_ws/src/kinect2_slam/kinect2_slam/named_memory_node.py &
PID4=$!
echo "Started named_memory (PID: $PID4)"

echo ""
echo "All memory nodes running!"
echo "World state: /tmp/world_state.json"
echo "Checkpoints: /tmp/stm/"
echo "Named memories: ~/howyouseeme_persistent/named_memories.json"
echo ""

# Wait for Ctrl+C
trap "echo ''; echo 'Stopping all nodes...'; kill $PID1 $PID2 $PID3 $PID4 2>/dev/null; exit 0" INT TERM

wait
