#!/bin/bash
# Restart semantic projection node with updated code

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

echo "Stopping old semantic projection instances..."
pkill -f semantic_projection
sleep 1

echo "Starting semantic projection with updated code..."
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v conda | tr '\n' ':' | sed 's/:$//')
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_ROOT/ros2_ws/install/setup.bash"

ros2 run cv_pipeline semantic_projection --ros-args \
    -p fx:=1081.37 \
    -p fy:=1081.37 \
    -p cx:=959.5 \
    -p cy:=539.5 \
    -p world_state_path:=/tmp/world_state.json \
    -p marker_lifetime:=30.0 \
    -p conf_threshold:=0.4 \
    -p depth_trunc:=5.0 > /tmp/semantic_new.log 2>&1 &

SEMANTIC_PID=$!
echo "Semantic projection started: PID $SEMANTIC_PID"
echo "Logs: /tmp/semantic_new.log"
echo ""
echo "Checking status..."
sleep 2

if ps -p $SEMANTIC_PID > /dev/null; then
    echo "✓ Running successfully"
    tail -10 /tmp/semantic_new.log
else
    echo "✗ Failed to start"
    cat /tmp/semantic_new.log
fi
