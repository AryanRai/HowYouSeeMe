#!/bin/bash
# Test YOLO + Semantic Projection

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v conda | tr '\n' ':' | sed 's/:$//')
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_ROOT/ros2_ws/install/setup.bash"

echo "=========================================="
echo "  YOLO + Semantic Projection Test"
echo "=========================================="
echo ""

# Start CV Pipeline
echo "[1/2] Starting CV Pipeline server..."
ros2 run cv_pipeline cv_pipeline_node > /tmp/cv_test.log 2>&1 &
CV_PID=$!
echo "      PID: $CV_PID"
sleep 5

# Start Semantic Projection
echo "[2/2] Starting Semantic Projection..."
ros2 run cv_pipeline semantic_projection --ros-args \
    -p fx:=1081.37 -p fy:=1081.37 -p cx:=959.5 -p cy:=539.5 \
    -p world_state_path:=/tmp/world_state.json \
    -p marker_lifetime:=30.0 -p conf_threshold:=0.4 \
    -p depth_trunc:=5.0 > /tmp/semantic_test.log 2>&1 &
SEM_PID=$!
echo "      PID: $SEM_PID"
sleep 3

echo ""
echo "=========================================="
echo "  Sending YOLO Detection Request"
echo "=========================================="
echo ""
echo "Request: YOLO detect @ 5 FPS for 10 seconds"
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'yolo11:task=detect,conf=0.4,stream=true,duration=10,fps=5'"

echo ""
echo "Waiting for detections..."
sleep 3

echo ""
echo "Checking /cv_pipeline/results..."
timeout 2 ros2 topic echo /cv_pipeline/results --once 2>&1 | head -30

echo ""
echo "Checking /semantic/markers..."
timeout 2 ros2 topic hz /semantic/markers 2>&1 | head -5

echo ""
echo "Checking world state..."
if [ -f /tmp/world_state.json ]; then
    echo "World state file exists:"
    cat /tmp/world_state.json | python3 -m json.tool 2>/dev/null | head -20
else
    echo "No world state file yet"
fi

echo ""
echo "=========================================="
echo "Logs:"
echo "  CV Pipeline: /tmp/cv_test.log"
echo "  Semantic:    /tmp/semantic_test.log"
echo ""
echo "Press Ctrl+C to stop"
echo "=========================================="

# Wait
wait $CV_PID $SEM_PID
