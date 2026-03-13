#!/bin/bash
# Phase 4: Semantic Projection System
# Requires Phase 2 (ORB-SLAM3) and CV Pipeline (YOLO) running

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

echo "=========================================="
echo "  Phase 4: Semantic Projection"
echo "=========================================="
echo ""
echo "Prerequisites:"
echo "  ✓ Phase 2 (ORB-SLAM3) must be running"
echo "  ✓ CV Pipeline (YOLO) must be running"
echo "  ✓ /cv_pipeline/results topic publishing"
echo ""
echo "This node will:"
echo "  - Back-project YOLO detections to 3D"
echo "  - Publish floating text labels in RViz"
echo "  - Save world state to /tmp/world_state.json"
echo ""
echo "Press Ctrl+C to stop"
echo "=========================================="
echo ""

# Source ROS without conda
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v conda | tr '\n' ':' | sed 's/:$//')
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_ROOT/ros2_ws/install/setup.bash"

# Check prerequisites
echo "Checking prerequisites..."

if ! ros2 topic list | grep -q "/orb_slam3/pose"; then
    echo "❌ Error: /orb_slam3/pose not found"
    echo "   Start Phase 2 first: ./scripts/run_phase2_3.sh"
    exit 1
fi

if ! ros2 topic list | grep -q "/cv_pipeline/results"; then
    echo "❌ Error: /cv_pipeline/results not found"
    echo "   Start CV Pipeline first"
    exit 1
fi

echo "✓ All prerequisites met"
echo ""

# Run semantic projection node
echo "Starting semantic projection node..."
ros2 run cv_pipeline semantic_projection \
    --ros-args \
    -p fx:=1081.37 \
    -p fy:=1081.37 \
    -p cx:=959.5 \
    -p cy:=539.5 \
    -p world_state_path:=/tmp/world_state.json \
    -p marker_lifetime:=30.0 \
    -p conf_threshold:=0.4 \
    -p depth_trunc:=5.0

