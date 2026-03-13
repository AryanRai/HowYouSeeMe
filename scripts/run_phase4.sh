#!/bin/bash
# Phase 4: Semantic Projection System
# Subscribes to /cv_pipeline/results from ANY active CV model (YOLO, SAM2,
# FastSAM, InsightFace, HSEmotion) and back-projects detections to 3D world
# space, publishing TF2 transforms and RViz markers for every entity.
#
# Prerequisites:
#   • Phase 2-3  — run_phase2_3.sh  (ORB-SLAM3 pose on /orb_slam3/pose)
#   • CV Pipeline — scripts/cv_pipeline_menu.sh or sam2_server_v2.py
#                   (results on /cv_pipeline/results)
#   • Kinect depth — /kinect2/hd/image_depth_rect

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

echo "=========================================="
echo "  Phase 4: Semantic Projection"
echo "=========================================="
echo ""
echo "Handles CV pipeline output from:"
echo "  • YOLO11  (detect / segment / pose / obb)"
echo "  • SAM2 / FastSAM (segmentation masks)"
echo "  • InsightFace (face detection & recognition)"
echo "  • HSEmotion (emotion recognition)"
echo ""
echo "Publishes:"
echo "  /semantic/markers      – coloured RViz text labels"
echo "  /semantic/world_state  – persistent JSON world state"
echo "  TF2: map → camera_link (ORB-SLAM3 camera pose)"
echo "  TF2: map → <label>_<id> (every detected entity)"
echo ""
echo "World state saved to /tmp/world_state.json every 5 s"
echo ""
echo "Press Ctrl+C to stop"
echo "=========================================="
echo ""

# Source ROS without conda
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v conda | tr '\n' ':' | sed 's/:$//')
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_ROOT/ros2_ws/install/setup.bash"

# ── Prerequisite checks ────────────────────────────────────────────────────
echo "Checking prerequisites..."
PREREQ_OK=true

if ! ros2 topic list 2>/dev/null | grep -q "/orb_slam3/pose"; then
    echo "❌  /orb_slam3/pose not found — start Phase 2-3 first:"
    echo "    ./scripts/run_phase2_3.sh"
    PREREQ_OK=false
fi

if ! ros2 topic list 2>/dev/null | grep -q "/kinect2/hd/image_depth_rect"; then
    echo "⚠️   /kinect2/hd/image_depth_rect not found (Kinect may not be running)"
fi

if ! ros2 topic list 2>/dev/null | grep -q "/cv_pipeline/results"; then
    echo "⚠️   /cv_pipeline/results not found — CV Pipeline not active."
    echo "    Start it with:  ./scripts/cv_pipeline_menu.sh"
    echo "    The semantic projection node will start and wait for results."
fi

if [ "$PREREQ_OK" = false ]; then
    echo ""
    echo "Fix the errors above and re-run this script."
    exit 1
fi

echo "✓ Prerequisites met"
echo ""

# ── Launch semantic projection node ────────────────────────────────────────
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

