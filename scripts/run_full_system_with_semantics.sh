#!/bin/bash
# Complete System: Phase 2 + Phase 3 + Phase 4
# Kinect + ORB-SLAM3 + TSDF + Semantic Projection

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

echo "=========================================="
echo "  Full SLAM + Semantic System"
echo "=========================================="
echo ""
echo "Components:"
echo "  1. Kinect v2 RGB-D"
echo "  2. ORB-SLAM3 tracking"
echo "  3. TSDF volumetric mapping"
echo "  4. Semantic projection (3D labels)"
echo "  5. RViz2 visualization"
echo ""
echo "Note: Start CV Pipeline separately for YOLO"
echo "Press Ctrl+C to stop"
echo "=========================================="
echo ""

# Cleanup function
cleanup() {
    echo ""
    echo "Shutting down all components..."
    kill $PHASE2_PID $TSDF_PID $SEMANTIC_PID $RVIZ_PID 2>/dev/null
    wait $PHASE2_PID $TSDF_PID $SEMANTIC_PID $RVIZ_PID 2>/dev/null
    echo "All processes stopped"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Source ROS without conda
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v conda | tr '\n' ':' | sed 's/:$//')
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_ROOT/ros2_ws/install/setup.bash"

# 1. Start Phase 2 (Kinect + ORB-SLAM3)
echo "Starting Phase 2 (Kinect + ORB-SLAM3)..."
"$SCRIPT_DIR/run_phase2_3.sh" > /tmp/phase2.log 2>&1 &
PHASE2_PID=$!
echo "   PID: $PHASE2_PID (logs: /tmp/phase2.log)"

# Wait for initialization
echo "   Waiting for ORB-SLAM3..."
sleep 15

if ! kill -0 $PHASE2_PID 2>/dev/null; then
    echo "   ❌ Phase 2 failed. Check /tmp/phase2.log"
    exit 1
fi

# 2. Start Phase 3 (TSDF)
echo ""
echo "Starting Phase 3 (TSDF integrator)..."
ros2 run kinect2_slam tsdf_integrator --ros-args \
    -p voxel_length:=0.04 \
    -p sdf_trunc:=0.08 \
    -p publish_rate:=1.0 \
    -p export_path:=/tmp/tsdf_mesh.ply \
    -p fx:=1081.37 \
    -p fy:=1081.37 \
    -p cx:=960.0 \
    -p cy:=540.0 > /tmp/tsdf.log 2>&1 &
TSDF_PID=$!
echo "   PID: $TSDF_PID (logs: /tmp/tsdf.log)"

sleep 3

# 3. Start Phase 4 (Semantic Projection)
echo ""
echo "Starting Phase 4 (Semantic projection)..."
echo "   Note: Requires CV Pipeline running for YOLO detections"

ros2 run cv_pipeline semantic_projection --ros-args \
    -p fx:=1081.37 \
    -p fy:=1081.37 \
    -p cx:=959.5 \
    -p cy:=539.5 \
    -p world_state_path:=/tmp/world_state.json \
    -p marker_lifetime:=30.0 \
    -p conf_threshold:=0.4 \
    -p depth_trunc:=5.0 > /tmp/semantic.log 2>&1 &
SEMANTIC_PID=$!
echo "   PID: $SEMANTIC_PID (logs: /tmp/semantic.log)"

sleep 2

# 4. Start RViz2
echo ""
echo "Starting RViz2..."
rviz2 -d "$WORKSPACE_ROOT/tsdf_rviz.rviz" > /tmp/rviz.log 2>&1 &
RVIZ_PID=$!
echo "   PID: $RVIZ_PID"

echo ""
echo "=========================================="
echo "  🎉 System Running!"
echo "=========================================="
echo ""
echo "Process IDs:"
echo "  Phase 2 (SLAM):      $PHASE2_PID"
echo "  Phase 3 (TSDF):      $TSDF_PID"
echo "  Phase 4 (Semantic):  $SEMANTIC_PID"
echo "  RViz2:               $RVIZ_PID"
echo ""
echo "Logs:"
echo "  Phase 2:   /tmp/phase2.log"
echo "  TSDF:      /tmp/tsdf.log"
echo "  Semantic:  /tmp/semantic.log"
echo "  RViz:      /tmp/rviz.log"
echo ""
echo "World state: /tmp/world_state.json"
echo ""
echo "To start CV Pipeline (YOLO):"
echo "  In another terminal:"
echo "  cd ~/Documents/GitHub/HowYouSeeMe"
echo "  source ros2_ws/install/setup.bash"
echo "  ros2 run cv_pipeline cv_pipeline_node"
echo ""
echo "Export mesh:"
echo "  ros2 service call /tsdf/export_mesh std_srvs/srv/Trigger"
echo ""
echo "Press Ctrl+C to stop"
echo "=========================================="

# Wait for processes
wait $PHASE2_PID $TSDF_PID $SEMANTIC_PID $RVIZ_PID
