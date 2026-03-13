#!/bin/bash
# Complete Phase 2-3 System with RViz
# Launches everything in background and opens RViz for visualization

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

echo "=========================================="
echo "  Complete SLAM System Launch"
echo "=========================================="
echo ""
echo "This will start:"
echo "  1. Kinect v2 + BlueLily IMU"
echo "  2. ORB-SLAM3 tracking"
echo "  3. TSDF integrator (in conda)"
echo "  4. RViz2 visualization"
echo ""
echo "Press Ctrl+C to stop everything"
echo "=========================================="
echo ""

# Cleanup function
cleanup() {
    echo ""
    echo "Shutting down all components..."
    kill $PHASE2_PID $TSDF_PID $RVIZ_PID 2>/dev/null
    wait $PHASE2_PID $TSDF_PID $RVIZ_PID 2>/dev/null
    echo "All processes stopped"
    exit 0
}

trap cleanup SIGINT SIGTERM

# 1. Start Phase 2 (no conda)
echo "Starting Phase 2 (Kinect + IMU + ORB-SLAM3)..."
"$SCRIPT_DIR/run_phase2_3.sh" 2>&1 | tee /tmp/phase2.log &
PHASE2_PID=$!
echo "   PID: $PHASE2_PID"
echo "   Logs streaming to terminal and /tmp/phase2.log"

# Wait for Phase 2 to initialize
echo "   Waiting for ORB-SLAM3 to start..."
sleep 15

# Check if Phase 2 is still running
if ! kill -0 $PHASE2_PID 2>/dev/null; then
    echo "   ❌ Phase 2 failed to start. Check /tmp/phase2.log"
    exit 1
fi

# 2. Start Phase 3 TSDF (system open3d, no conda)
echo ""
echo "Starting Phase 3 (TSDF integrator)..."

# Source ROS without conda
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v conda | tr '\n' ':' | sed 's/:$//')
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_ROOT/ros2_ws/install/setup.bash"

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

# Wait for TSDF to start
sleep 3

# 3. Start RViz2
echo ""
echo "Starting RViz2..."
echo "   Loading TSDF visualization config..."
echo ""

# Source ROS without conda
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v conda | tr '\n' ':' | sed 's/:$//')
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_ROOT/ros2_ws/install/setup.bash"

rviz2 -d "$WORKSPACE_ROOT/tsdf_rviz.rviz" > /tmp/rviz.log 2>&1 &
RVIZ_PID=$!
echo "   PID: $RVIZ_PID"

echo ""
echo "=========================================="
echo "  🎉 System Running!"
echo "=========================================="
echo ""
echo "Process IDs:"
echo "  Phase 2 (SLAM):    $PHASE2_PID"
echo "  Phase 3 (TSDF):    $TSDF_PID"
echo "  RViz2:             $RVIZ_PID"
echo ""
echo "Logs:"
echo "  Phase 2: /tmp/phase2.log"
echo "  TSDF:    /tmp/tsdf.log"
echo "  RViz:    /tmp/rviz.log"
echo ""
echo "In RViz:"
echo "  - TSDF point cloud should be visible automatically"
echo "  - Fixed Frame is set to 'map'"
echo "  - Move the Kinect slowly to build 3D reconstruction"
echo ""
echo "Export mesh:"
echo "  ros2 service call /tsdf/export_mesh std_srvs/srv/Trigger"
echo ""
echo "Press Ctrl+C to stop all processes"
echo "=========================================="

# Wait for processes
wait $PHASE2_PID $TSDF_PID $RVIZ_PID
