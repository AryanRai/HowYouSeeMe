#!/bin/bash
# Complete SLAM System with Semantic Projection
# Launches: Kinect + ORB-SLAM3 + TSDF + Semantic + CV Pipeline + RViz
# Optional: Add --rerun flag to enable recording

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

# Check for --rerun flag
ENABLE_RERUN=false
if [[ "$1" == "--rerun" ]]; then
    ENABLE_RERUN=true
fi

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}  Complete SLAM + Semantic System${NC}"
echo -e "${CYAN}========================================${NC}"
echo ""
echo "This will launch:"
echo "  1. Kinect v2 RGB-D bridge"
echo "  2. ORB-SLAM3 tracking (RGB-D mode)"
echo "  3. TSDF volumetric mapping"
echo "  4. Semantic projection (3D labels)"
echo "  5. CV Pipeline server"
echo "  6. RViz2 visualization"
if [ "$ENABLE_RERUN" = true ]; then
    echo "  7. Rerun logger (recording enabled)"
    SESSION_FILE="/tmp/howyouseeme_$(date +%Y%m%d_%H%M%S).rrd"
    echo ""
    echo -e "${GREEN}Rerun recording:${NC} $SESSION_FILE"
fi
echo ""
echo -e "${YELLOW}After startup, CV Pipeline menu will open${NC}"
echo -e "${YELLOW}Use YOLO detection for semantic mapping${NC}"
echo ""
echo "Press Ctrl+C to stop all components"
echo -e "${CYAN}========================================${NC}"
echo ""

# Cleanup function
cleanup() {
    echo ""
    echo -e "${YELLOW}Shutting down all components...${NC}"
    if [ "$ENABLE_RERUN" = true ]; then
        kill $PHASE2_PID $TSDF_PID $SEMANTIC_PID $CV_PIPELINE_PID $RERUN_PID $RVIZ_PID 2>/dev/null
        wait $PHASE2_PID $TSDF_PID $SEMANTIC_PID $CV_PIPELINE_PID $RERUN_PID $RVIZ_PID 2>/dev/null
    else
        kill $PHASE2_PID $TSDF_PID $SEMANTIC_PID $CV_PIPELINE_PID $RVIZ_PID 2>/dev/null
        wait $PHASE2_PID $TSDF_PID $SEMANTIC_PID $CV_PIPELINE_PID $RVIZ_PID 2>/dev/null
    fi
    echo -e "${GREEN}All processes stopped${NC}"
    if [ "$ENABLE_RERUN" = true ]; then
        echo ""
        echo -e "${CYAN}Rerun recording saved:${NC}"
        echo "  $SESSION_FILE"
        echo ""
        echo "Replay with:"
        echo "  rerun $SESSION_FILE"
        echo ""
    fi
    exit 0
}

trap cleanup SIGINT SIGTERM

# Source ROS without conda
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v conda | tr '\n' ':' | sed 's/:$//')
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_ROOT/ros2_ws/install/setup.bash"

# 1. Start Phase 2 (Kinect + ORB-SLAM3)
echo -e "${BLUE}[1/6]${NC} Starting Kinect + ORB-SLAM3..."
"$SCRIPT_DIR/run_phase2_3.sh" > /tmp/phase2.log 2>&1 &
PHASE2_PID=$!
echo "      PID: $PHASE2_PID (logs: /tmp/phase2.log)"

# Wait for initialization
echo "      Waiting for ORB-SLAM3 initialization..."
sleep 15

if ! kill -0 $PHASE2_PID 2>/dev/null; then
    echo -e "${RED}      ❌ Phase 2 failed. Check /tmp/phase2.log${NC}"
    exit 1
fi
echo -e "${GREEN}      ✓ ORB-SLAM3 running${NC}"

# 2. Start Phase 3 (TSDF)
echo ""
echo -e "${BLUE}[2/6]${NC} Starting TSDF integrator..."
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
echo "      PID: $TSDF_PID (logs: /tmp/tsdf.log)"
sleep 3
echo -e "${GREEN}      ✓ TSDF running${NC}"

# 3. Start Phase 4 (Semantic Projection)
echo ""
echo -e "${BLUE}[3/6]${NC} Starting semantic projection..."
ros2 run cv_pipeline semantic_projection --ros-args \
    -p fx:=1081.37 \
    -p fy:=1081.37 \
    -p cx:=959.5 \
    -p cy:=539.5 \
    -p world_state_path:=/tmp/world_state.json \
    -p marker_lifetime:=30.0 \
    -p conf_threshold:=0.4 \
    -p depth_trunc:=5.0 \
    -p merge_threshold:=0.3 \
    -p flip_x_axis:=true \
    -p flip_y_axis:=true \
    -p debug_projection:=false > /tmp/semantic.log 2>&1 &
SEMANTIC_PID=$!
echo "      PID: $SEMANTIC_PID (logs: /tmp/semantic.log)"
sleep 2
echo -e "${GREEN}      ✓ Semantic projection running${NC}"

# 4. Start Rerun Logger (optional)
if [ "$ENABLE_RERUN" = true ]; then
    echo ""
    echo -e "${BLUE}[4/7]${NC} Starting Rerun logger..."
    ros2 run kinect2_slam rerun_logger --ros-args \
        -p recording_name:=howyouseeme \
        -p spawn_viewer:=true \
        -p save_path:="$SESSION_FILE" \
        -p rgb_downsample:=2 \
        -p pc_max_points:=50000 > /tmp/rerun.log 2>&1 &
    RERUN_PID=$!
    echo "      PID: $RERUN_PID (logs: /tmp/rerun.log)"
    sleep 3
    echo -e "${GREEN}      ✓ Rerun logger running${NC}"
    CV_STEP="[5/7]"
    RVIZ_STEP="[6/7]"
    STATUS_STEP="[7/7]"
else
    CV_STEP="[4/6]"
    RVIZ_STEP="[5/6]"
    STATUS_STEP="[6/6]"
fi

# 5. Start CV Pipeline Server (Python)
echo ""
echo -e "${BLUE}${CV_STEP}${NC} Starting CV Pipeline server (Python)..."
nohup ~/anaconda3/envs/howyouseeme/bin/python \
    "$WORKSPACE_ROOT/ros2_ws/src/cv_pipeline/python/sam2_server_v2.py" \
    > /tmp/cv_pipeline.log 2>&1 &
CV_PIPELINE_PID=$!
echo "      PID: $CV_PIPELINE_PID (logs: /tmp/cv_pipeline.log)"
sleep 5
echo -e "${GREEN}      ✓ CV Pipeline server running${NC}"

# 6. Start RViz2
echo ""
echo -e "${BLUE}${RVIZ_STEP}${NC} Starting RViz2..."
rviz2 -d "$WORKSPACE_ROOT/rviz_configs/tsdf_rviz.rviz" > /tmp/rviz.log 2>&1 &
RVIZ_PID=$!
echo "      PID: $RVIZ_PID"
sleep 3
echo -e "${GREEN}      ✓ RViz2 running${NC}"

# 7. Display system status
echo ""
echo -e "${CYAN}========================================${NC}"
if [ "$ENABLE_RERUN" = true ]; then
    echo -e "${GREEN}  🎉 System Running with Rerun!${NC}"
else
    echo -e "${GREEN}  🎉 System Running!${NC}"
fi
echo -e "${CYAN}========================================${NC}"
echo ""
echo -e "${YELLOW}Process IDs:${NC}"
echo "  Phase 2 (SLAM):      $PHASE2_PID"
echo "  Phase 3 (TSDF):      $TSDF_PID"
echo "  Phase 4 (Semantic):  $SEMANTIC_PID"
echo "  CV Pipeline:         $CV_PIPELINE_PID"
if [ "$ENABLE_RERUN" = true ]; then
    echo "  Rerun Logger:        $RERUN_PID"
fi
echo "  RViz2:               $RVIZ_PID"
echo ""
echo -e "${YELLOW}Logs:${NC}"
echo "  Phase 2:     /tmp/phase2.log"
echo "  TSDF:        /tmp/tsdf.log"
echo "  Semantic:    /tmp/semantic.log"
echo "  CV Pipeline: /tmp/cv_pipeline.log"
if [ "$ENABLE_RERUN" = true ]; then
    echo "  Rerun:       /tmp/rerun.log"
fi
echo "  RViz:        /tmp/rviz.log"
echo ""
echo -e "${YELLOW}World State:${NC}"
echo "  JSON file:   /tmp/world_state.json"
echo "  ROS topic:   /semantic/world_state"
echo ""
echo -e "${YELLOW}RViz Displays:${NC}"
echo "  • TSDF Point Cloud:  /tsdf/pointcloud"
echo "  • Semantic Labels:   /semantic/markers"
echo "  • TF Frames:         map → kinect2_rgb_optical_frame"
if [ "$ENABLE_RERUN" = true ]; then
    echo ""
    echo -e "${YELLOW}Rerun Viewer:${NC}"
    echo "  • Timeline scrubbing enabled"
    echo "  • Recording to: $SESSION_FILE"
    echo "  • All streams synced by timestamp"
fi
echo ""
echo -e "${YELLOW}Useful Commands:${NC}"
echo "  Export mesh:     ros2 service call /tsdf/export_mesh std_srvs/srv/Trigger"
echo "  View world:      cat /tmp/world_state.json | python3 -m json.tool"
echo "  Monitor topics:  ros2 topic list"
echo ""
echo -e "${CYAN}========================================${NC}"
echo ""

# 8. Launch CV Pipeline Menu
echo -e "${BLUE}${STATUS_STEP}${NC} Opening CV Pipeline menu..."
echo ""
echo -e "${YELLOW}Recommended: Start YOLO detection for semantic mapping${NC}"
echo "  Select: 3) YOLO11 → 1) Detection → Stream mode"
echo ""
sleep 3

# Run the menu in foreground
"$SCRIPT_DIR/cv_pipeline_menu.sh"

# When menu exits, cleanup
cleanup
