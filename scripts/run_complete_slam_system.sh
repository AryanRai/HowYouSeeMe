#!/bin/bash
# Complete SLAM System with Semantic Projection
# Launches: Kinect + ORB-SLAM3 + TSDF + Semantic + CV Pipeline + RViz

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

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
echo "  7. Memory system (5-tier)"
echo ""
echo -e "${YELLOW}After startup, CV Pipeline menu will open${NC}"
echo -e "${YELLOW}Use YOLO detection for semantic mapping${NC}"
echo ""
echo -e "${YELLOW}Note: Rerun C++ disabled (Arrow version mismatch)${NC}"
echo ""
echo "Press Ctrl+C to stop all components"
echo -e "${CYAN}========================================${NC}"
echo ""

# Cleanup function
cleanup() {
    echo ""
    echo -e "${YELLOW}Shutting down all components...${NC}"
    kill $PHASE2_PID $TSDF_PID $SEMANTIC_PID $CV_PIPELINE_PID $MEMORY_PID1 $MEMORY_PID2 $MEMORY_PID3 $MEMORY_PID4 $RVIZ_PID 2>/dev/null
    wait $PHASE2_PID $TSDF_PID $SEMANTIC_PID $CV_PIPELINE_PID $MEMORY_PID1 $MEMORY_PID2 $MEMORY_PID3 $MEMORY_PID4 $RVIZ_PID 2>/dev/null
    echo -e "${GREEN}All processes stopped${NC}"
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
python3 "$WORKSPACE_ROOT/ros2_ws/src/kinect2_slam/kinect2_slam/tsdf_integrator_node.py" \
    --ros-args \
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
python3 "$WORKSPACE_ROOT/ros2_ws/src/cv_pipeline/cv_pipeline/semantic_projection_node.py" \
    --ros-args \
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

# 4. Start Memory System (5-tier)
echo ""
echo -e "${BLUE}[4/8]${NC} Starting memory system (5-tier)..."
    
    # Create directories
    mkdir -p /tmp/stm
    mkdir -p ~/howyouseeme_persistent
    
    # Event checkpointer
    python3 "$WORKSPACE_ROOT/ros2_ws/src/kinect2_slam/kinect2_slam/event_checkpointer_node.py" > /tmp/memory_checkpointer.log 2>&1 &
    MEMORY_PID1=$!
    echo "      Event checkpointer PID: $MEMORY_PID1"
    
    # Async analyser
    python3 "$WORKSPACE_ROOT/ros2_ws/src/kinect2_slam/kinect2_slam/async_analyser_node.py" > /tmp/memory_analyser.log 2>&1 &
    MEMORY_PID2=$!
    echo "      Async analyser PID: $MEMORY_PID2"
    
    # World synthesiser
    python3 "$WORKSPACE_ROOT/ros2_ws/src/kinect2_slam/kinect2_slam/world_synthesiser_node.py" > /tmp/memory_synthesiser.log 2>&1 &
    MEMORY_PID3=$!
    echo "      World synthesiser PID: $MEMORY_PID3"
    
    # Named memory
    python3 "$WORKSPACE_ROOT/ros2_ws/src/kinect2_slam/kinect2_slam/named_memory_node.py" > /tmp/memory_named.log 2>&1 &
    MEMORY_PID4=$!
    echo "      Named memory PID: $MEMORY_PID4"
    
    sleep 3
    echo -e "${GREEN}      ✓ Memory system running (4 nodes)${NC}"

# 5. Start CV Pipeline Server (Python) - use system python for NumPy 1.x compatibility
echo ""
echo -e "${BLUE}[5/7]${NC} Starting CV Pipeline server (Python)..."
nohup python3 \
    "$WORKSPACE_ROOT/ros2_ws/src/cv_pipeline/python/sam2_server_v2.py" \
    > /tmp/cv_pipeline.log 2>&1 &
CV_PIPELINE_PID=$!
echo "      PID: $CV_PIPELINE_PID (logs: /tmp/cv_pipeline.log)"
sleep 5
echo -e "${GREEN}      ✓ CV Pipeline server running${NC}"

# 6. Start RViz2
echo ""
echo -e "${BLUE}[6/7]${NC} Starting RViz2..."
rviz2 -d "$WORKSPACE_ROOT/rviz_configs/tsdf_rviz.rviz" > /tmp/rviz.log 2>&1 &
RVIZ_PID=$!
echo "      PID: $RVIZ_PID"
sleep 3
echo -e "${GREEN}      ✓ RViz2 running${NC}"

# 8. Display system status
echo ""
echo -e "${CYAN}========================================${NC}"
echo -e "${GREEN}  🎉 System Running!${NC}"
echo -e "${CYAN}========================================${NC}"
echo ""
echo -e "${YELLOW}Process IDs:${NC}"
echo "  Phase 2 (SLAM):      $PHASE2_PID"
echo "  Phase 3 (TSDF):      $TSDF_PID"
echo "  Phase 4 (Semantic):  $SEMANTIC_PID"
echo "  CV Pipeline:         $CV_PIPELINE_PID"
echo "  Memory System:"
echo "    - Checkpointer:    $MEMORY_PID1"
echo "    - Analyser:        $MEMORY_PID2"
echo "    - Synthesiser:     $MEMORY_PID3"
echo "    - Named Memory:    $MEMORY_PID4"
echo "  RViz2:               $RVIZ_PID"
echo ""
echo -e "${YELLOW}Logs:${NC}"
echo "  Phase 2:     /tmp/phase2.log"
echo "  TSDF:        /tmp/tsdf.log"
echo "  Semantic:    /tmp/semantic.log"
echo "  CV Pipeline: /tmp/cv_pipeline.log"
echo "  Memory:"
echo "    - Checkpointer:  /tmp/memory_checkpointer.log"
echo "    - Analyser:      /tmp/memory_analyser.log"
echo "    - Synthesiser:   /tmp/memory_synthesiser.log"
echo "    - Named Memory:  /tmp/memory_named.log"
echo "  RViz:        /tmp/rviz.log"
echo ""
echo -e "${YELLOW}World State:${NC}"
echo "  JSON file:   /tmp/world_state.json"
echo "  ROS topic:   /semantic/world_state"
echo ""
echo -e "${YELLOW}Memory System:${NC}"
echo "  Checkpoints:     /tmp/stm/"
echo "  Named memories:  ~/howyouseeme_persistent/named_memories.json"
echo "  Topics:"
echo "    - /memory/checkpoint_saved"
echo "    - /memory/checkpoint_enriched"
echo "    - /memory/updated"
echo ""
echo -e "${YELLOW}RViz Displays:${NC}"
echo "  • TSDF Point Cloud:  /tsdf/pointcloud"
echo "  • Semantic Labels:   /semantic/markers"
echo "  • TF Frames:         map → kinect2_rgb_optical_frame"
echo ""
echo -e "${YELLOW}Useful Commands:${NC}"
echo "  Export mesh:     ros2 service call /tsdf/export_mesh std_srvs/srv/Trigger"
echo "  View world:      cat /tmp/world_state.json | python3 -m json.tool"
echo "  Monitor topics:  ros2 topic list"
echo ""
echo -e "${CYAN}========================================${NC}"
echo ""

# 9. Launch CV Pipeline Menu in new terminal
echo ""
echo -e "${BLUE}[7/7]${NC} Opening CV Pipeline menu in new terminal..."
echo ""
echo -e "${YELLOW}Recommended: Start YOLO detection for semantic mapping${NC}"
echo "  Select: 3) YOLO11 → 1) Detection → Stream mode"
echo ""

# Try to open in a new terminal
if command -v gnome-terminal &> /dev/null; then
    gnome-terminal -- bash -c "cd '$WORKSPACE_ROOT' && '$SCRIPT_DIR/cv_pipeline_menu.sh'; exec bash" &
    echo -e "${GREEN}      ✓ CV Pipeline menu opened in new terminal${NC}"
elif command -v xterm &> /dev/null; then
    xterm -e "cd '$WORKSPACE_ROOT' && '$SCRIPT_DIR/cv_pipeline_menu.sh'; bash" &
    echo -e "${GREEN}      ✓ CV Pipeline menu opened in new terminal${NC}"
elif command -v konsole &> /dev/null; then
    konsole -e bash -c "cd '$WORKSPACE_ROOT' && '$SCRIPT_DIR/cv_pipeline_menu.sh'; exec bash" &
    echo -e "${GREEN}      ✓ CV Pipeline menu opened in new terminal${NC}"
else
    echo -e "${YELLOW}      ⚠ No terminal emulator found. Run manually:${NC}"
    echo "        $SCRIPT_DIR/cv_pipeline_menu.sh"
fi

echo ""
echo -e "${CYAN}========================================${NC}"
echo -e "${GREEN}  System is running!${NC}"
echo -e "${CYAN}========================================${NC}"
echo ""
echo "Press Ctrl+C to stop all components"
echo ""

# Wait for all processes
wait
