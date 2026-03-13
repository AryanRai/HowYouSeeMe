#!/bin/bash
# ============================================================
# run_full_system.sh — HowYouSeeMe complete pipeline launcher
# ============================================================
#
# Starts every subsystem in the correct order:
#
#   Phase 2-3  │ Kinect v2 bridge
#              │ BlueLily IMU (if attached)
#              │ ORB-SLAM3 RGB-D+IMU → /orb_slam3/pose
#              │ TSDF integrator    → /tsdf/pointcloud  (if Open3D present)
#
#   CV Pipeline│ sam2_server_v2.py (all models on /cv_pipeline/results)
#   (server)   │ Use cv_pipeline_menu.sh in another terminal to trigger runs
#
#   Phase 4    │ semantic_projection_node.py
#              │ Handles YOLO / SAM2 / FastSAM / InsightFace / HSEmotion
#              │ Publishes TF2 (camera_link + per-entity frames)
#              │ Publishes /semantic/markers  (RViz)
#              │ Saves    /tmp/world_state.json
#
#   RViz       │ full_system_rviz.rviz
#
# Usage:
#   ./scripts/run_full_system.sh            # start everything
#   ./scripts/cv_pipeline_menu.sh           # (separate terminal) trigger models
# ============================================================

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

# ── Colour helpers ──────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; CYAN='\033[0;36m'; NC='\033[0m'

info()    { echo -e "${BLUE}ℹ  $*${NC}"; }
success() { echo -e "${GREEN}✅ $*${NC}"; }
warn()    { echo -e "${YELLOW}⚠️  $*${NC}"; }
error()   { echo -e "${RED}❌ $*${NC}"; }

# ── Remove conda from PATH to avoid GLIBCXX conflicts ──────────────────────
if [ -n "${CONDA_DEFAULT_ENV:-}" ]; then
    info "Removing conda from PATH to avoid GLIBCXX conflicts..."
    export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v conda | tr '\n' ':' | sed 's/:$//')
    unset CONDA_DEFAULT_ENV CONDA_PREFIX CONDA_PYTHON_EXE CONDA_SHLVL
fi

# ── Library paths ──────────────────────────────────────────────────────────
export LD_LIBRARY_PATH="${WORKSPACE_ROOT}/libfreenect2/freenect2/lib:${LD_LIBRARY_PATH:-}"

# ── Source ROS ─────────────────────────────────────────────────────────────
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE_ROOT}/ros2_ws/install/setup.bash"

# ── PID tracking ───────────────────────────────────────────────────────────
PIDS=()

cleanup() {
    echo ""
    info "Shutting down all components..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    for pid in "${PIDS[@]}"; do
        wait "$pid" 2>/dev/null || true
    done
    success "All processes stopped"
    exit 0
}
trap cleanup SIGINT SIGTERM

# ── Feature detection ──────────────────────────────────────────────────────
BLUELILY_ENABLED=false
TSDF_ENABLED=false
CV_SERVER_SCRIPT=""

if [ -e "/dev/ttyACM0" ]; then
    BLUELILY_ENABLED=true
fi

if python3 -c "import open3d" 2>/dev/null; then
    TSDF_ENABLED=true
fi

# Prefer the conda env Python for the CV server (has torch, sam2, etc.)
CV_SERVER_PYTHON=""
for candidate in \
    "${HOME}/anaconda3/envs/howyouseeme/bin/python" \
    "${HOME}/miniconda3/envs/howyouseeme/bin/python" \
    "$(conda run -n howyouseeme which python 2>/dev/null || true)"; do
    if [ -x "$candidate" ]; then
        CV_SERVER_PYTHON="$candidate"
        break
    fi
done
CV_SERVER_PYTHON="${CV_SERVER_PYTHON:-python3}"

CV_SERVER_SCRIPT="${WORKSPACE_ROOT}/ros2_ws/src/cv_pipeline/python/sam2_server_v2.py"

# ── Banner ─────────────────────────────────────────────────────────────────
if [ "$TSDF_ENABLED" = true ]; then
    TSDF_STATUS="TSDF (Open3D)"
else
    TSDF_STATUS="TSDF disabled (install open3d)"
fi
echo ""
echo -e "${CYAN}============================================================${NC}"
echo -e "${CYAN}  HowYouSeeMe — Full Pipeline${NC}"
echo -e "${CYAN}============================================================${NC}"
echo ""
echo "  Phases 2-3  : ORB-SLAM3 + ${TSDF_STATUS}"
echo "  CV Pipeline : sam2_server_v2.py (YOLO / SAM2 / InsightFace / ...)"
echo "  Phase 4     : semantic_projection_node (TF2 + RViz markers)"
echo "  RViz        : full_system_rviz.rviz"
if [ "$BLUELILY_ENABLED" = true ]; then
    echo "  IMU         : BlueLily on /dev/ttyACM0"
fi
echo ""
echo "  Interact via: ./scripts/cv_pipeline_menu.sh (new terminal)"
echo ""
echo -e "${CYAN}============================================================${NC}"
echo ""

# ── Step 1 – BlueLily IMU ──────────────────────────────────────────────────
if [ "$BLUELILY_ENABLED" = true ]; then
    info "Starting BlueLily IMU bridge..."
    [ ! -w "/dev/ttyACM0" ] && sudo chmod 666 /dev/ttyACM0
    ros2 run bluelily_bridge bluelily_imu_node --ros-args \
        -p port:=/dev/ttyACM0 \
        -p baud_rate:=115200 \
        -p frame_id:=bluelily_imu &
    PIDS+=($!)
    sleep 2

    info "Publishing static TF: kinect2_link → bluelily_imu"
    ros2 run tf2_ros static_transform_publisher \
        -0.1 0 0 0 0 0 kinect2_link bluelily_imu &
    PIDS+=($!)
    sleep 1
    success "IMU ready"
fi

# ── Step 2 – Kinect v2 bridge ──────────────────────────────────────────────
info "Starting Kinect v2 bridge..."
ros2 launch kinect2_bridge kinect2_bridge_launch.yaml &
PIDS+=($!)
info "Waiting for Kinect to initialise (8 s)..."
sleep 8

for attempt in $(seq 1 5); do
    if ros2 topic list 2>/dev/null | grep -q "/kinect2/hd/image_color"; then
        success "Kinect topics available"
        break
    fi
    [ "$attempt" -lt 5 ] && { warn "Waiting for topics... ($attempt/5)"; sleep 2; } \
                          || warn "Kinect topics not found — continuing anyway"
done

# ── Step 3 – ORB-SLAM3 ────────────────────────────────────────────────────
info "Starting ORB-SLAM3 RGB-D+IMU tracking..."
ros2 run kinect2_slam orb_slam3_node --ros-args \
    -p voc_file:="${HOME}/ORB_SLAM3/Vocabulary/ORBvoc.txt" \
    -p settings_file:="${WORKSPACE_ROOT}/orb_slam3_configs/Kinect2_RGBD_IMU.yaml" \
    -r /camera/rgb/image_raw:=/kinect2/hd/image_color \
    -r /camera/depth_registered/image_raw:=/kinect2/hd/image_depth_rect \
    -r /imu:=/imu/data &
PIDS+=($!)
sleep 2
success "ORB-SLAM3 started  →  /orb_slam3/pose"

# ── Step 4 – TSDF integrator ──────────────────────────────────────────────
if [ "$TSDF_ENABLED" = true ]; then
    info "Starting TSDF volumetric integrator..."
    ros2 run kinect2_slam tsdf_integrator --ros-args \
        -p voxel_length:=0.04 \
        -p sdf_trunc:=0.08 \
        -p publish_rate:=1.0 \
        -p export_path:=/tmp/tsdf_mesh.ply \
        -p fx:=1081.37 \
        -p fy:=1081.37 \
        -p cx:=960.0 \
        -p cy:=540.0 &
    PIDS+=($!)
    sleep 2
    success "TSDF integrator started  →  /tsdf/pointcloud"
else
    warn "TSDF disabled (open3d not found)"
fi

# ── Step 5 – CV Pipeline server ───────────────────────────────────────────
if [ -f "$CV_SERVER_SCRIPT" ]; then
    info "Starting CV pipeline server (all models)..."
    "$CV_SERVER_PYTHON" "$CV_SERVER_SCRIPT" \
        --ros-args \
        -r __ns:=/ &
    PIDS+=($!)
    sleep 3
    success "CV pipeline server started  →  /cv_pipeline/results"
    info "  Trigger models: ./scripts/cv_pipeline_menu.sh"
else
    warn "CV server not found at ${CV_SERVER_SCRIPT}"
    warn "Launch it manually: ./scripts/cv_pipeline_menu.sh"
fi

# ── Step 6 – Semantic projection (Phase 4) ────────────────────────────────
info "Starting semantic projection node (Phase 4)..."
ros2 run cv_pipeline semantic_projection \
    --ros-args \
    -p fx:=1081.37 \
    -p fy:=1081.37 \
    -p cx:=959.5 \
    -p cy:=539.5 \
    -p world_state_path:=/tmp/world_state.json \
    -p marker_lifetime:=30.0 \
    -p conf_threshold:=0.4 \
    -p depth_trunc:=5.0 &
PIDS+=($!)
sleep 2
success "Semantic projection ready"
success "  TF2  : map → camera_link  (ORB-SLAM3 camera pose)"
success "  TF2  : map → <label>_<id> (per-entity frames)"
success "  Pub  : /semantic/markers, /semantic/world_state"

# ── Step 7 – RViz ─────────────────────────────────────────────────────────
RVIZ_CFG="${WORKSPACE_ROOT}/full_system_rviz.rviz"
if [ -f "$RVIZ_CFG" ]; then
    info "Launching RViz..."
    ros2 run rviz2 rviz2 -d "$RVIZ_CFG" &
    PIDS+=($!)
    sleep 2
    success "RViz launched"
else
    warn "RViz config not found at ${RVIZ_CFG}"
fi

# ── Summary ────────────────────────────────────────────────────────────────
echo ""
echo -e "${CYAN}============================================================${NC}"
echo -e "${GREEN}  🎉 Full system running!${NC}"
echo -e "${CYAN}============================================================${NC}"
echo ""
echo "Topics:"
echo "  /kinect2/hd/image_color        – RGB camera feed"
echo "  /kinect2/hd/image_depth_rect   – Depth image"
echo "  /orb_slam3/pose                – Camera pose (world frame)"
[ "$TSDF_ENABLED" = true ] && echo "  /tsdf/pointcloud               – Dense 3-D reconstruction"
echo "  /cv_pipeline/results           – Active CV model output (JSON)"
echo "  /cv_pipeline/visualization     – Annotated camera image"
echo "  /semantic/markers              – RViz 3-D text labels"
echo "  /semantic/world_state          – JSON world state"
echo ""
echo "TF2 Frames:"
echo "  map → camera_link              – ORB-SLAM3 camera pose"
echo "  map → yolo_<label>_<id>        – YOLO detections"
echo "  map → face_<name>_<id>         – InsightFace detections"
echo "  map → sam2_seg_<id>            – SAM2 segments"
echo "  map → fastsam_seg_<id>         – FastSAM segments"
echo ""
echo "CV pipeline control (new terminal):"
echo "  ./scripts/cv_pipeline_menu.sh"
echo ""
echo "Export TSDF mesh:"
echo "  ros2 service call /tsdf/export_mesh std_srvs/srv/Trigger"
echo ""
echo "World state:"
echo "  cat /tmp/world_state.json"
echo ""
echo "Press Ctrl+C to stop everything"
echo -e "${CYAN}============================================================${NC}"

# Keep script alive
wait "${PIDS[@]}" 2>/dev/null || true
