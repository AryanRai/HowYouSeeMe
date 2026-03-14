#!/bin/bash
# Run Complete System: Kinect + IMU + ORB-SLAM3 + TSDF
# After this script is running you can launch:
#   ./scripts/cv_pipeline_menu.sh   – interactive CV pipeline (all models)
#   ./scripts/run_phase4.sh         – semantic projection + TF2 world state

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

# Remove conda from PATH completely to avoid GLIBCXX conflicts
if [ -n "$CONDA_DEFAULT_ENV" ]; then
    echo "Removing conda from PATH to avoid library conflicts..."
    export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v conda | tr '\n' ':' | sed 's/:$//')
    unset CONDA_DEFAULT_ENV
    unset CONDA_PREFIX
    unset CONDA_PYTHON_EXE
    unset CONDA_SHLVL
fi

# Set library path for libfreenect2 and ORB-SLAM3
export LD_LIBRARY_PATH="$WORKSPACE_ROOT/libfreenect2/freenect2/lib:/home/aryan/ORB_SLAM3/lib:$LD_LIBRARY_PATH"

# Source ROS
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_ROOT/ros2_ws/install/setup.bash"

echo "=========================================="
echo "  Phase 2-3: ORB-SLAM3 + TSDF System"
echo "=========================================="
echo ""

# Check for BlueLily IMU
BLUELILY_ENABLED=false
BLUELILY_PORT=""

# Check common ports
for port in /dev/ttyACM0 /dev/ttyACM1 /dev/ttyACM2 /dev/ttyUSB0 /dev/ttyUSB1; do
    if [ -e "$port" ]; then
        echo "✅ BlueLily IMU detected on $port"
        # Fix permissions if needed
        if [ ! -w "$port" ]; then
            echo "   Fixing permissions..."
            sudo chmod 666 "$port" 2>/dev/null || true
        fi
        BLUELILY_ENABLED=true
        BLUELILY_PORT="$port"
        break
    fi
done

if [ "$BLUELILY_ENABLED" = false ]; then
    echo "⚠️  BlueLily IMU not found on any port"
    # Check if running non-interactively (from another script)
    if [ -t 0 ]; then
        echo "   Continue without IMU? (y/n)"
        read -r response
        if [[ ! "$response" =~ ^[Yy]$ ]]; then
            exit 1
        fi
    else
        echo "   Continuing without IMU (non-interactive mode)"
    fi
fi

# Check for Open3D (TSDF)
TSDF_ENABLED=false
if python3 -c "import open3d" 2>/dev/null; then
    echo "✅ Open3D detected — TSDF integrator will be started"
    TSDF_ENABLED=true
else
    echo "⚠️  Open3D not found — TSDF integrator disabled"
    echo "   Install with: pip install open3d"
fi

echo ""
echo "Components:"
echo "  1. Kinect v2 RGB-D Camera"
if [ "$BLUELILY_ENABLED" = true ]; then
    echo "  2. BlueLily 9-axis IMU (800 Hz)"
fi
echo "  3. ORB-SLAM3 RGB-D+IMU tracking"
if [ "$TSDF_ENABLED" = true ]; then
    echo "  4. TSDF volumetric integration (Open3D)"
fi
echo ""
echo "Next steps after this starts:"
echo "  → ./scripts/cv_pipeline_menu.sh   (interactive CV pipeline)"
echo "  → ./scripts/run_phase4.sh         (semantic projection + TF2)"
echo "=========================================="
echo ""

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "Shutting down all components..."
    kill $BLUELILY_PID $TF_PUB_PID $KINECT_PID $ORB_SLAM3_PID $TSDF_PID 2>/dev/null
    wait $BLUELILY_PID $TF_PUB_PID $KINECT_PID $ORB_SLAM3_PID $TSDF_PID 2>/dev/null
    echo "All processes stopped"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Pre-compute total step count
TOTAL=2  # Always: Kinect + ORB-SLAM3
[ "$BLUELILY_ENABLED" = true ] && TOTAL=$((TOTAL+1))
[ "$TSDF_ENABLED" = true ]     && TOTAL=$((TOTAL+1))
STEP=1

# 1. Start BlueLily IMU (if available)
if [ "$BLUELILY_ENABLED" = true ]; then
    echo "$STEP/$TOTAL Starting BlueLily IMU bridge on $BLUELILY_PORT..."
    STEP=$((STEP+1))
    ros2 run bluelily_bridge bluelily_imu_node --ros-args \
        -p port:="$BLUELILY_PORT" \
        -p baud_rate:=115200 \
        -p frame_id:=bluelily_imu &
    BLUELILY_PID=$!
    echo "    PID: $BLUELILY_PID"
    echo "    Waiting for IMU to initialize..."
    sleep 2

    # Verify IMU data
    timeout 2 ros2 topic echo /imu/data --once > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "    ✅ IMU data streaming at ~800 Hz"
    else
        echo "    ⚠️  IMU started but no data yet (will retry)"
    fi

    # Publish static TF between kinect2_link and bluelily_imu
    echo "    Publishing static TF: kinect2_link -> bluelily_imu (10cm behind)"
    ros2 run tf2_ros static_transform_publisher \
        -0.1 0 0 0 0 0 kinect2_link bluelily_imu &
    TF_PUB_PID=$!
    echo "    TF Publisher PID: $TF_PUB_PID"
    sleep 1
    echo ""
fi

# 2. Start Kinect2 bridge
echo "$STEP/$TOTAL Starting Kinect2 bridge..."
STEP=$((STEP+1))
ros2 launch kinect2_bridge kinect2_bridge_launch.yaml &
KINECT_PID=$!
echo "    PID: $KINECT_PID"
echo "    Waiting for Kinect to initialize..."
sleep 8

# Check if topics are available
echo "    Verifying Kinect topics..."
RETRY=0
MAX_RETRIES=5
while [ $RETRY -lt $MAX_RETRIES ]; do
    if ros2 topic list 2>/dev/null | grep -q "/kinect2/hd/image_color"; then
        echo "    ✅ Kinect topics available"
        break
    else
        RETRY=$((RETRY+1))
        if [ $RETRY -lt $MAX_RETRIES ]; then
            echo "    ⏳ Waiting for topics... (attempt $RETRY/$MAX_RETRIES)"
            sleep 2
        else
            echo "    ❌ Kinect topics not found after $MAX_RETRIES attempts!"
            echo "    Continuing anyway..."
        fi
    fi
done
echo ""

# 3. Start ORB-SLAM3 node
echo "$STEP/$TOTAL Starting ORB-SLAM3 RGB-D+IMU tracking..."
STEP=$((STEP+1))
ros2 run kinect2_slam orb_slam3_node --ros-args \
    -p voc_file:="$HOME/ORB_SLAM3/Vocabulary/ORBvoc.txt" \
    -p settings_file:="$WORKSPACE_ROOT/orb_slam3_configs/Kinect2_RGBD_IMU.yaml" \
    -r /camera/rgb/image_raw:=/kinect2/hd/image_color \
    -r /camera/depth_registered/image_raw:=/kinect2/hd/image_depth_rect \
    -r /imu:=/imu/data &
ORB_SLAM3_PID=$!
echo "    PID: $ORB_SLAM3_PID"
sleep 2
echo ""

# 4. Start TSDF integrator (requires Open3D)
TSDF_PID=""
if [ "$TSDF_ENABLED" = true ]; then
    echo "$STEP/$TOTAL Starting TSDF volumetric integrator..."
    ros2 run kinect2_slam tsdf_integrator --ros-args \
        -p voxel_length:=0.04 \
        -p sdf_trunc:=0.08 \
        -p publish_rate:=1.0 \
        -p export_path:=/tmp/tsdf_mesh.ply \
        -p fx:=1081.37 \
        -p fy:=1081.37 \
        -p cx:=960.0 \
        -p cy:=540.0 &
    TSDF_PID=$!
    echo "    PID: $TSDF_PID"
    echo "    Publishes /tsdf/pointcloud (dense coloured reconstruction)"
    echo "    Service  /tsdf/export_mesh  → /tmp/tsdf_mesh.ply"
    sleep 2
fi

echo ""
echo "=========================================="
echo "  🎉 Phase 2-3 Ready!"
echo "=========================================="
echo ""
echo "Process IDs:"
if [ "$BLUELILY_ENABLED" = true ]; then
    echo "  BlueLily IMU:      $BLUELILY_PID"
    echo "  TF Publisher:      $TF_PUB_PID"
fi
echo "  Kinect Bridge:     $KINECT_PID"
echo "  ORB-SLAM3:         $ORB_SLAM3_PID"
if [ -n "$TSDF_PID" ]; then
    echo "  TSDF Integrator:   $TSDF_PID"
fi
echo ""
echo "Key Topics:"
echo "  📷 RGB Image:      /kinect2/hd/image_color"
echo "  📏 Depth Image:    /kinect2/hd/image_depth_rect"
if [ "$BLUELILY_ENABLED" = true ]; then
    echo "  📊 IMU Data:       /imu/data (~800 Hz)"
fi
echo "  🎯 SLAM Pose:      /orb_slam3/pose"
if [ "$TSDF_ENABLED" = true ]; then
    echo "  🗺️  TSDF Cloud:    /tsdf/pointcloud"
fi
echo ""
echo "Next steps in separate terminals:"
echo "  → ./scripts/cv_pipeline_menu.sh   (interactive CV pipeline)"
echo "  → ./scripts/run_phase4.sh         (semantic projection + TF2)"
echo "  → ./scripts/run_full_system.sh    (all-in-one with RViz)"
echo ""
echo "Press Ctrl+C to stop all processes"
echo "=========================================="

# Wait for processes
wait $BLUELILY_PID $TF_PUB_PID $KINECT_PID $ORB_SLAM3_PID $TSDF_PID 2>/dev/null
