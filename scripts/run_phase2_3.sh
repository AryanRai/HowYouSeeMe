#!/bin/bash
# Run Complete System: Kinect + IMU + ORB-SLAM3 + TSDF

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

# Set library path for libfreenect2
export LD_LIBRARY_PATH="$WORKSPACE_ROOT/libfreenect2/freenect2/lib:$LD_LIBRARY_PATH"

# Source ROS
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_ROOT/ros2_ws/install/setup.bash"

echo "=========================================="
echo "  Phase 2-3: ORB-SLAM3 + TSDF System"
echo "=========================================="
echo ""

# Check for BlueLily IMU
BLUELILY_ENABLED=false
if [ -e "/dev/ttyACM0" ]; then
    echo "✅ BlueLily IMU detected on /dev/ttyACM0"
    # Fix permissions if needed
    if [ ! -w "/dev/ttyACM0" ]; then
        echo "   Fixing permissions..."
        sudo chmod 666 /dev/ttyACM0
    fi
    BLUELILY_ENABLED=true
else
    echo "⚠️  BlueLily IMU not found on /dev/ttyACM0"
    echo "   Continue without IMU? (y/n)"
    read -r response
    if [[ ! "$response" =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo ""
echo "Components:"
echo "  1. Kinect v2 RGB-D Camera"
if [ "$BLUELILY_ENABLED" = true ]; then
    echo "  2. BlueLily 9-axis IMU (800 Hz)"
fi
echo "  3. ORB-SLAM3 RGB-D+IMU tracking"
echo "  4. TSDF volumetric integration"
echo ""
echo "=========================================="
echo ""

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "Shutting down all components..."
    if [ "$BLUELILY_ENABLED" = true ]; then
        kill $BLUELILY_PID $TF_PUB_PID $KINECT_PID $ORB_SLAM3_PID 2>/dev/null
        wait $BLUELILY_PID $TF_PUB_PID $KINECT_PID $ORB_SLAM3_PID 2>/dev/null
    else
        kill $KINECT_PID $ORB_SLAM3_PID 2>/dev/null
        wait $KINECT_PID $ORB_SLAM3_PID 2>/dev/null
    fi
    echo "All processes stopped"
    exit 0
}

trap cleanup SIGINT SIGTERM

# 1. Start BlueLily IMU (if available)
if [ "$BLUELILY_ENABLED" = true ]; then
    echo "1/4 Starting BlueLily IMU bridge..."
    ros2 run bluelily_bridge bluelily_imu_node --ros-args \
        -p port:=/dev/ttyACM0 \
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
if [ "$BLUELILY_ENABLED" = true ]; then
    echo "2/4 Starting Kinect2 bridge..."
else
    echo "1/3 Starting Kinect2 bridge..."
fi
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
if [ "$BLUELILY_ENABLED" = true ]; then
    echo "3/4 Starting ORB-SLAM3 RGB-D+IMU tracking..."
else
    echo "2/3 Starting ORB-SLAM3 RGB-D+IMU tracking..."
fi
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

# 4. Start TSDF integrator (DISABLED - requires conda's open3d)
# if [ "$BLUELILY_ENABLED" = true ]; then
#     echo "4/4 Starting TSDF volumetric integrator..."
# else
#     echo "3/3 Starting TSDF volumetric integrator..."
# fi
# ros2 run kinect2_slam tsdf_integrator --ros-args \
#     -p voxel_length:=0.04 \
#     -p sdf_trunc:=0.08 \
#     -p publish_rate:=1.0 \
#     -p export_path:=/tmp/tsdf_mesh.ply \
#     -p fx:=1081.37 \
#     -p fy:=1081.37 \
#     -p cx:=960.0 \
#     -p cy:=540.0 &
# TSDF_PID=$!
# echo "    PID: $TSDF_PID"
# sleep 2

echo ""
echo "Note: TSDF integrator disabled (requires conda environment)"
echo "      ORB-SLAM3 provides pose estimation without dense reconstruction"
TSDF_PID=""

echo ""
echo "=========================================="
echo "  🎉 System Ready!"
echo "=========================================="
echo ""
echo "Process IDs:"
if [ "$BLUELILY_ENABLED" = true ]; then
    echo "  BlueLily IMU:      $BLUELILY_PID"
    echo "  TF Publisher:      $TF_PUB_PID"
fi
echo "  Kinect Bridge:     $KINECT_PID"
echo "  ORB-SLAM3:         $ORB_SLAM3_PID"
echo ""
echo "Key Topics:"
echo "  📷 RGB Image:      /kinect2/hd/image_color"
echo "  📏 Depth Image:    /kinect2/hd/image_depth_rect"
if [ "$BLUELILY_ENABLED" = true ]; then
    echo "  📊 IMU Data:       /imu/data (~800 Hz)"
fi
echo "  🎯 SLAM Pose:      /orb_slam3/pose"
echo ""
echo "ORB-SLAM3 Viewer:"
echo "  - Pangolin window shows camera trajectory and map points"
echo "  - Green = tracked features, Red = lost tracking"
echo ""
echo "Press Ctrl+C to stop all processes"
echo "=========================================="

# Wait for processes
if [ "$BLUELILY_ENABLED" = true ]; then
    wait $BLUELILY_PID $TF_PUB_PID $KINECT_PID $ORB_SLAM3_PID
else
    wait $KINECT_PID $ORB_SLAM3_PID
fi
