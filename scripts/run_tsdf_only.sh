#!/bin/bash
# Run TSDF Integrator Only
# Run this in a SEPARATE terminal after starting Phase 2

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

echo "=========================================="
echo "  TSDF Integrator (Phase 3)"
echo "=========================================="
echo ""
echo "Make sure Phase 2 (ORB-SLAM3) is already running!"
echo ""

# Check if Phase 2 is running
if ! ros2 topic list 2>/dev/null | grep -q "/orb_slam3/pose"; then
    echo "❌ Error: /orb_slam3/pose topic not found"
    echo "   Please start Phase 2 first: ./scripts/run_phase2_3.sh"
    exit 1
fi

echo "✅ ORB-SLAM3 detected, starting TSDF integrator..."
echo ""

# Source ROS2 (no conda needed - using system open3d)
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_ROOT/ros2_ws/install/setup.bash"

echo "Starting TSDF volumetric integrator..."
echo ""
echo "Parameters:"
echo "  Voxel length: 0.04m (balanced detail)"
echo "  SDF truncation: 0.08m"
echo "  Publish rate: 1 Hz"
echo ""
echo "Visualization:"
echo "  Open RViz2 and add PointCloud2 display on /tsdf/pointcloud"
echo ""
echo "Export mesh:"
echo "  ros2 service call /tsdf/export_mesh std_srvs/srv/Trigger"
echo ""
echo "Press Ctrl+C to stop"
echo "=========================================="
echo ""

# Run TSDF integrator
ros2 run kinect2_slam tsdf_integrator --ros-args \
    -p voxel_length:=0.04 \
    -p sdf_trunc:=0.08 \
    -p publish_rate:=1.0 \
    -p export_path:=/tmp/tsdf_mesh.ply \
    -p fx:=1081.37 \
    -p fy:=1081.37 \
    -p cx:=960.0 \
    -p cy:=540.0
