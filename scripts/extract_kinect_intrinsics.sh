#!/bin/bash

# Extract Kinect v2 intrinsics from camera_info topic
# Saves to calibration_results/kinect2_intrinsics.txt

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

OUTPUT_DIR="calibration_results"
OUTPUT_FILE="${OUTPUT_DIR}/kinect2_intrinsics.txt"

echo "=========================================="
echo "  Extract Kinect v2 Intrinsics"
echo "=========================================="
echo ""

# Check if Kinect bridge is running
if ! pgrep -f kinect2_bridge_node > /dev/null; then
    echo -e "${RED}❌ Kinect bridge not running${NC}"
    echo "Start with: ros2 launch kinect2_ros2_cuda kinect2_bridge.launch.py"
    exit 1
fi

echo -e "${GREEN}✅ Kinect bridge running${NC}"
echo ""

# Get camera info
echo -e "${YELLOW}Reading camera_info...${NC}"
CAMERA_INFO=$(timeout 5 ros2 topic echo /kinect2/hd/camera_info --once 2>/dev/null)

if [ -z "$CAMERA_INFO" ]; then
    echo -e "${RED}❌ Could not read camera_info${NC}"
    exit 1
fi

# Extract intrinsics from K matrix
# K: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
FX=$(echo "$CAMERA_INFO" | grep -A1 "^k:" | tail -1 | awk '{print $2}' | tr -d ',')
FY=$(echo "$CAMERA_INFO" | grep -A1 "^k:" | tail -1 | awk '{print $5}' | tr -d ',')
CX=$(echo "$CAMERA_INFO" | grep -A1 "^k:" | tail -1 | awk '{print $3}' | tr -d ',')
CY=$(echo "$CAMERA_INFO" | grep -A1 "^k:" | tail -1 | awk '{print $6}' | tr -d ',')

# Extract distortion
D=$(echo "$CAMERA_INFO" | grep -A1 "^d:" | tail -1)

# Extract resolution
WIDTH=$(echo "$CAMERA_INFO" | grep "^width:" | awk '{print $2}')
HEIGHT=$(echo "$CAMERA_INFO" | grep "^height:" | awk '{print $2}')

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Save to file
cat > "$OUTPUT_FILE" << EOF
# Kinect v2 HD Camera Intrinsics
# Extracted: $(date)
# Topic: /kinect2/hd/camera_info

Resolution: ${WIDTH}x${HEIGHT}

Intrinsics (K matrix):
  fx: ${FX}
  fy: ${FY}
  cx: ${CX}
  cy: ${CY}

Distortion (D):
${D}

# For Kalibr kinect2_cam.yaml:
intrinsics: [${FX}, ${FY}, ${CX}, ${CY}]
distortion_coeffs: [0.0, 0.0, 0.0, 0.0]  # Kinect pre-rectifies
resolution: [${WIDTH}, ${HEIGHT}]

# For ORB-SLAM3 config:
Camera.fx: ${FX}
Camera.fy: ${FY}
Camera.cx: ${CX}
Camera.cy: ${CY}
Camera.k1: 0.0
Camera.k2: 0.0
Camera.p1: 0.0
Camera.p2: 0.0
Camera.width: ${WIDTH}
Camera.height: ${HEIGHT}
EOF

# Display results
echo ""
echo "=========================================="
echo -e "${GREEN}Intrinsics extracted!${NC}"
echo "=========================================="
echo ""
cat "$OUTPUT_FILE"
echo ""
echo "Saved to: ${OUTPUT_FILE}"
echo ""
echo "Next steps:"
echo "1. Update kalibr_configs/kinect2_cam.yaml with these values"
echo "2. Use these values in ORB-SLAM3 config (Phase 2)"
echo ""
