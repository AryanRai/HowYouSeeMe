#!/bin/bash

# Record calibration bag for Kalibr IMU-camera calibration
# Usage: ./record_kalibr_bag.sh [duration_seconds]

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

DURATION=${1:-60}  # Default 60 seconds
OUTPUT_DIR="calibration_results"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BAG_NAME="kalibr_calib_${TIMESTAMP}"

echo "=========================================="
echo "  Kalibr Calibration Bag Recording"
echo "=========================================="
echo ""

# Check if bridges are running
echo -e "${YELLOW}[1/5] Checking prerequisites...${NC}"

if ! pgrep -f kinect2_bridge_node > /dev/null; then
    echo -e "${RED}❌ Kinect bridge not running${NC}"
    echo "Start with: ros2 launch kinect2_ros2_cuda kinect2_bridge.launch.py"
    exit 1
fi

if ! pgrep -f bluelily_imu_node > /dev/null; then
    echo -e "${RED}❌ BlueLily bridge not running${NC}"
    echo "Start with: ros2 launch bluelily_bridge bluelily_imu.launch.py"
    exit 1
fi

echo -e "${GREEN}✅ Both bridges running${NC}"
echo ""

# Check topic rates
echo -e "${YELLOW}[2/5] Checking topic rates...${NC}"

KINECT_RATE=$(timeout 3 ros2 topic hz /kinect2/hd/image_color 2>&1 | grep "average rate" | awk '{print $3}' | cut -d. -f1 || echo "0")
IMU_RATE=$(timeout 3 ros2 topic hz /imu/data 2>&1 | grep "average rate" | awk '{print $3}' | cut -d. -f1 || echo "0")

if [ "$KINECT_RATE" -lt 10 ]; then
    echo -e "${RED}❌ Kinect rate too low: ${KINECT_RATE} Hz${NC}"
    exit 1
fi

if [ "$IMU_RATE" -lt 50 ]; then
    echo -e "${RED}❌ IMU rate too low: ${IMU_RATE} Hz${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Kinect: ${KINECT_RATE} Hz${NC}"
echo -e "${GREEN}✅ IMU: ${IMU_RATE} Hz${NC}"
echo ""

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Instructions
echo -e "${YELLOW}[3/5] Recording instructions${NC}"
echo ""
echo "Duration: ${DURATION} seconds"
echo "Output: ${OUTPUT_DIR}/${BAG_NAME}"
echo ""
echo "Move the checkerboard (or robot head) through:"
echo "  ✅ Translation: X (left/right), Y (up/down), Z (forward/back)"
echo "  ✅ Rotation: Roll, Pitch, Yaw"
echo "  ✅ Include some FAST rotations (excite gyroscopes)"
echo "  ✅ Include some linear accelerations"
echo "  ✅ Keep board FULLY VISIBLE throughout"
echo "  ✅ Vary distance: 0.5m to 3m from camera"
echo ""
echo -e "${YELLOW}Press ENTER when ready to start recording...${NC}"
read

# Record
echo -e "${YELLOW}[4/5] Recording...${NC}"
echo ""

cd "$OUTPUT_DIR"
timeout ${DURATION} ros2 bag record \
  /kinect2/hd/image_color \
  /imu/data \
  -o "$BAG_NAME" || true

cd ..

echo ""
echo -e "${GREEN}✅ Recording complete!${NC}"
echo ""

# Verify
echo -e "${YELLOW}[5/5] Verifying recording...${NC}"

BAG_INFO=$(ros2 bag info "${OUTPUT_DIR}/${BAG_NAME}" 2>&1)

echo "$BAG_INFO"
echo ""

# Extract message counts
KINECT_MSGS=$(echo "$BAG_INFO" | grep "/kinect2/hd/image_color" | awk '{print $3}')
IMU_MSGS=$(echo "$BAG_INFO" | grep "/imu/data" | awk '{print $3}')

if [ -z "$KINECT_MSGS" ] || [ "$KINECT_MSGS" -lt 500 ]; then
    echo -e "${RED}❌ Too few Kinect messages: ${KINECT_MSGS}${NC}"
    echo "Expected: ~$((DURATION * 14)) messages"
    exit 1
fi

if [ -z "$IMU_MSGS" ] || [ "$IMU_MSGS" -lt 5000 ]; then
    echo -e "${RED}❌ Too few IMU messages: ${IMU_MSGS}${NC}"
    echo "Expected: ~$((DURATION * 100)) messages"
    exit 1
fi

echo -e "${GREEN}✅ Kinect messages: ${KINECT_MSGS}${NC}"
echo -e "${GREEN}✅ IMU messages: ${IMU_MSGS}${NC}"
echo ""

# Get file size
BAG_SIZE=$(du -sh "${OUTPUT_DIR}/${BAG_NAME}" | awk '{print $1}')
echo "Bag size: ${BAG_SIZE}"
echo ""

echo "=========================================="
echo -e "${GREEN}Recording successful!${NC}"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Convert to ROS1 bag (if using Docker Kalibr):"
echo "   rosbags-convert ${OUTPUT_DIR}/${BAG_NAME}"
echo ""
echo "2. Run Kalibr calibration:"
echo "   cd kalibr_configs"
echo "   kalibr_calibrate_imu_camera \\"
echo "     --bag ../${OUTPUT_DIR}/${BAG_NAME}_ros1/${BAG_NAME}_ros1_0.db3 \\"
echo "     --cam kinect2_cam.yaml \\"
echo "     --imu bluelily_imu.yaml \\"
echo "     --target target.yaml \\"
echo "     --show-extraction"
echo ""
echo "See: docs/PHASE1_KALIBR_CALIBRATION.md"
echo ""
