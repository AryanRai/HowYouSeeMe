#!/bin/bash

# BlueLily Bridge Verification Script
# Runs all checklist verification commands

set -e

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "=========================================="
echo "  BlueLily Bridge Verification"
echo "=========================================="
echo ""

# Check if bridge is running
echo -e "${YELLOW}[1/8] Checking if bridge is running...${NC}"
if pgrep -f bluelily_imu_node > /dev/null; then
    echo -e "${GREEN}✅ Bridge is running${NC}"
else
    echo -e "${RED}❌ Bridge is NOT running${NC}"
    echo "Start with: ros2 launch bluelily_bridge bluelily_imu.launch.py"
    exit 1
fi
echo ""

# Check topic exists
echo -e "${YELLOW}[2/8] Checking /imu/data topic exists...${NC}"
if ros2 topic list | grep -q "^/imu/data$"; then
    echo -e "${GREEN}✅ Topic /imu/data exists${NC}"
else
    echo -e "${RED}❌ Topic /imu/data NOT found${NC}"
    exit 1
fi
echo ""

# Check topic type
echo -e "${YELLOW}[3/8] Checking topic type...${NC}"
TOPIC_TYPE=$(ros2 topic type /imu/data)
if [ "$TOPIC_TYPE" = "sensor_msgs/msg/Imu" ]; then
    echo -e "${GREEN}✅ Topic type is sensor_msgs/msg/Imu${NC}"
else
    echo -e "${RED}❌ Topic type is $TOPIC_TYPE (expected sensor_msgs/msg/Imu)${NC}"
    exit 1
fi
echo ""

# Check publish rate
echo -e "${YELLOW}[4/8] Checking publish rate (5 second sample)...${NC}"
RATE=$(timeout 5 ros2 topic hz /imu/data 2>&1 | grep "average rate" | awk '{print $3}')
if [ -n "$RATE" ]; then
    RATE_INT=$(echo "$RATE" | cut -d. -f1)
    if [ "$RATE_INT" -ge 50 ] && [ "$RATE_INT" -le 850 ]; then
        echo -e "${GREEN}✅ Publish rate: ${RATE} Hz (target: 100-800 Hz)${NC}"
    else
        echo -e "${YELLOW}⚠️  Publish rate: ${RATE} Hz (expected 100-800 Hz)${NC}"
    fi
else
    echo -e "${RED}❌ Could not measure publish rate${NC}"
fi
echo ""

# Check message content
echo -e "${YELLOW}[5/8] Checking message content...${NC}"
MSG=$(timeout 2 ros2 topic echo /imu/data --once 2>/dev/null)

# Check frame_id
FRAME_ID=$(echo "$MSG" | grep "frame_id:" | awk '{print $2}' | tr -d "'")
if [ "$FRAME_ID" = "imu_link" ]; then
    echo -e "${GREEN}✅ frame_id is 'imu_link'${NC}"
else
    echo -e "${RED}❌ frame_id is '$FRAME_ID' (expected 'imu_link')${NC}"
fi

# Check timestamp is recent
STAMP_SEC=$(echo "$MSG" | grep -A1 "stamp:" | grep "sec:" | awk '{print $2}')
CURRENT_SEC=$(date +%s)
if [ -n "$STAMP_SEC" ] && [ "$STAMP_SEC" -gt 0 ]; then
    AGE=$((CURRENT_SEC - STAMP_SEC))
    if [ "$AGE" -lt 10 ]; then
        echo -e "${GREEN}✅ Timestamp is recent (${AGE}s old)${NC}"
    else
        echo -e "${YELLOW}⚠️  Timestamp is ${AGE}s old${NC}"
    fi
else
    echo -e "${RED}❌ Timestamp is zero or invalid${NC}"
fi

# Check covariances are not all zeros
ACCEL_COV=$(echo "$MSG" | grep -A1 "linear_acceleration_covariance:" | tail -1 | grep -o "[0-9.e-]*" | head -1)
GYRO_COV=$(echo "$MSG" | grep -A1 "angular_velocity_covariance:" | tail -1 | grep -o "[0-9.e-]*" | head -1)

if [ "$ACCEL_COV" != "0.0" ] && [ "$ACCEL_COV" != "0" ]; then
    echo -e "${GREEN}✅ Accelerometer covariance is set (not all zeros)${NC}"
else
    echo -e "${RED}❌ Accelerometer covariance is all zeros${NC}"
fi

if [ "$GYRO_COV" != "0.0" ] && [ "$GYRO_COV" != "0" ]; then
    echo -e "${GREEN}✅ Gyroscope covariance is set (not all zeros)${NC}"
else
    echo -e "${RED}❌ Gyroscope covariance is all zeros${NC}"
fi
echo ""

# Check TF2 chain
echo -e "${YELLOW}[6/8] Checking TF2 static transform...${NC}"
if ros2 run tf2_ros tf2_echo imu_link kinect2_link 2>&1 | grep -q "At time"; then
    echo -e "${GREEN}✅ TF2 transform imu_link → kinect2_link exists${NC}"
else
    echo -e "${RED}❌ TF2 transform imu_link → kinect2_link NOT found${NC}"
    echo "This is REQUIRED for ORB-SLAM3 IMU fusion"
fi
echo ""

# Sanity check: acceleration when stationary
echo -e "${YELLOW}[7/8] Sanity check: linear acceleration (should be ~9.81 on Z when flat)...${NC}"
ACCEL_Z=$(echo "$MSG" | grep -A3 "linear_acceleration:" | grep "z:" | awk '{print $2}')
if [ -n "$ACCEL_Z" ]; then
    ACCEL_Z_INT=$(echo "$ACCEL_Z" | cut -d. -f1 | tr -d '-')
    if [ "$ACCEL_Z_INT" -ge 8 ] && [ "$ACCEL_Z_INT" -le 11 ]; then
        echo -e "${GREEN}✅ Z acceleration: ${ACCEL_Z} m/s² (correct units)${NC}"
    elif [ "$ACCEL_Z_INT" -eq 0 ] || [ "$ACCEL_Z_INT" -eq 1 ]; then
        echo -e "${RED}❌ Z acceleration: ${ACCEL_Z} (still in g, not m/s²)${NC}"
    else
        echo -e "${YELLOW}⚠️  Z acceleration: ${ACCEL_Z} m/s² (unexpected, check sensor orientation)${NC}"
    fi
fi
echo ""

# Sanity check: gyroscope when stationary
echo -e "${YELLOW}[8/8] Sanity check: angular velocity (should be near 0 when stationary)...${NC}"
GYRO_X=$(echo "$MSG" | grep -A3 "angular_velocity:" | grep "x:" | awk '{print $2}' | tr -d '-')
GYRO_Y=$(echo "$MSG" | grep -A3 "angular_velocity:" | grep "y:" | awk '{print $2}' | tr -d '-')
GYRO_Z=$(echo "$MSG" | grep -A3 "angular_velocity:" | grep "z:" | awk '{print $2}' | tr -d '-')

if [ -n "$GYRO_X" ]; then
    GYRO_MAX=$(echo -e "$GYRO_X\n$GYRO_Y\n$GYRO_Z" | sort -n | tail -1)
    GYRO_MAX_INT=$(echo "$GYRO_MAX" | cut -d. -f1)
    
    if [ "$GYRO_MAX_INT" -lt 1 ]; then
        echo -e "${GREEN}✅ Angular velocity near zero (correct units: rad/s)${NC}"
    elif [ "$GYRO_MAX_INT" -gt 6 ]; then
        echo -e "${RED}❌ Angular velocity too high (still in deg/s, not rad/s)${NC}"
    else
        echo -e "${YELLOW}⚠️  Angular velocity: max=${GYRO_MAX} rad/s (check if sensor is moving)${NC}"
    fi
fi
echo ""

echo "=========================================="
echo -e "${GREEN}Verification complete!${NC}"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. If all checks pass, bridge is ready for Kalibr calibration"
echo "2. Run: ros2 run tf2_tools view_frames"
echo "3. Check frames.pdf for complete TF2 chain"
echo "4. Proceed to Phase 1: Kalibr calibration"
