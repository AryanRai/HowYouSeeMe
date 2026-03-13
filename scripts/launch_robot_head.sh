#!/bin/bash
# Launch Complete Robot Head System
# Kinect v2 + BlueLily IMU + SLAM + CV Pipeline + RViz

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}=========================================${NC}"
echo -e "${CYAN}  Robot Head - Complete System Launch${NC}"
echo -e "${CYAN}=========================================${NC}"
echo ""

# Check BlueLily connection
if [ ! -e "/dev/ttyACM0" ]; then
    echo -e "${RED}❌ BlueLily not found on /dev/ttyACM0${NC}"
    echo ""
    echo "Available serial devices:"
    ls -la /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "  None found"
    echo ""
    echo "Continue without BlueLily? (y/n)"
    read -r response
    if [[ ! "$response" =~ ^[Yy]$ ]]; then
        exit 1
    fi
    BLUELILY_ENABLED=false
else
    echo -e "${GREEN}✅ BlueLily found on /dev/ttyACM0${NC}"
    # Fix permissions
    if [ ! -w "/dev/ttyACM0" ]; then
        echo "Fixing permissions..."
        sudo chmod 666 /dev/ttyACM0
    fi
    BLUELILY_ENABLED=true
fi

echo ""

# Source ROS2
echo "Sourcing ROS2 workspace..."
source ros2_ws/install/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
echo -e "${GREEN}✅ ROS2 sourced${NC}"
echo ""

# Kill existing processes
echo "Cleaning up existing processes..."
pkill -f kinect2_bridge
pkill -f sam2_server
pkill -f rtabmap
pkill -f bluelily_imu
sleep 1
echo -e "${GREEN}✅ Cleanup complete${NC}"
echo ""

echo -e "${CYAN}=========================================${NC}"
echo -e "${CYAN}  Starting Components${NC}"
echo -e "${CYAN}=========================================${NC}"
echo ""

# 1. Start BlueLily IMU (if available)
if [ "$BLUELILY_ENABLED" = true ]; then
    echo -e "${YELLOW}[1/5] Starting BlueLily IMU...${NC}"
    ros2 run bluelily_bridge bluelily_imu_node --ros-args \
        -p port:=/dev/ttyACM0 \
        -p baud_rate:=115200 \
        -p frame_id:=bluelily_imu &
    BLUELILY_PID=$!
    sleep 2
    
    # Verify IMU data
    timeout 2 ros2 topic echo /imu/data --once > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ BlueLily IMU active${NC}"
    else
        echo -e "${YELLOW}⚠️  BlueLily IMU started but no data yet${NC}"
    fi
else
    echo -e "${YELLOW}[1/5] Skipping BlueLily IMU${NC}"
fi

echo ""

# 2. Start Kinect2 Bridge
echo -e "${YELLOW}[2/5] Starting Kinect v2 Bridge...${NC}"
ros2 launch kinect2_bridge kinect2_bridge_launch.yaml > /tmp/kinect2_bridge.log 2>&1 &
KINECT_PID=$!

# Wait for Kinect to initialize
echo "Waiting for Kinect to initialize..."
sleep 5

# Check if Kinect topics are available
timeout 5 ros2 topic list | grep -q "/kinect2/hd/image_color"
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ Kinect v2 active${NC}"
else
    echo -e "${RED}❌ Kinect v2 failed to start${NC}"
    echo "Check /tmp/kinect2_bridge.log for errors"
fi

echo ""

# 3. Start RTABMap SLAM with IMU fusion
echo -e "${YELLOW}[3/5] Starting RTABMap SLAM...${NC}"

if [ "$BLUELILY_ENABLED" = true ]; then
    echo "  Using IMU fusion for enhanced localization"
    ros2 launch rtabmap_launch rtabmap.launch.py \
        rtabmap_args:="--delete_db_on_start" \
        rgb_topic:=/kinect2/hd/image_color \
        depth_topic:=/kinect2/hd/image_depth_rect \
        camera_info_topic:=/kinect2/hd/camera_info \
        frame_id:=kinect2_link \
        imu_topic:=/imu/data \
        wait_imu_to_init:=true > /tmp/rtabmap.log 2>&1 &
else
    echo "  Running without IMU"
    ros2 launch rtabmap_launch rtabmap.launch.py \
        rtabmap_args:="--delete_db_on_start" \
        rgb_topic:=/kinect2/hd/image_color \
        depth_topic:=/kinect2/hd/image_depth_rect \
        camera_info_topic:=/kinect2/hd/camera_info \
        frame_id:=kinect2_link > /tmp/rtabmap.log 2>&1 &
fi

RTABMAP_PID=$!
sleep 3
echo -e "${GREEN}✅ SLAM started${NC}"

echo ""

# 4. Start CV Pipeline Server
echo -e "${YELLOW}[4/5] Starting CV Pipeline Server...${NC}"
conda activate howyouseeme 2>/dev/null
python3 ros2_ws/src/cv_pipeline/python/sam2_server_v2.py > /tmp/cv_pipeline.log 2>&1 &
CV_PID=$!

echo "Waiting for models to load (~3 seconds)..."
sleep 3
echo -e "${GREEN}✅ CV Pipeline ready${NC}"

echo ""

# 5. Start RViz
echo -e "${YELLOW}[5/5] Starting RViz...${NC}"
ros2 run rviz2 rviz2 -d full_system_rviz.rviz > /tmp/rviz.log 2>&1 &
RVIZ_PID=$!
sleep 2
echo -e "${GREEN}✅ RViz started${NC}"

echo ""
echo -e "${CYAN}=========================================${NC}"
echo -e "${CYAN}  System Ready!${NC}"
echo -e "${CYAN}=========================================${NC}"
echo ""

if [ "$BLUELILY_ENABLED" = true ]; then
    echo -e "${GREEN}BlueLily IMU:${NC} PID $BLUELILY_PID"
    echo "  Topics: /imu/data, /bluelily/state"
fi

echo -e "${GREEN}Kinect v2:${NC} PID $KINECT_PID"
echo "  Topics: /kinect2/hd/*"
echo ""
echo -e "${GREEN}RTABMap SLAM:${NC} PID $RTABMAP_PID"
if [ "$BLUELILY_ENABLED" = true ]; then
    echo "  Mode: RGB-D + IMU Fusion"
else
    echo "  Mode: RGB-D Only"
fi
echo ""
echo -e "${GREEN}CV Pipeline:${NC} PID $CV_PID"
echo "  Models: SAM2, FastSAM, YOLO11, InsightFace, Emotion"
echo ""
echo -e "${GREEN}RViz:${NC} PID $RVIZ_PID"
echo ""

echo -e "${YELLOW}Quick Commands:${NC}"
echo "  Interactive Menu: ./cv_menu.sh"
echo "  Monitor IMU: ros2 topic hz /imu/data"
echo "  Monitor SLAM: ros2 topic hz /rtabmap/odom"
echo "  Stop All: ./kill_all.sh"
echo ""

echo -e "${CYAN}Logs:${NC}"
echo "  Kinect: /tmp/kinect2_bridge.log"
echo "  SLAM: /tmp/rtabmap.log"
echo "  CV Pipeline: /tmp/cv_pipeline.log"
echo "  RViz: /tmp/rviz.log"
echo ""

echo -e "${GREEN}Press Ctrl+C to stop all processes${NC}"
echo ""

# Wait for user interrupt
trap "echo ''; echo 'Stopping all processes...'; kill $BLUELILY_PID $KINECT_PID $RTABMAP_PID $CV_PID $RVIZ_PID 2>/dev/null; exit 0" INT

wait
