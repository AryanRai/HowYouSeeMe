#!/bin/bash
# Comprehensive test to verify no blocking issues

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}  Non-Blocking Streaming Test${NC}"
echo -e "${CYAN}========================================${NC}"
echo ""

# Check if server is running
if ! pgrep -f "sam2_server_v2.py" > /dev/null; then
    echo -e "${RED}❌ CV Pipeline server not running${NC}"
    echo "Please start it first:"
    echo "  ./launch_kinect_sam2_server.sh"
    exit 1
fi

echo -e "${YELLOW}This test will verify that streaming doesn't block RTAB-Map${NC}"
echo ""

# Function to check RTAB-Map delay
check_rtabmap_delay() {
    echo -e "${CYAN}Checking RTAB-Map delay...${NC}"
    # Get the last few lines of rtabmap output
    timeout 2 ros2 topic echo /rtabmap/info --once 2>/dev/null | grep -i "delay" || echo "  (Check terminal logs for delay values)"
}

echo -e "${YELLOW}Step 1: Baseline - Check RTAB-Map delay before streaming${NC}"
check_rtabmap_delay
echo ""
sleep 2

echo -e "${YELLOW}Step 2: Start YOLO streaming (10 seconds @ 5 FPS)${NC}"
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'yolo11:task=detect,conf=0.25,stream=true,duration=10,fps=5'" 2>/dev/null

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ Streaming started${NC}"
else
    echo -e "${RED}❌ Failed to start streaming${NC}"
    exit 1
fi

echo ""
echo "Waiting 3 seconds for streaming to stabilize..."
sleep 3

echo -e "${YELLOW}Step 3: Check RTAB-Map delay DURING streaming${NC}"
echo "  (Delay should still be low, < 0.3s)"
check_rtabmap_delay
echo ""

echo "Continuing streaming..."
sleep 4

echo -e "${YELLOW}Step 4: Stop streaming${NC}"
./stop_cv_streaming.sh
sleep 2

echo ""
echo -e "${YELLOW}Step 5: Check RTAB-Map delay AFTER streaming${NC}"
echo "  (Delay should return to normal, < 0.2s)"
check_rtabmap_delay
echo ""

echo -e "${YELLOW}Step 6: Test rapid model switching${NC}"
echo "Running SAM2..."
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'sam2:mode=everything'" 2>/dev/null
sleep 1

echo "Running YOLO..."
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'yolo11:task=detect,conf=0.25'" 2>/dev/null
sleep 1

echo "Running SAM2 again..."
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'sam2:mode=everything'" 2>/dev/null
sleep 1

echo -e "${GREEN}✅ Rapid switching complete${NC}"
echo ""

echo -e "${YELLOW}Step 7: Final RTAB-Map delay check${NC}"
check_rtabmap_delay
echo ""

echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}  Test Complete!${NC}"
echo -e "${CYAN}========================================${NC}"
echo ""
echo "Expected results:"
echo "  ✅ RTAB-Map delay stays low throughout"
echo "  ✅ No 'time difference' warnings"
echo "  ✅ Smooth model transitions"
echo "  ✅ No blocking behavior"
echo ""
echo "Check the cv_pipeline_server terminal for:"
echo "  - 'Still processing previous frame, skipping...' messages (normal)"
echo "  - No long pauses or freezes"
echo "  - Clean start/stop of streaming"
echo ""
