#!/bin/bash
# Test the streaming fix

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}  Testing Streaming Fix${NC}"
echo -e "${CYAN}========================================${NC}"
echo ""

echo -e "${YELLOW}Test 1: Start YOLO streaming for 5 seconds${NC}"
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'yolo11:task=detect,conf=0.25,stream=true,duration=5,fps=5'" 2>/dev/null

echo "Streaming for 5 seconds..."
sleep 6

echo ""
echo -e "${YELLOW}Test 2: Check if we can run another model immediately${NC}"
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'sam2:mode=everything'" 2>/dev/null

echo "Waiting 2 seconds..."
sleep 2

echo ""
echo -e "${YELLOW}Test 3: Start another YOLO stream${NC}"
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'yolo11:task=pose,conf=0.25,stream=true,duration=3,fps=5'" 2>/dev/null

echo "Streaming for 3 seconds..."
sleep 4

echo ""
echo -e "${GREEN}✅ Test complete!${NC}"
echo ""
echo "Check the cv_pipeline_server logs to verify:"
echo "  1. No blocking during streaming"
echo "  2. Clean transitions between models"
echo "  3. RTAB-Map stays in sync"
