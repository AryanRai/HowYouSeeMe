#!/bin/bash
# Debug CV Pipeline Issues

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}  CV Pipeline Diagnostics${NC}"
echo -e "${CYAN}========================================${NC}"
echo ""

# Check if server is running
echo -e "${YELLOW}1. Checking CV Pipeline Server...${NC}"
if pgrep -f "sam2_server_v2.py" > /dev/null; then
    PID=$(pgrep -f "sam2_server_v2.py")
    echo -e "${GREEN}✅ Server is running (PID: $PID)${NC}"
else
    echo -e "${RED}❌ Server is NOT running${NC}"
    echo "   Start with: ./launch_kinect_sam2_server.sh"
fi
echo ""

# Check ROS2 topics
echo -e "${YELLOW}2. Checking ROS2 Topics...${NC}"
if ros2 topic list 2>/dev/null | grep -q "cv_pipeline"; then
    echo -e "${GREEN}✅ CV Pipeline topics found:${NC}"
    ros2 topic list | grep cv_pipeline | sed 's/^/   /'
else
    echo -e "${RED}❌ No CV Pipeline topics found${NC}"
fi
echo ""

# Check topic rates
echo -e "${YELLOW}3. Checking Topic Activity...${NC}"
echo "   Checking /cv_pipeline/visualization (5 second sample)..."
RATE=$(timeout 5 ros2 topic hz /cv_pipeline/visualization 2>&1 | grep "average rate" | awk '{print $3}')
if [ ! -z "$RATE" ]; then
    echo -e "${GREEN}   ✅ Publishing at ${RATE} Hz${NC}"
else
    echo -e "${YELLOW}   ⚠️  No messages (might be idle)${NC}"
fi
echo ""

# Check latest result
echo -e "${YELLOW}4. Checking Latest Result...${NC}"
RESULT=$(timeout 2 ros2 topic echo /cv_pipeline/results --once 2>&1)
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ Latest result received${NC}"
    echo "$RESULT" | head -10 | sed 's/^/   /'
else
    echo -e "${YELLOW}⚠️  No recent results (might be idle)${NC}"
fi
echo ""

# Check Kinect topics
echo -e "${YELLOW}5. Checking Kinect Topics...${NC}"
if ros2 topic list 2>/dev/null | grep -q "kinect2"; then
    echo -e "${GREEN}✅ Kinect topics found${NC}"
    KINECT_RATE=$(timeout 3 ros2 topic hz /kinect2/qhd/image_color 2>&1 | grep "average rate" | awk '{print $3}')
    if [ ! -z "$KINECT_RATE" ]; then
        echo -e "${GREEN}   Publishing at ${KINECT_RATE} Hz${NC}"
    fi
else
    echo -e "${RED}❌ No Kinect topics found${NC}"
    echo "   Start with: ./launch_kinect_sam2_server.sh"
fi
echo ""

# Test single request
echo -e "${YELLOW}6. Testing Single Frame Request...${NC}"
echo "   Sending test request..."
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'sam2:prompt_type=point'" 2>/dev/null

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ Request sent${NC}"
    echo "   Waiting for result (3 seconds)..."
    sleep 3
    
    RESULT=$(timeout 1 ros2 topic echo /cv_pipeline/results --once 2>&1)
    if echo "$RESULT" | grep -q "processing_time"; then
        echo -e "${GREEN}✅ Result received!${NC}"
        PROC_TIME=$(echo "$RESULT" | grep "processing_time" | awk -F': ' '{print $2}' | tr -d ',')
        echo "   Processing time: ${PROC_TIME}s"
    else
        echo -e "${RED}❌ No result received${NC}"
    fi
else
    echo -e "${RED}❌ Failed to send request${NC}"
fi
echo ""

# Check for errors in logs
echo -e "${YELLOW}7. Checking for Recent Errors...${NC}"
if pgrep -f "sam2_server_v2.py" > /dev/null; then
    echo "   (Check terminal running the server for detailed logs)"
    echo "   Common issues:"
    echo "   - 'No RGB image available' = Kinect not publishing"
    echo "   - 'Model not loaded' = SAM2 failed to load"
    echo "   - 'Visualization error' = Image processing issue"
else
    echo -e "${RED}   Server not running, no logs to check${NC}"
fi
echo ""

# Summary
echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}  Summary${NC}"
echo -e "${CYAN}========================================${NC}"
echo ""
echo "If visualization stops working after streaming:"
echo "  1. Check server logs for errors"
echo "  2. Try stopping streaming: ./stop_cv_streaming.sh"
echo "  3. Send a single frame request"
echo "  4. If still broken, restart server:"
echo "     pkill -f sam2_server_v2.py"
echo "     ./launch_kinect_sam2_server.sh"
echo ""
echo "For detailed debugging, run server with:"
echo "  python3 ros2_ws/src/cv_pipeline/python/sam2_server_v2.py"
echo "  (Shows all debug messages)"
echo ""
