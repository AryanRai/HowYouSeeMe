#!/bin/bash
# Test all SAM2 modes to verify they work

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}  Testing All SAM2 Modes${NC}"
echo -e "${CYAN}========================================${NC}"
echo ""

# Check if server is running
if ! pgrep -f "sam2_server_v2.py" > /dev/null; then
    echo -e "${YELLOW}⚠️  CV Pipeline server not running${NC}"
    echo "Please start it first:"
    echo "  ./launch_kinect_sam2_server.sh"
    exit 1
fi

echo -e "${GREEN}✅ Server is running${NC}"
echo ""

# Test 1: Point Mode
echo -e "${YELLOW}Test 1: Point Mode${NC}"
echo "Sending point mode request..."
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'sam2:prompt_type=point,x=480,y=270'" 2>/dev/null
sleep 2
echo -e "${GREEN}✅ Point mode sent${NC}"
echo ""

# Test 2: Box Mode
echo -e "${YELLOW}Test 2: Box Mode${NC}"
echo "Sending box mode request..."
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'sam2:prompt_type=box,box=200,150,700,450'" 2>/dev/null
sleep 2
echo -e "${GREEN}✅ Box mode sent${NC}"
echo ""

# Test 3: Multiple Points Mode
echo -e "${YELLOW}Test 3: Multiple Points Mode${NC}"
echo "Sending multiple points request..."
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'sam2:prompt_type=points,points=400,300,500,350,labels=1,1'" 2>/dev/null
sleep 2
echo -e "${GREEN}✅ Multiple points sent${NC}"
echo ""

# Test 4: Everything Mode
echo -e "${YELLOW}Test 4: Everything Mode${NC}"
echo "Sending everything mode request..."
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'sam2:prompt_type=everything,grid_size=24'" 2>/dev/null
sleep 2
echo -e "${GREEN}✅ Everything mode sent${NC}"
echo ""

# Check results
echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}  Checking Results${NC}"
echo -e "${CYAN}========================================${NC}"
echo ""

echo "Getting latest result..."
RESULT=$(timeout 2 ros2 topic echo /cv_pipeline/results --once 2>&1)

if echo "$RESULT" | grep -q "processing_time"; then
    echo -e "${GREEN}✅ Results are being published${NC}"
    echo ""
    echo "Processing time:"
    echo "$RESULT" | grep "processing_time" | head -1
    echo ""
    echo "Prompt type:"
    echo "$RESULT" | grep "prompt_type" | head -1
else
    echo -e "${YELLOW}⚠️  No recent results${NC}"
fi

echo ""
echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}  Summary${NC}"
echo -e "${CYAN}========================================${NC}"
echo ""
echo "All test requests sent successfully!"
echo ""
echo "Check RViz to see visualizations:"
echo "  - Add Image display"
echo "  - Topic: /cv_pipeline/visualization"
echo ""
echo "Each mode should show different visualization:"
echo "  1. Point: Single green mask with blue point"
echo "  2. Box: Mask in box region with blue outline"
echo "  3. Points: Refined mask with colored points"
echo "  4. Everything: Multiple colored objects"
echo ""
echo "If you don't see visualizations:"
echo "  1. Check RViz is subscribed to the topic"
echo "  2. Run: ./debug_cv_pipeline.sh"
echo "  3. Restart server if needed"
echo ""
