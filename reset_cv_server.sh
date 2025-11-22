#!/bin/bash
# Force reset CV Pipeline server state

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}  Force Reset CV Pipeline Server${NC}"
echo -e "${CYAN}========================================${NC}"
echo ""

echo -e "${YELLOW}This will force reset the server state if it's stuck${NC}"
echo ""

echo "Sending force reset command..."
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'sam2:reset=true'" 2>/dev/null

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ Reset command sent${NC}"
    echo ""
    echo "Server should be ready for new requests now."
    echo ""
    echo "You can also try:"
    echo "  1. Stop streaming: ./stop_cv_streaming.sh"
    echo "  2. Restart server: pkill -f sam2_server_v2.py && ./launch_kinect_sam2_server.sh"
else
    echo -e "${RED}❌ Failed to send reset command${NC}"
    echo ""
    echo "Try restarting the server:"
    echo "  pkill -f sam2_server_v2.py"
    echo "  ./launch_kinect_sam2_server.sh"
fi

echo ""
