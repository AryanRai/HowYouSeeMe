#!/bin/bash
# Quick script to stop CV Pipeline streaming

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}  Stop CV Pipeline Streaming${NC}"
echo -e "${CYAN}========================================${NC}"
echo ""

echo "Sending stop command to CV Pipeline..."
echo ""

ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'sam2:stop=true'" 2>/dev/null

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ Stop command sent successfully!${NC}"
    echo ""
    echo "Streaming should stop within 1-2 seconds."
else
    echo -e "${RED}❌ Failed to send stop command${NC}"
    echo ""
    echo "Possible reasons:"
    echo "  - CV Pipeline server is not running"
    echo "  - ROS2 topics are not available"
    echo ""
    echo "Check if server is running:"
    echo "  pgrep -f sam2_server_v2.py"
fi

echo ""
