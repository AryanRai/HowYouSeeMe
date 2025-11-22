#!/bin/bash
# Stop SAM2/CV Pipeline streaming mode

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}Stopping CV Pipeline streaming...${NC}"
echo ""

ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:stop=true'" 2>/dev/null

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ Stop command sent${NC}"
else
    echo -e "${RED}❌ Failed to send stop command${NC}"
fi
