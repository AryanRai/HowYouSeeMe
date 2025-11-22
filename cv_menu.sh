#!/bin/bash
# Quick launcher for CV Pipeline Menu with system check

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}  CV Pipeline Menu Launcher${NC}"
echo -e "${CYAN}========================================${NC}"
echo ""

# Check if CV Pipeline server is running
if pgrep -f "sam2_server_v2.py" > /dev/null; then
    echo -e "${GREEN}✅ CV Pipeline Server is running${NC}"
    echo ""
    ./cv_pipeline_menu.sh
else
    echo -e "${YELLOW}⚠️  CV Pipeline Server is not running${NC}"
    echo ""
    echo "Would you like to:"
    echo "  1) Launch Full System (Kinect + SLAM + CV Pipeline + RViz)"
    echo "  2) Launch Kinect + CV Pipeline only"
    echo "  3) Continue anyway (server might be running with different name)"
    echo "  0) Exit"
    echo ""
    echo -n "Select option: "
    read choice
    
    case $choice in
        1)
            echo ""
            echo -e "${GREEN}Launching full system...${NC}"
            echo "This will open in a new terminal. Wait for it to start, then run this script again."
            echo ""
            gnome-terminal -- bash -c "./launch_full_system_rviz.sh; exec bash" 2>/dev/null || \
            xterm -e "./launch_full_system_rviz.sh" 2>/dev/null || \
            konsole -e "./launch_full_system_rviz.sh" 2>/dev/null || \
            echo -e "${RED}Could not open terminal. Please run ./launch_full_system_rviz.sh manually${NC}"
            ;;
        2)
            echo ""
            echo -e "${GREEN}Launching Kinect + CV Pipeline...${NC}"
            echo "This will open in a new terminal. Wait for it to start, then run this script again."
            echo ""
            gnome-terminal -- bash -c "./launch_kinect_sam2_server.sh; exec bash" 2>/dev/null || \
            xterm -e "./launch_kinect_sam2_server.sh" 2>/dev/null || \
            konsole -e "./launch_kinect_sam2_server.sh" 2>/dev/null || \
            echo -e "${RED}Could not open terminal. Please run ./launch_kinect_sam2_server.sh manually${NC}"
            ;;
        3)
            echo ""
            ./cv_pipeline_menu.sh
            ;;
        0)
            echo "Exiting..."
            exit 0
            ;;
        *)
            echo -e "${RED}Invalid option${NC}"
            exit 1
            ;;
    esac
fi
