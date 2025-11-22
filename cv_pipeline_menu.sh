#!/bin/bash
# Interactive CV Pipeline Menu - Nested Model-Specific Menus
# Auto-executes after parameter input

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m' # No Color

# Trap Ctrl+C to offer stopping streaming
trap ctrl_c INT

ctrl_c() {
    echo ""
    echo ""
    echo -e "${YELLOW}⚠️  Ctrl+C detected${NC}"
    echo ""
    echo "What would you like to do?"
    echo "  1) Stop active streaming and return to menu"
    echo "  2) Exit menu completely"
    echo "  3) Cancel (return to menu)"
    echo ""
    echo -n "Select option: "
    read choice
    
    case $choice in
        1)
            echo ""
            echo "Stopping streaming..."
            ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
                "data: 'sam2:stop=true'" 2>/dev/null
            if [ $? -eq 0 ]; then
                echo -e "${GREEN}✅ Stop command sent${NC}"
            fi
            sleep 1
            ;;
        2)
            echo ""
            echo -e "${GREEN}Goodbye! 👋${NC}"
            exit 0
            ;;
        3)
            echo ""
            echo "Returning to menu..."
            sleep 1
            ;;
    esac
}

# Function to print colored text
print_header() {
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}$1${NC}"
    echo -e "${CYAN}========================================${NC}"
}

print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

# Function to show main menu
show_main_menu() {
    clear
    print_header "  CV Pipeline - Model Selection"
    echo ""
    echo -e "${MAGENTA}Select a Model:${NC}"
    echo ""
    echo "  1) 🎯 SAM2 - Segment Anything Model 2"
    echo "  2) ⚡ FastSAM - Faster SAM with Text Prompts"
    echo "  3) 🔍 YOLO11 - Detection, Pose, Segmentation, OBB"
    echo "  4) 📊 [Future] Depth Anything"
    echo "  5) 🧠 [Future] DINO Features"
    echo ""
    echo -e "${CYAN}System Commands:${NC}"
    echo "  8) 📋 List Available Models"
    echo "  9) 🛑 Stop Active Streaming"
    echo ""
    echo -e "${YELLOW}Tips:${NC}"
    echo "  • Press Ctrl+C anytime to stop streaming"
    echo "  • You can switch between streaming models instantly!"
    echo "  • Cooldown only affects single-frame processing"
    echo ""
    echo "  0) 🚪 Exit"
    echo ""
    echo -n "Select option: "
}

# Function to show SAM2 menu
show_sam2_menu() {
    while true; do
        clear
        print_header "  SAM2 - Segment Anything Model 2"
        echo ""
        echo -e "${MAGENTA}Select Mode:${NC}"
        echo ""
        echo "  1) 📍 Point Mode - Segment object at a point"
        echo "  2) 📦 Box Mode - Segment region in bounding box"
        echo "  3) 🎯 Multiple Points - Foreground/background points"
        echo "  4) 🌐 Everything Mode - Segment all objects"
        echo "  5) 💬 Prompt Mode - Interactive prompting"
        echo ""
        echo "  0) ⬅️  Back to Model Selection"
        echo ""
        echo -n "Select mode: "
        read choice
        
        case $choice in
            1)
                sam2_point_mode
                ;;
            2)
                sam2_box_mode
                ;;
            3)
                sam2_points_mode
                ;;
            4)
                sam2_everything_mode
                ;;
            5)
                sam2_prompt_mode
                ;;
            0)
                return
                ;;
            *)
                print_error "Invalid option"
                sleep 1
                ;;
        esac
    done
}

# SAM2 Point Mode
sam2_point_mode() {
    clear
    print_header "  SAM2 - Point Mode"
    echo ""
    echo -e "${YELLOW}Segment object at a specific point${NC}"
    echo ""
    
    # Get parameters
    echo -n "X coordinate (0-960, default 480): "
    read x
    x=${x:-480}
    
    echo -n "Y coordinate (0-540, default 270): "
    read y
    y=${y:-270}
    
    echo -n "Duration in seconds (-1 for streaming, 0 for single frame): "
    read duration
    duration=${duration:-0}
    
    if [ "$duration" -ne 0 ]; then
        echo -n "FPS (1-30, default 5): "
        read fps
        fps=${fps:-5}
    fi
    
    # Build and send request
    send_sam2_request "point" "x=$x,y=$y" "$duration" "$fps"
}

# SAM2 Box Mode
sam2_box_mode() {
    clear
    print_header "  SAM2 - Box Mode"
    echo ""
    echo -e "${YELLOW}Segment everything inside a bounding box${NC}"
    echo ""
    
    echo -n "Box coordinates x1,y1,x2,y2 (default 200,150,700,450): "
    read box
    box=${box:-200,150,700,450}
    
    echo -n "Duration in seconds (-1 for streaming, 0 for single frame): "
    read duration
    duration=${duration:-0}
    
    if [ "$duration" -ne 0 ]; then
        echo -n "FPS (1-30, default 5): "
        read fps
        fps=${fps:-5}
    fi
    
    send_sam2_request "box" "box=$box" "$duration" "$fps"
}

# SAM2 Multiple Points Mode
sam2_points_mode() {
    clear
    print_header "  SAM2 - Multiple Points Mode"
    echo ""
    echo -e "${YELLOW}Use multiple foreground/background points${NC}"
    echo ""
    
    echo -n "Points x1,y1,x2,y2,... (default 480,270): "
    read points
    points=${points:-480,270}
    
    echo -n "Labels 1,1,0,... (1=fg, 0=bg, default 1): "
    read labels
    labels=${labels:-1}
    
    echo -n "Duration in seconds (-1 for streaming, 0 for single frame): "
    read duration
    duration=${duration:-0}
    
    if [ "$duration" -ne 0 ]; then
        echo -n "FPS (1-30, default 5): "
        read fps
        fps=${fps:-5}
    fi
    
    send_sam2_request "points" "points=$points,labels=$labels" "$duration" "$fps"
}

# SAM2 Everything Mode
sam2_everything_mode() {
    clear
    print_header "  SAM2 - Everything Mode"
    echo ""
    echo -e "${YELLOW}Automatically segment all objects${NC}"
    echo ""
    
    echo -n "Grid size (16-64, default 32): "
    read grid_size
    grid_size=${grid_size:-32}
    
    echo -n "Duration in seconds (-1 for streaming, 0 for single frame): "
    read duration
    duration=${duration:-0}
    
    if [ "$duration" -ne 0 ]; then
        echo -n "FPS (1-30, default 5): "
        read fps
        fps=${fps:-5}
    fi
    
    send_sam2_request "everything" "grid_size=$grid_size" "$duration" "$fps"
}

# SAM2 Prompt Mode
sam2_prompt_mode() {
    clear
    print_header "  SAM2 - Prompt Mode"
    echo ""
    echo -e "${YELLOW}Interactive prompting with custom parameters${NC}"
    echo ""
    
    echo "Enter custom prompt parameters (e.g., prompt_type=point,x=500,y=300)"
    echo -n "Parameters: "
    read params
    
    if [ -z "$params" ]; then
        print_error "No parameters provided"
        sleep 2
        return
    fi
    
    echo -n "Duration in seconds (-1 for streaming, 0 for single frame): "
    read duration
    duration=${duration:-0}
    
    if [ "$duration" -ne 0 ]; then
        echo -n "FPS (1-30, default 5): "
        read fps
        fps=${fps:-5}
    fi
    
    send_sam2_request "custom" "$params" "$duration" "$fps"
}

# Function to show FastSAM menu
show_fastsam_menu() {
    while true; do
        clear
        print_header "  FastSAM - Fast Segment Anything"
        echo ""
        echo -e "${MAGENTA}Select Mode:${NC}"
        echo ""
        echo "  1) 📍 Point Mode - Segment object at a point"
        echo "  2) 📦 Box Mode - Segment region in bounding box"
        echo "  3) 🎯 Multiple Points - Foreground/background points"
        echo "  4) 🌐 Everything Mode - Segment all objects"
        echo "  5) 💬 Text Mode - Segment using text description"
        echo ""
        echo "  0) ⬅️  Back to Model Selection"
        echo ""
        echo -n "Select mode: "
        read choice
        
        case $choice in
            1)
                fastsam_point_mode
                ;;
            2)
                fastsam_box_mode
                ;;
            3)
                fastsam_points_mode
                ;;
            4)
                fastsam_everything_mode
                ;;
            5)
                fastsam_text_mode
                ;;
            0)
                return
                ;;
            *)
                print_error "Invalid option"
                sleep 1
                ;;
        esac
    done
}

# FastSAM Point Mode
fastsam_point_mode() {
    clear
    print_header "  FastSAM - Point Mode"
    echo ""
    echo -e "${YELLOW}Segment object at a specific point${NC}"
    echo ""
    
    echo -n "X coordinate (0-960, default 480): "
    read x
    x=${x:-480}
    
    echo -n "Y coordinate (0-540, default 270): "
    read y
    y=${y:-270}
    
    echo -n "Duration in seconds (-1 for streaming, 0 for single frame): "
    read duration
    duration=${duration:-0}
    
    if [ "$duration" -ne 0 ]; then
        echo -n "FPS (1-30, default 5): "
        read fps
        fps=${fps:-5}
    fi
    
    send_fastsam_request "point" "x=$x,y=$y" "$duration" "$fps"
}

# FastSAM Box Mode
fastsam_box_mode() {
    clear
    print_header "  FastSAM - Box Mode"
    echo ""
    echo -e "${YELLOW}Segment everything inside a bounding box${NC}"
    echo ""
    
    echo -n "Box coordinates x1,y1,x2,y2 (default 200,150,700,450): "
    read box
    box=${box:-200,150,700,450}
    
    echo -n "Duration in seconds (-1 for streaming, 0 for single frame): "
    read duration
    duration=${duration:-0}
    
    if [ "$duration" -ne 0 ]; then
        echo -n "FPS (1-30, default 5): "
        read fps
        fps=${fps:-5}
    fi
    
    send_fastsam_request "box" "box=$box" "$duration" "$fps"
}

# FastSAM Multiple Points Mode
fastsam_points_mode() {
    clear
    print_header "  FastSAM - Multiple Points Mode"
    echo ""
    echo -e "${YELLOW}Use multiple foreground/background points${NC}"
    echo ""
    
    echo -n "Points x1,y1,x2,y2,... (default 480,270): "
    read points
    points=${points:-480,270}
    
    echo -n "Labels 1,1,0,... (1=fg, 0=bg, default 1): "
    read labels
    labels=${labels:-1}
    
    echo -n "Duration in seconds (-1 for streaming, 0 for single frame): "
    read duration
    duration=${duration:-0}
    
    if [ "$duration" -ne 0 ]; then
        echo -n "FPS (1-30, default 5): "
        read fps
        fps=${fps:-5}
    fi
    
    send_fastsam_request "points" "points=$points,labels=$labels" "$duration" "$fps"
}

# FastSAM Everything Mode
fastsam_everything_mode() {
    clear
    print_header "  FastSAM - Everything Mode"
    echo ""
    echo -e "${YELLOW}Automatically segment all objects${NC}"
    echo ""
    
    echo -n "Duration in seconds (-1 for streaming, 0 for single frame): "
    read duration
    duration=${duration:-0}
    
    if [ "$duration" -ne 0 ]; then
        echo -n "FPS (1-30, default 5): "
        read fps
        fps=${fps:-5}
    fi
    
    send_fastsam_request "everything" "" "$duration" "$fps"
}

# FastSAM Text Mode (NEW!)
fastsam_text_mode() {
    clear
    print_header "  FastSAM - Text Mode"
    echo ""
    echo -e "${YELLOW}Segment using natural language description${NC}"
    echo ""
    echo "Examples:"
    echo "  - 'a photo of a dog'"
    echo "  - 'the yellow car'"
    echo "  - 'person wearing red shirt'"
    echo ""
    
    echo -n "Enter text description: "
    read text
    
    if [ -z "$text" ]; then
        print_error "No text provided"
        sleep 2
        return
    fi
    
    echo -n "Duration in seconds (-1 for streaming, 0 for single frame): "
    read duration
    duration=${duration:-0}
    
    if [ "$duration" -ne 0 ]; then
        echo -n "FPS (1-30, default 5): "
        read fps
        fps=${fps:-5}
    fi
    
    send_fastsam_request "text" "text=$text" "$duration" "$fps"
}

# Function to send FastSAM request
send_fastsam_request() {
    local mode=$1
    local params=$2
    local duration=$3
    local fps=$4
    
    echo ""
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    
    # Build request
    local request="fastsam:"
    
    if [ -z "$params" ]; then
        request="${request}prompt_type=${mode}"
    else
        request="${request}prompt_type=${mode},${params}"
    fi
    
    # Add streaming parameters
    if [ "$duration" -eq -1 ]; then
        request="${request},stream=true,duration=999999,fps=${fps}"
        echo -e "${MAGENTA}🔄 Starting CONTINUOUS streaming @ ${fps} FPS${NC}"
        echo -e "${YELLOW}   Use option 9 to stop${NC}"
    elif [ "$duration" -gt 0 ]; then
        request="${request},stream=true,duration=${duration},fps=${fps}"
        echo -e "${MAGENTA}🎬 Starting streaming: ${duration}s @ ${fps} FPS${NC}"
    else
        echo -e "${GREEN}📸 Processing single frame${NC}"
    fi
    
    echo ""
    echo -e "${BLUE}Request:${NC} $request"
    echo ""
    
    # Send request
    ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
        "data: '$request'" 2>/dev/null
    
    if [ $? -eq 0 ]; then
        print_success "Request sent successfully!"
        echo ""
        print_info "Watch results in RViz: /cv_pipeline/visualization"
        
        if [ "$duration" -ne 0 ]; then
            print_info "Monitor: ros2 topic echo /cv_pipeline/results"
        fi
    else
        print_error "Failed to send request"
        print_warning "Is the CV Pipeline server running?"
    fi
    
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
    echo "Press Enter to continue..."
    read
}

# Function to show YOLO11 menu
show_yolo11_menu() {
    while true; do
        clear
        print_header "  YOLO11 - Multi-Task Vision Model"
        echo ""
        echo -e "${MAGENTA}Select Task:${NC}"
        echo ""
        echo "  1) 🔍 Detection - Detect objects with bounding boxes"
        echo "  2) 🎭 Segmentation - Instance segmentation masks"
        echo "  3) 🧍 Pose Estimation - Human pose keypoints"
        echo "  4) 📐 OBB - Oriented bounding boxes"
        echo ""
        echo "  0) ⬅️  Back to Model Selection"
        echo ""
        echo -n "Select task: "
        read choice
        
        case $choice in
            1)
                yolo11_detection
                ;;
            2)
                yolo11_segmentation
                ;;
            3)
                yolo11_pose
                ;;
            4)
                yolo11_obb
                ;;
            0)
                return
                ;;
            *)
                print_error "Invalid option"
                sleep 1
                ;;
        esac
    done
}

# YOLO11 Detection
yolo11_detection() {
    clear
    print_header "  YOLO11 - Object Detection"
    echo ""
    echo -e "${YELLOW}Detect objects with bounding boxes${NC}"
    echo ""
    
    echo -n "Confidence threshold (0.0-1.0, default 0.25): "
    read conf
    conf=${conf:-0.25}
    
    echo -n "IOU threshold (0.0-1.0, default 0.7): "
    read iou
    iou=${iou:-0.7}
    
    echo -n "Duration in seconds (-1 for streaming, 0 for single frame): "
    read duration
    duration=${duration:-0}
    
    if [ "$duration" -ne 0 ]; then
        echo -n "FPS (1-30, default 10): "
        read fps
        fps=${fps:-10}
    fi
    
    send_yolo11_request "detect" "conf=$conf,iou=$iou" "$duration" "$fps"
}

# YOLO11 Segmentation
yolo11_segmentation() {
    clear
    print_header "  YOLO11 - Instance Segmentation"
    echo ""
    echo -e "${YELLOW}Segment objects with masks${NC}"
    echo ""
    
    echo -n "Confidence threshold (0.0-1.0, default 0.25): "
    read conf
    conf=${conf:-0.25}
    
    echo -n "IOU threshold (0.0-1.0, default 0.7): "
    read iou
    iou=${iou:-0.7}
    
    echo -n "Duration in seconds (-1 for streaming, 0 for single frame): "
    read duration
    duration=${duration:-0}
    
    if [ "$duration" -ne 0 ]; then
        echo -n "FPS (1-30, default 10): "
        read fps
        fps=${fps:-10}
    fi
    
    send_yolo11_request "segment" "conf=$conf,iou=$iou" "$duration" "$fps"
}

# YOLO11 Pose Estimation
yolo11_pose() {
    clear
    print_header "  YOLO11 - Pose Estimation"
    echo ""
    echo -e "${YELLOW}Detect human poses with keypoints${NC}"
    echo ""
    
    echo -n "Confidence threshold (0.0-1.0, default 0.25): "
    read conf
    conf=${conf:-0.25}
    
    echo -n "IOU threshold (0.0-1.0, default 0.7): "
    read iou
    iou=${iou:-0.7}
    
    echo -n "Duration in seconds (-1 for streaming, 0 for single frame): "
    read duration
    duration=${duration:-0}
    
    if [ "$duration" -ne 0 ]; then
        echo -n "FPS (1-30, default 10): "
        read fps
        fps=${fps:-10}
    fi
    
    send_yolo11_request "pose" "conf=$conf,iou=$iou" "$duration" "$fps"
}

# YOLO11 OBB
yolo11_obb() {
    clear
    print_header "  YOLO11 - Oriented Bounding Boxes"
    echo ""
    echo -e "${YELLOW}Detect objects with rotated boxes${NC}"
    echo ""
    
    echo -n "Confidence threshold (0.0-1.0, default 0.25): "
    read conf
    conf=${conf:-0.25}
    
    echo -n "IOU threshold (0.0-1.0, default 0.7): "
    read iou
    iou=${iou:-0.7}
    
    echo -n "Duration in seconds (-1 for streaming, 0 for single frame): "
    read duration
    duration=${duration:-0}
    
    if [ "$duration" -ne 0 ]; then
        echo -n "FPS (1-30, default 10): "
        read fps
        fps=${fps:-10}
    fi
    
    send_yolo11_request "obb" "conf=$conf,iou=$iou" "$duration" "$fps"
}

# Function to send YOLO11 request
send_yolo11_request() {
    local task=$1
    local params=$2
    local duration=$3
    local fps=$4
    
    echo ""
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    
    # Build request
    local request="yolo11:task=${task},${params}"
    
    # Add streaming parameters
    if [ "$duration" -eq -1 ]; then
        request="${request},stream=true,duration=999999,fps=${fps}"
        echo -e "${MAGENTA}🔄 Starting CONTINUOUS streaming @ ${fps} FPS${NC}"
        echo -e "${YELLOW}   Use option 9 to stop${NC}"
    elif [ "$duration" -gt 0 ]; then
        request="${request},stream=true,duration=${duration},fps=${fps}"
        echo -e "${MAGENTA}🎬 Starting streaming: ${duration}s @ ${fps} FPS${NC}"
    else
        echo -e "${GREEN}📸 Processing single frame${NC}"
    fi
    
    echo ""
    echo -e "${BLUE}Request:${NC} $request"
    echo ""
    
    # Send request
    ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
        "data: '$request'" 2>/dev/null
    
    if [ $? -eq 0 ]; then
        print_success "Request sent successfully!"
        echo ""
        print_info "Watch results in RViz: /cv_pipeline/visualization"
        
        if [ "$duration" -ne 0 ]; then
            print_info "Monitor: ros2 topic echo /cv_pipeline/results"
        fi
    else
        print_error "Failed to send request"
        print_warning "Is the CV Pipeline server running?"
    fi
    
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
    echo "Press Enter to continue..."
    read
}

# Function to send SAM2 request
send_sam2_request() {
    local mode=$1
    local params=$2
    local duration=$3
    local fps=$4
    
    echo ""
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    
    # Build request
    local request="sam2:"
    
    if [ "$mode" = "custom" ]; then
        request="${request}${params}"
    else
        request="${request}prompt_type=${mode},${params}"
    fi
    
    # Add streaming parameters
    if [ "$duration" -eq -1 ]; then
        request="${request},stream=true,duration=999999,fps=${fps}"
        echo -e "${MAGENTA}🔄 Starting CONTINUOUS streaming @ ${fps} FPS${NC}"
        echo -e "${YELLOW}   Use option 9 to stop${NC}"
    elif [ "$duration" -gt 0 ]; then
        request="${request},stream=true,duration=${duration},fps=${fps}"
        echo -e "${MAGENTA}🎬 Starting streaming: ${duration}s @ ${fps} FPS${NC}"
    else
        echo -e "${GREEN}📸 Processing single frame${NC}"
    fi
    
    echo ""
    echo -e "${BLUE}Request:${NC} $request"
    echo ""
    
    # Send request
    ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
        "data: '$request'" 2>/dev/null
    
    if [ $? -eq 0 ]; then
        print_success "Request sent successfully!"
        echo ""
        print_info "Watch results in RViz: /cv_pipeline/visualization"
        
        if [ "$duration" -ne 0 ]; then
            print_info "Monitor: ros2 topic echo /cv_pipeline/results"
        fi
    else
        print_error "Failed to send request"
        print_warning "Is the CV Pipeline server running?"
    fi
    
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
    echo "Press Enter to continue..."
    read
}







# Function to list models
list_models() {
    clear
    print_header "  List Available Models"
    echo ""
    echo "Querying available models..."
    echo ""
    
    ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
        "data: 'sam2:list_models=true'" 2>/dev/null
    
    sleep 1
    
    echo ""
    print_info "Check results: ros2 topic echo /cv_pipeline/results --once"
    echo ""
    echo "Press Enter to continue..."
    read
}

# Function to stop streaming
stop_streaming() {
    clear
    print_header "  Stop Active Streaming"
    echo ""
    echo -e "${YELLOW}This will stop any active streaming session.${NC}"
    echo ""
    echo "Sending stop command..."
    echo ""
    
    ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
        "data: 'sam2:stop=true'" 2>/dev/null
    
    if [ $? -eq 0 ]; then
        print_success "Stop command sent successfully!"
        echo ""
        print_info "Server entering 0.5s cooldown period..."
        echo "This allows the image pipeline to stabilize."
        echo ""
        sleep 0.6
        print_success "Cooldown complete - ready for new requests"
        echo ""
        print_info "Note: You can start streaming again immediately!"
        echo "Cooldown only affects single-frame processing."
    else
        print_error "Failed to send stop command"
        echo ""
        print_warning "Possible reasons:"
        echo "  - CV Pipeline server is not running"
        echo "  - No active streaming session"
        echo "  - ROS2 communication issue"
    fi
    
    echo ""
    echo "Press Enter to continue..."
    read
}

# Main loop
while true; do
    show_main_menu
    read choice
    
    case $choice in
        1)
            show_sam2_menu
            ;;
        2)
            show_fastsam_menu
            ;;
        3)
            show_yolo11_menu
            ;;
        4|5)
            clear
            print_warning "Model not yet implemented"
            echo ""
            echo "Coming soon:"
            echo "  - Depth Anything: Depth estimation"
            echo "  - DINO: Feature extraction"
            echo ""
            echo "Press Enter to continue..."
            read
            ;;
        8)
            list_models
            ;;
        9)
            stop_streaming
            ;;
        0)
            clear
            print_success "Goodbye! 👋"
            exit 0
            ;;
        *)
            print_error "Invalid option"
            sleep 1
            ;;
    esac
done
