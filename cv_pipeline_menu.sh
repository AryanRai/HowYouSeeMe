#!/bin/bash
# Interactive CV Pipeline Menu
# Select models, modes, and parameters

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Default values
MODEL="sam2"
MODE="point"
STREAMING=false
DURATION=10
FPS=5
X=480
Y=270
BOX="200,150,700,450"
POINTS="480,270"
LABELS="1"
GRID_SIZE=32

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
    print_header "  CV Pipeline Interactive Menu"
    echo ""
    echo -e "${GREEN}Current Configuration:${NC}"
    echo -e "  Model:     ${YELLOW}$MODEL${NC}"
    echo -e "  Mode:      ${YELLOW}$MODE${NC}"
    echo -e "  Streaming: ${YELLOW}$STREAMING${NC}"
    if [ "$STREAMING" = true ]; then
        echo -e "  Duration:  ${YELLOW}${DURATION}s${NC}"
        echo -e "  FPS:       ${YELLOW}$FPS${NC}"
    fi
    echo ""
    echo -e "${CYAN}Options:${NC}"
    echo "  1) Select Model"
    echo "  2) Select Mode"
    echo "  3) Configure Parameters"
    echo "  4) Toggle Streaming Mode"
    echo "  5) Send Request"
    echo "  6) View Results"
    echo "  7) List Available Models"
    echo "  8) Get Model Info"
    echo "  9) Stop Streaming"
    echo "  0) Exit"
    echo ""
    echo -n "Select option: "
}

# Function to select model
select_model() {
    clear
    print_header "  Select Model"
    echo ""
    echo "Available Models:"
    echo "  1) SAM2 - Segment Anything Model 2"
    echo "  2) [Future] Depth Anything"
    echo "  3) [Future] YOLO"
    echo "  4) [Future] DINO"
    echo "  0) Back"
    echo ""
    echo -n "Select model: "
    read choice
    
    case $choice in
        1)
            MODEL="sam2"
            print_success "Selected SAM2"
            sleep 1
            ;;
        2|3|4)
            print_warning "Model not yet implemented"
            sleep 2
            ;;
        0)
            return
            ;;
        *)
            print_error "Invalid option"
            sleep 1
            ;;
    esac
}

# Function to select mode
select_mode() {
    clear
    print_header "  Select Mode for $MODEL"
    echo ""
    
    if [ "$MODEL" = "sam2" ]; then
        echo "SAM2 Modes:"
        echo "  1) Point Mode - Segment object at a point"
        echo "  2) Box Mode - Segment region in bounding box"
        echo "  3) Multiple Points - Use multiple fg/bg points"
        echo "  4) Everything Mode - Segment all objects"
        echo "  0) Back"
        echo ""
        echo -n "Select mode: "
        read choice
        
        case $choice in
            1)
                MODE="point"
                print_success "Selected Point Mode"
                sleep 1
                ;;
            2)
                MODE="box"
                print_success "Selected Box Mode"
                sleep 1
                ;;
            3)
                MODE="points"
                print_success "Selected Multiple Points Mode"
                sleep 1
                ;;
            4)
                MODE="everything"
                print_success "Selected Everything Mode"
                sleep 1
                ;;
            0)
                return
                ;;
            *)
                print_error "Invalid option"
                sleep 1
                ;;
        esac
    fi
}

# Function to configure parameters
configure_parameters() {
    clear
    print_header "  Configure Parameters for $MODE Mode"
    echo ""
    
    case $MODE in
        point)
            echo "Current: X=$X, Y=$Y"
            echo ""
            echo -n "Enter X coordinate (or press Enter to keep $X): "
            read input
            [ ! -z "$input" ] && X=$input
            
            echo -n "Enter Y coordinate (or press Enter to keep $Y): "
            read input
            [ ! -z "$input" ] && Y=$input
            
            print_success "Point set to ($X, $Y)"
            sleep 1
            ;;
        
        box)
            echo "Current: $BOX (x1,y1,x2,y2)"
            echo ""
            echo -n "Enter box coordinates (x1,y1,x2,y2) or press Enter to keep: "
            read input
            [ ! -z "$input" ] && BOX=$input
            
            print_success "Box set to $BOX"
            sleep 1
            ;;
        
        points)
            echo "Current Points: $POINTS"
            echo "Current Labels: $LABELS (1=foreground, 0=background)"
            echo ""
            echo -n "Enter points (x1,y1,x2,y2,...) or press Enter to keep: "
            read input
            [ ! -z "$input" ] && POINTS=$input
            
            echo -n "Enter labels (1,1,0,...) or press Enter to keep: "
            read input
            [ ! -z "$input" ] && LABELS=$input
            
            print_success "Points configured"
            sleep 1
            ;;
        
        everything)
            echo "Current Grid Size: $GRID_SIZE"
            echo ""
            echo -n "Enter grid size (16-64) or press Enter to keep $GRID_SIZE: "
            read input
            [ ! -z "$input" ] && GRID_SIZE=$input
            
            print_success "Grid size set to $GRID_SIZE"
            sleep 1
            ;;
        
        *)
            print_error "No parameters for this mode"
            sleep 1
            ;;
    esac
}

# Function to toggle streaming
toggle_streaming() {
    clear
    print_header "  Streaming Configuration"
    echo ""
    
    if [ "$STREAMING" = true ]; then
        echo "Streaming is currently ENABLED"
        echo ""
        echo "  1) Disable Streaming"
        echo "  2) Change Duration (current: ${DURATION}s)"
        echo "  3) Change FPS (current: $FPS)"
        echo "  0) Back"
        echo ""
        echo -n "Select option: "
        read choice
        
        case $choice in
            1)
                STREAMING=false
                print_success "Streaming disabled"
                sleep 1
                ;;
            2)
                echo -n "Enter duration in seconds: "
                read input
                [ ! -z "$input" ] && DURATION=$input
                print_success "Duration set to ${DURATION}s"
                sleep 1
                ;;
            3)
                echo -n "Enter FPS (1-30): "
                read input
                [ ! -z "$input" ] && FPS=$input
                print_success "FPS set to $FPS"
                sleep 1
                ;;
            0)
                return
                ;;
        esac
    else
        echo "Streaming is currently DISABLED"
        echo ""
        echo -n "Enable streaming? (y/n): "
        read choice
        
        if [ "$choice" = "y" ] || [ "$choice" = "Y" ]; then
            STREAMING=true
            
            echo -n "Enter duration in seconds (default: 10): "
            read input
            [ ! -z "$input" ] && DURATION=$input
            
            echo -n "Enter FPS (default: 5): "
            read input
            [ ! -z "$input" ] && FPS=$input
            
            print_success "Streaming enabled: ${DURATION}s @ ${FPS} FPS"
            sleep 1
        fi
    fi
}

# Function to build and send request
send_request() {
    clear
    print_header "  Sending Request"
    echo ""
    
    # Build request string
    REQUEST="${MODEL}:prompt_type=${MODE}"
    
    # Add mode-specific parameters
    case $MODE in
        point)
            REQUEST="${REQUEST},x=${X},y=${Y}"
            ;;
        box)
            REQUEST="${REQUEST},box=${BOX}"
            ;;
        points)
            REQUEST="${REQUEST},points=${POINTS},labels=${LABELS}"
            ;;
        everything)
            REQUEST="${REQUEST},grid_size=${GRID_SIZE}"
            ;;
    esac
    
    # Add streaming parameters
    if [ "$STREAMING" = true ]; then
        REQUEST="${REQUEST},stream=true,duration=${DURATION},fps=${FPS}"
    fi
    
    echo -e "${YELLOW}Request:${NC} $REQUEST"
    echo ""
    echo "Sending to /cv_pipeline/model_request..."
    echo ""
    
    # Send request
    ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
        "data: '$REQUEST'"
    
    if [ $? -eq 0 ]; then
        print_success "Request sent successfully!"
        echo ""
        if [ "$STREAMING" = true ]; then
            print_info "Streaming started for ${DURATION}s @ ${FPS} FPS"
            print_info "Watch in RViz: /cv_pipeline/visualization"
        else
            print_info "Processing... Check /cv_pipeline/visualization in RViz"
        fi
    else
        print_error "Failed to send request"
    fi
    
    echo ""
    echo "Press Enter to continue..."
    read
}

# Function to view results
view_results() {
    clear
    print_header "  View Results"
    echo ""
    echo "Listening to /cv_pipeline/results (Ctrl+C to stop)..."
    echo ""
    
    ros2 topic echo /cv_pipeline/results
}

# Function to list models
list_models() {
    clear
    print_header "  List Available Models"
    echo ""
    echo "Sending request..."
    echo ""
    
    ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
        "data: 'sam2:list_models=true'"
    
    sleep 1
    
    echo ""
    echo "Check results with:"
    echo "  ros2 topic echo /cv_pipeline/results --once"
    echo ""
    echo "Press Enter to continue..."
    read
}

# Function to get model info
get_model_info() {
    clear
    print_header "  Get Model Info"
    echo ""
    echo "Getting info for: $MODEL"
    echo ""
    
    ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
        "data: '${MODEL}:model_info=true'"
    
    sleep 1
    
    echo ""
    echo "Check results with:"
    echo "  ros2 topic echo /cv_pipeline/results --once"
    echo ""
    echo "Press Enter to continue..."
    read
}

# Function to stop streaming
stop_streaming() {
    clear
    print_header "  Stop Streaming"
    echo ""
    echo "Sending stop command..."
    echo ""
    
    ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
        "data: 'sam2:stop=true'"
    
    if [ $? -eq 0 ]; then
        print_success "Stop command sent"
        STREAMING=false
    else
        print_error "Failed to send stop command"
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
            select_model
            ;;
        2)
            select_mode
            ;;
        3)
            configure_parameters
            ;;
        4)
            toggle_streaming
            ;;
        5)
            send_request
            ;;
        6)
            view_results
            ;;
        7)
            list_models
            ;;
        8)
            get_model_info
            ;;
        9)
            stop_streaming
            ;;
        0)
            clear
            print_success "Goodbye!"
            exit 0
            ;;
        *)
            print_error "Invalid option"
            sleep 1
            ;;
    esac
done
