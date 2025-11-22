#!/bin/bash

# Launch Kinect + CV Pipeline Server V2 (Multi-Model Support)

echo "========================================="
echo "  Kinect + CV Pipeline Server V2"
echo "========================================="
echo ""

# Source ROS2
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

# Activate conda for Python
export CONDA_PREFIX="/home/aryan/anaconda3/envs/howyouseeme"
export PATH="$CONDA_PREFIX/bin:$PATH"
export PYTHONPATH="$CONDA_PREFIX/lib/python3.12/site-packages:$PYTHONPATH"

# Kill any existing processes
echo "Cleaning up existing processes..."
pkill -f kinect2_bridge_node 2>/dev/null
pkill -f sam2_server.py 2>/dev/null
pkill -f rtabmap 2>/dev/null
pkill -f cv_pipeline_node 2>/dev/null
sleep 1

# Launch Kinect
echo ""
echo "Starting Kinect2 bridge..."
ros2 launch kinect2_bridge kinect2_bridge_launch.yaml &
KINECT_PID=$!

# Wait for Kinect to initialize
echo "Waiting for Kinect to initialize..."
sleep 5

# Check if Kinect topics are available
echo "Checking Kinect topics..."
if ros2 topic list | grep -q "/kinect2/qhd/image_color"; then
    echo "✅ Kinect topics found"
else
    echo "❌ Kinect topics not found!"
    exit 1
fi

# Launch CV Pipeline Server V2
echo ""
echo "Starting CV Pipeline Server V2 (loading models, please wait)..."
echo "Using extensible model manager architecture"
python3 ros2_ws/src/cv_pipeline/python/sam2_server_v2.py &
SAM2_PID=$!

# Wait for models to load
echo "Waiting for models to load (~2-3 seconds)..."
sleep 3

echo ""
echo "========================================="
echo "  System Ready!"
echo "========================================="
echo ""
echo "Kinect PID: $KINECT_PID"
echo "CV Pipeline Server PID: $SAM2_PID"
echo ""
echo "Models are PRE-LOADED and ready!"
echo ""
echo "Available Commands:"
echo ""
echo "  List models:"
echo "    ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \\"
echo "      \"data: 'sam2:list_models=true'\""
echo ""
echo "  SAM2 point mode:"
echo "    ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \\"
echo "      \"data: 'sam2:prompt_type=point'\""
echo ""
echo "  SAM2 box mode:"
echo "    ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \\"
echo "      \"data: 'sam2:prompt_type=box,box=200,150,700,450'\""
echo ""
echo "  SAM2 everything mode:"
echo "    ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \\"
echo "      \"data: 'sam2:prompt_type=everything'\""
echo ""
echo "  Start streaming:"
echo "    ./start_sam2_stream.sh 10 5"
echo ""
echo "  View all modes:"
echo "    ./sam2_modes_guide.sh"
echo ""
echo "  View results:"
echo "    ros2 topic echo /cv_pipeline/results"
echo ""
echo "Processing is FAST (~0.1-0.3s) since models are pre-loaded!"
echo ""
echo "Press Ctrl+C to stop all processes"
echo ""

# Wait for user interrupt
trap "echo 'Stopping...'; kill $KINECT_PID $SAM2_PID 2>/dev/null; exit" INT
wait
