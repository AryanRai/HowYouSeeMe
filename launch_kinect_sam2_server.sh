#!/bin/bash

# Launch Kinect + SAM2 Server (Model Pre-loaded)

echo "========================================="
echo "  Kinect + SAM2 Server"
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

# Launch SAM2 Server
echo ""
echo "Starting SAM2 Server (loading model, please wait)..."
python3 ros2_ws/src/cv_pipeline/python/sam2_server.py &
SAM2_PID=$!

# Wait for SAM2 to load
echo "Waiting for SAM2 model to load (~2-3 seconds)..."
sleep 3

echo ""
echo "========================================="
echo "  System Ready!"
echo "========================================="
echo ""
echo "Kinect PID: $KINECT_PID"
echo "SAM2 Server PID: $SAM2_PID"
echo ""
echo "SAM2 model is PRE-LOADED and ready!"
echo ""
echo "To send a request:"
echo "  ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \"data: 'sam2:prompt_type=point'\""
echo ""
echo "To view results:"
echo "  ros2 topic echo /cv_pipeline/results"
echo ""
echo "Processing should be FAST (~0.3-0.5s) since model is already loaded!"
echo ""
echo "Press Ctrl+C to stop all processes"
echo ""

# Wait for user interrupt
trap "echo 'Stopping...'; kill $KINECT_PID $SAM2_PID 2>/dev/null; exit" INT
wait
