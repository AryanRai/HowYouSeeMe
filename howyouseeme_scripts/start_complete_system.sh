#!/bin/bash
echo "🚀 Starting Complete HowYouSeeMe System..."
source ~/ros2_ws/install/setup.bash
export LD_LIBRARY_PATH=$HOME/libfreenect2-modern/lib:$LD_LIBRARY_PATH

# Start kinect bridge in background
echo "Starting Kinect2 Bridge..."
ros2 run kinect2_bridge kinect2_bridge_node &
KINECT_PID=$!

# Wait a bit for kinect to initialize
sleep 5

# Start HowYouSeeMe subscriber
echo "Starting HowYouSeeMe Subscriber..."
ros2 run howyouseeme_ros2 kinect_subscriber &
HOWYOUSEEME_PID=$!

# Wait for user interrupt
echo "System running. Press Ctrl+C to stop..."
trap "echo 'Stopping...'; kill $KINECT_PID $HOWYOUSEEME_PID; exit" INT
wait
