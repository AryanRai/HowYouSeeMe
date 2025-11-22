#!/bin/bash

# Capture Kinect frame and process with SAM2 (Standalone)

echo "========================================="
echo "  Kinect Frame Capture + SAM2 Processing"
echo "========================================="
echo ""

# Source ROS2
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

# Activate conda
source /home/aryan/anaconda3/bin/activate howyouseeme

# Check if Kinect is running
echo "1. Checking Kinect status..."
if ! ros2 topic list | grep -q "/kinect2/qhd/image_color"; then
    echo "❌ Kinect not running!"
    echo "   Start it with: ros2 launch kinect2_bridge kinect2_bridge_launch.yaml"
    exit 1
fi
echo "✅ Kinect is running"

# Create output directory
mkdir -p /tmp/kinect_captures
OUTPUT_DIR="/tmp/kinect_captures"

echo ""
echo "2. Capturing RGB frame..."
timeout 3 ros2 run image_view image_saver --ros-args \
  -r image:=/kinect2/qhd/image_color \
  -p filename_format:="${OUTPUT_DIR}/rgb_%04d.jpg" 2>/dev/null &
RGB_PID=$!

sleep 2
kill $RGB_PID 2>/dev/null
wait $RGB_PID 2>/dev/null

# Find the captured file
RGB_FILE=$(ls -t ${OUTPUT_DIR}/rgb_*.jpg 2>/dev/null | head -1)

if [ -z "$RGB_FILE" ]; then
    echo "❌ Failed to capture RGB image"
    exit 1
fi

echo "✅ RGB captured: $RGB_FILE"

echo ""
echo "3. Capturing Depth frame..."
timeout 3 ros2 run image_view image_saver --ros-args \
  -r image:=/kinect2/qhd/image_depth \
  -p filename_format:="${OUTPUT_DIR}/depth_%04d.png" 2>/dev/null &
DEPTH_PID=$!

sleep 2
kill $DEPTH_PID 2>/dev/null
wait $DEPTH_PID 2>/dev/null

# Find the captured file
DEPTH_FILE=$(ls -t ${OUTPUT_DIR}/depth_*.png 2>/dev/null | head -1)

if [ -z "$DEPTH_FILE" ]; then
    echo "⚠️  No depth image captured (using dummy)"
    # Create dummy depth
    python3 << EOF
import cv2
import numpy as np
depth = np.zeros((540, 960), dtype=np.uint16)
cv2.imwrite('${OUTPUT_DIR}/depth_dummy.png', depth)
EOF
    DEPTH_FILE="${OUTPUT_DIR}/depth_dummy.png"
fi

echo "✅ Depth captured: $DEPTH_FILE"

echo ""
echo "4. Processing with SAM2 Tiny..."
echo "   (This will take ~0.3-0.7 seconds)"
echo ""

python ros2_ws/src/cv_pipeline/python/sam2_worker.py \
    --rgb "$RGB_FILE" \
    --depth "$DEPTH_FILE" \
    --params "prompt_type=point" \
    --model-size tiny

echo ""
echo "========================================="
echo "  Processing Complete!"
echo "========================================="
echo ""
echo "Captured files:"
echo "  RGB:   $RGB_FILE"
echo "  Depth: $DEPTH_FILE"
echo ""
echo "To process again:"
echo "  ./capture_and_process_kinect.sh"
echo ""
echo "To view captured image:"
echo "  eog $RGB_FILE"
