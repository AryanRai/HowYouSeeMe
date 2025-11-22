#!/bin/bash
# Start SAM2 streaming mode for continuous segmentation

# Default values
DURATION=${1:-10}  # Default 10 seconds
FPS=${2:-5}        # Default 5 FPS

echo "=========================================="
echo "  SAM2 Streaming Mode"
echo "=========================================="
echo ""
echo "Duration: ${DURATION} seconds"
echo "FPS: ${FPS} frames per second"
echo ""
echo "Starting stream..."
echo ""

# Send streaming request
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:prompt_type=point,stream=true,duration=${DURATION},fps=${FPS}'"

echo ""
echo "✅ Stream started!"
echo ""
echo "Watch in RViz: /cv_pipeline/visualization"
echo "Monitor results: ros2 topic echo /cv_pipeline/results"
echo ""
echo "To stop early:"
echo "  ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \\"
echo "    \"data: 'sam2:stop=true'\""
echo ""
echo "=========================================="
