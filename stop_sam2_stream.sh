#!/bin/bash
# Stop SAM2 streaming mode

echo "Stopping SAM2 stream..."

ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:stop=true'"

echo "✅ Stop command sent"
