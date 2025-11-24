#!/bin/bash
# Quick test script for emotion detection

echo "========================================="
echo "  Testing Emotion Detection"
echo "========================================="
echo ""

# Check if server is running
if ! pgrep -f "sam2_server_v2.py" > /dev/null; then
    echo "❌ CV Pipeline server is not running"
    echo "Start it with: ./launch_kinect_sam2_server.sh"
    exit 1
fi

echo "✅ CV Pipeline server is running"
echo ""

# Test single frame emotion detection
echo "Testing single frame emotion detection..."
echo ""

ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'insightface:mode=emotion'" 2>/dev/null

if [ $? -eq 0 ]; then
    echo "✅ Request sent successfully!"
    echo ""
    echo "Waiting for results (5 seconds)..."
    sleep 5
    
    echo ""
    echo "Check results with:"
    echo "  ros2 topic echo /cv_pipeline/results --once"
    echo ""
    echo "Watch visualization in RViz:"
    echo "  Topic: /cv_pipeline/visualization"
else
    echo "❌ Failed to send request"
    exit 1
fi

echo ""
echo "========================================="
echo "  Test Complete!"
echo "========================================="
echo ""
echo "To test streaming:"
echo "  ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \\"
echo "    \"data: 'insightface:mode=emotion,stream=true,duration=30,fps=5'\""
echo ""
echo "Or use the menu:"
echo "  ./cv_menu.sh"
echo "  Select: 5 (Emotion Detection)"
