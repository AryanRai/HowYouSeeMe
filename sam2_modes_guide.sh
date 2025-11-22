#!/bin/bash
# SAM2 Prompting Modes Guide

cat << 'EOF'
========================================
  SAM2 Prompting Modes Guide
========================================

SAM2 supports multiple prompting modes:

1. POINT MODE (default)
   Segment object at a specific point
   
   # Center point (default)
   ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
     "data: 'sam2:prompt_type=point'"
   
   # Custom point (x=500, y=300)
   ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
     "data: 'sam2:prompt_type=point,x=500,y=300'"

2. BOX MODE
   Segment everything inside a bounding box
   
   # Full image
   ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
     "data: 'sam2:prompt_type=box,box=0,0,960,540'"
   
   # Custom box (x1,y1,x2,y2)
   ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
     "data: 'sam2:prompt_type=box,box=200,150,700,450'"

3. MULTIPLE POINTS MODE
   Use multiple points with foreground/background labels
   
   # Two foreground points
   ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
     "data: 'sam2:prompt_type=points,points=400,300,500,350,labels=1,1'"
   
   # Foreground + background (1=fg, 0=bg)
   ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
     "data: 'sam2:prompt_type=points,points=480,270,200,100,labels=1,0'"

4. EVERYTHING MODE
   Segment all objects in the frame (automatic)
   
   # Default grid
   ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
     "data: 'sam2:prompt_type=everything'"
   
   # Custom grid size
   ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
     "data: 'sam2:prompt_type=everything,grid_size=16'"

5. STREAMING MODE
   Continuous segmentation for video
   
   # Stream for 10 seconds at 5 FPS
   ./start_sam2_stream.sh 10 5
   
   # With custom prompt
   ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
     "data: 'sam2:prompt_type=box,box=200,150,700,450,stream=true,duration=15,fps=3'"

========================================
VISUALIZATION

Watch results in RViz:
  - Topic: /cv_pipeline/visualization
  - Shows: masks, prompts, scores

Monitor JSON results:
  ros2 topic echo /cv_pipeline/results

========================================
EXAMPLES

# Segment object at mouse click position
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:prompt_type=point,x=640,y=360'"

# Segment region of interest
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:prompt_type=box,box=300,200,600,400'"

# Refine with multiple points
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:prompt_type=points,points=480,270,450,250,500,300,labels=1,1,1'"

# Segment everything (like SAM's automatic mode)
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:prompt_type=everything,grid_size=24'"

========================================
EOF
