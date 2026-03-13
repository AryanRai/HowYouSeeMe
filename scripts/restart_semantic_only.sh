#!/bin/bash
# Restart only the semantic projection node (useful after code changes)

echo "Stopping semantic projection..."
pkill -f semantic_projection

sleep 2

echo "Starting semantic projection..."
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

ros2 run cv_pipeline semantic_projection --ros-args \
    -p fx:=1081.37 \
    -p fy:=1081.37 \
    -p cx:=959.5 \
    -p cy:=539.5 \
    -p world_state_path:=/tmp/world_state.json \
    -p marker_lifetime:=30.0 \
    -p conf_threshold:=0.4 \
    -p depth_trunc:=5.0 \
    -p merge_threshold:=0.3 \
    -p flip_x_axis:=true \
    -p flip_y_axis:=true \
    -p debug_projection:=false

