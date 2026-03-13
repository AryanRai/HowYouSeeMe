#!/bin/bash
# Run Rerun logger in conda environment with numpy 2.x

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

# Activate conda environment
source ~/anaconda3/etc/profile.d/conda.sh
conda activate howyouseeme

# Install latest rerun if needed
pip list | grep rerun-sdk || pip install rerun-sdk

# Run the rerun logger with conda python directly from source
export PYTHONPATH=$PYTHONPATH:/opt/ros/jazzy/lib/python3.12/site-packages
cd "$WORKSPACE_ROOT"
~/anaconda3/envs/howyouseeme/bin/python \
    ros2_ws/src/kinect2_slam/kinect2_slam/rerun_logger_node.py \
    "$@"
