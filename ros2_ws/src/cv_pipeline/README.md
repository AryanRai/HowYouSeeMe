# CV Pipeline - Modular Computer Vision for Kinect

Lazy-loaded computer vision models for real-time RGB-D processing.

## Quick Start

```bash
# 1. Build
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select cv_pipeline
source install/setup.bash

# 2. Launch (with Kinect)
./launch_kinect_cv_pipeline.sh

# 3. Send request
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam3:prompt=person'"

# 4. View results
ros2 topic echo /cv_pipeline/results
```

## Models

- ✅ **SAM3**: Open-vocabulary segmentation
- 🔄 **YOLO**: Object detection (coming soon)
- 🔄 **DepthAnything**: Depth estimation (coming soon)

## Documentation

See [docs/CV_Pipeline_Guide.md](../../../docs/CV_Pipeline_Guide.md) for full documentation.

## Architecture

- **C++ Node**: Fast ROS2 integration, manages Python workers
- **Python Workers**: Run AI models on demand
- **Lazy Loading**: Models only loaded when requested

## Requirements

- ROS2 Jazzy
- OpenCV
- Python 3.12+ with PyTorch 2.7+
- CUDA 12.6+ (recommended for real-time)
- Conda environment: `howyouseeme`
