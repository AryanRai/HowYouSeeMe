# CV Pipeline V2 - Quick Reference Guide

## Overview

The CV Pipeline V2 uses an extensible model manager architecture that makes it easy to add new models and switch between them.

## Launch Scripts

### 1. Full System (Kinect + SLAM + CV Pipeline + RViz)
```bash
./launch_full_system_rviz.sh
```
Launches everything with corrected coordinate frames and full visualization.

### 2. Kinect + CV Pipeline Only
```bash
./launch_kinect_sam2_server.sh
```
Launches just Kinect and CV Pipeline (no SLAM, no RViz).

## Available Models

Currently available:
- **SAM2** - Segment Anything Model 2 (image segmentation)

Easy to add:
- Depth Anything (depth estimation)
- YOLO (object detection)
- DINO (feature extraction)
- GroundingDINO (open-vocabulary detection)

## SAM2 Modes

### 1. Point Mode (Default)
Segment object at a specific point.

```bash
# Center point
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:prompt_type=point'"

# Custom point (x=640, y=360)
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:prompt_type=point,x=640,y=360'"
```

### 2. Box Mode
Segment everything inside a bounding box.

```bash
# Custom box (x1,y1,x2,y2)
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:prompt_type=box,box=200,150,700,450'"
```

### 3. Multiple Points Mode
Use multiple foreground/background points for refinement.

```bash
# Two foreground points
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:prompt_type=points,points=400,300,500,350,labels=1,1'"

# Foreground + background (1=fg, 0=bg)
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:prompt_type=points,points=480,270,200,100,labels=1,0'"
```

### 4. Everything Mode
Automatically segment all objects in the frame.

```bash
# Default grid
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:prompt_type=everything'"

# Custom grid size
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:prompt_type=everything,grid_size=16'"
```

### 5. Streaming Mode
Continuous segmentation for video.

```bash
# Using helper script (10 seconds @ 5 FPS)
./start_sam2_stream.sh 10 5

# Manual command
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:prompt_type=point,stream=true,duration=15,fps=3'"

# Stop streaming
./stop_sam2_stream.sh
```

## System Commands

### List Available Models
```bash
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:list_models=true'"
```

### Get Model Info
```bash
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:model_info=true'"
```

### View Results
```bash
# Single result
ros2 topic echo /cv_pipeline/results --once

# Continuous monitoring
ros2 topic echo /cv_pipeline/results
```

## Visualization

### RViz Topics
- **Camera Image**: `/kinect2/qhd/image_color`
- **CV Results**: `/cv_pipeline/visualization`
- **SLAM Map**: `/rtabmap/mapData`
- **Point Cloud**: `/kinect2/qhd/points`

### Coordinate Frames
- **Base Frame**: `kinect2_link` (corrected orientation)
- **SLAM Frames**: `map`, `odom`
- **Optical Frame**: `kinect2_rgb_optical_frame`

## Architecture

### Components
1. **ModelManager** - Manages multiple CV models
2. **BaseModel** - Abstract base class for all models
3. **SAM2Model** - SAM2 implementation
4. **CVPipelineServer** - ROS2 node

### Files
- `cv_model_manager.py` - Model manager and base classes
- `sam2_server_v2.py` - ROS2 server using model manager
- `ADD_NEW_MODEL_GUIDE.md` - Guide for adding new models

## Performance

- **Model Loading**: ~2-3 seconds (one-time)
- **Processing Time**: ~0.1-0.3 seconds per frame
- **Streaming**: Up to 10 FPS (configurable)
- **GPU Memory**: ~0.28 GB (SAM2 tiny)

## Troubleshooting

### Model Not Loading
```bash
# Check if conda environment is activated
conda activate howyouseeme

# Test model manager
python3 test_model_manager.py
```

### No Visualization
```bash
# Check if topic is publishing
ros2 topic hz /cv_pipeline/visualization

# Send a request first
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:prompt_type=point'"
```

### Coordinate Frame Issues
The system now uses `kinect2_link` as the base frame with corrected orientation. RTAB-Map handles the coordinate transformations internally.

## Helper Scripts

- `./sam2_modes_guide.sh` - View all SAM2 modes
- `./start_sam2_stream.sh [duration] [fps]` - Start streaming
- `./stop_sam2_stream.sh` - Stop streaming
- `./test_model_manager.py` - Test model manager
- `./kill_all.sh` - Stop all processes

## Adding New Models

See `ros2_ws/src/cv_pipeline/python/ADD_NEW_MODEL_GUIDE.md` for detailed instructions on adding new models to the pipeline.

## Future Enhancements

- [ ] Depth Anything integration
- [ ] YOLO object detection
- [ ] DINO feature extraction
- [ ] Model ensembles
- [ ] Model chaining
- [ ] Automatic model selection
- [ ] Performance benchmarking
