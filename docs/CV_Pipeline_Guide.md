# Computer Vision Pipeline Guide

## Overview

The CV Pipeline is a modular framework for running multiple computer vision models on Kinect RGB-D data. Models are **lazy-loaded** - they only run when requested by an LLM or user, making it efficient for real-time applications.

## Architecture

```
Kinect v2 RGB-D Stream
    ↓
C++ ROS2 CV Pipeline Node (cv_pipeline_node)
    ├─ Subscribes to Kinect topics
    ├─ Rate-limits processing (5 FPS default)
    ├─ Receives model requests
    └─ Spawns Python workers on demand
        ↓
Python Model Workers (lazy-loaded)
    ├─ SAM3 (Segmentation) ✅
    ├─ YOLO (Object Detection) 🔄
    ├─ DepthAnything (Depth Estimation) 🔄
    └─ Custom Models... 🔄
```

## Current Models

### SAM3 (Segment Anything Model 3)

**Status**: ✅ Implemented

**Description**: Meta's latest open-vocabulary segmentation model. Can segment objects based on text prompts like "person", "red car", "all objects", etc.

**Performance**: 
- Model size: ~848M parameters
- Expected latency: 100-500ms per frame (with CUDA)
- Supports: Text prompts, point prompts, box prompts

**Usage**:
```bash
# Segment all people
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String "data: 'sam3:prompt=person'"

# Segment all objects
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String "data: 'sam3:prompt=all objects'"

# Segment specific items
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String "data: 'sam3:prompt=red cup'"
```

## Setup

### 1. Install SAM3

SAM3 is already installed in your `howyouseeme` conda environment. To use it, you need to authenticate with Hugging Face:

```bash
# Activate conda environment
conda activate howyouseeme

# Login to Hugging Face (you need an access token)
python -c "from huggingface_hub import login; login()"

# Or set token as environment variable
export HF_TOKEN="your_token_here"
```

### 2. Build the CV Pipeline

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select cv_pipeline --symlink-install
source install/setup.bash
```

### 3. Launch

```bash
# Launch Kinect + CV Pipeline together
./launch_kinect_cv_pipeline.sh

# Or launch separately:
# Terminal 1: Kinect
ros2 launch kinect2_ros2_cuda kinect2_bridge.launch.py

# Terminal 2: CV Pipeline
ros2 launch cv_pipeline cv_pipeline.launch.py
```

## Usage

### Sending Model Requests

The CV Pipeline listens on `/cv_pipeline/model_request` for model requests. Format:

```
model_name:param1=value1,param2=value2
```

Examples:

```bash
# SAM3 with text prompt
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam3:prompt=person'"

# SAM3 with different prompt
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam3:prompt=chair'"
```

### Viewing Results

Results are published as JSON on `/cv_pipeline/results`:

```bash
ros2 topic echo /cv_pipeline/results
```

Example output:
```json
{
  "model": "sam3",
  "prompt": "person",
  "num_masks": 2,
  "processing_time": 0.234,
  "device": "cuda",
  "mask_stats": [
    {
      "id": 0,
      "area": 45678,
      "bbox": [120, 80, 200, 400],
      "score": 0.95
    },
    {
      "id": 1,
      "area": 32100,
      "bbox": [450, 100, 180, 380],
      "score": 0.87
    }
  ]
}
```

### Visualization

Visual output with overlays is published on `/cv_pipeline/visualization`. View in RViz:

```bash
rviz2
# Add Image display
# Set topic to /cv_pipeline/visualization
```

## Configuration

Edit `ros2_ws/src/cv_pipeline/config/models.yaml`:

```yaml
pipeline:
  processing:
    max_fps: 5.0  # Adjust processing rate
```

Edit launch file parameters:

```bash
ros2 launch cv_pipeline cv_pipeline.launch.py max_fps:=10.0
```

## Performance Tuning

### For Real-Time Performance

1. **Use CUDA**: Ensure PyTorch is using GPU
   ```bash
   conda activate howyouseeme
   python -c "import torch; print(torch.cuda.is_available())"
   ```

2. **Reduce Processing Rate**: Lower `max_fps` parameter
   ```bash
   ros2 launch cv_pipeline cv_pipeline.launch.py max_fps:=3.0
   ```

3. **Use Smaller Models**: SAM3 has different model sizes
   - `sam3_hiera_small` - Faster, less accurate
   - `sam3_hiera_large` - Slower, more accurate (default)

### Expected Performance

| Model | Device | Latency | FPS |
|-------|--------|---------|-----|
| SAM3 Large | CUDA (RTX 3060) | ~200ms | ~5 FPS |
| SAM3 Large | CPU | ~2000ms | ~0.5 FPS |
| SAM3 Small | CUDA | ~100ms | ~10 FPS |

## Adding New Models

To add a new model (e.g., YOLO):

1. **Create Python worker**: `ros2_ws/src/cv_pipeline/python/yolo_worker.py`
   ```python
   #!/usr/bin/env python3
   import argparse
   import json
   # ... implement model loading and processing
   ```

2. **Make it executable**:
   ```bash
   chmod +x ros2_ws/src/cv_pipeline/python/yolo_worker.py
   ```

3. **Use it**:
   ```bash
   ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
     "data: 'yolo:confidence=0.5'"
   ```

The C++ node will automatically find and execute `yolo_worker.py`.

## Troubleshooting

### SAM3 Not Loading

**Issue**: "SAM3 not available - running in mock mode"

**Solutions**:
1. Check conda environment is activated
2. Verify SAM3 installation: `pip list | grep sam3`
3. Check Hugging Face authentication
4. Download model manually:
   ```python
   from sam3.model_builder import build_sam3_image_model
   model = build_sam3_image_model()  # Will download if needed
   ```

### CUDA Out of Memory

**Issue**: "CUDA out of memory"

**Solutions**:
1. Reduce processing rate: `max_fps:=3.0`
2. Close other GPU applications
3. Use smaller model variant
4. Process at lower resolution

### Slow Processing

**Issue**: Processing takes > 1 second per frame

**Solutions**:
1. Verify CUDA is being used (check logs for "device: cuda")
2. Ensure GPU drivers are up to date
3. Check GPU utilization: `nvidia-smi`
4. Consider using mock mode for testing

### No Images Received

**Issue**: "No RGB image provided"

**Solutions**:
1. Verify Kinect is running: `ros2 topic list | grep kinect`
2. Check image topics: `ros2 topic hz /kinect2/qhd/image_color`
3. Verify topic names in launch file match Kinect output

## Integration with LLM

The CV Pipeline is designed to be triggered by an LLM. Example integration:

```python
# In your LLM tool/function
def segment_objects(prompt: str):
    """Segment objects in the current camera view"""
    import rclpy
    from std_msgs.msg import String
    
    # Publish request
    msg = String()
    msg.data = f"sam3:prompt={prompt}"
    publisher.publish(msg)
    
    # Wait for result on /cv_pipeline/results
    # ... handle result
```

## Future Models

Planned additions:

- **YOLO**: Fast object detection
- **DepthAnything**: Monocular depth estimation
- **CLIP**: Image-text understanding
- **Custom models**: Task-specific models

## Files

```
ros2_ws/src/cv_pipeline/
├── src/
│   └── cv_pipeline_node.cpp       # Main C++ node
├── python/
│   └── sam3_worker.py             # SAM3 Python worker
├── launch/
│   └── cv_pipeline.launch.py     # Launch file
├── config/
│   └── models.yaml                # Configuration
├── package.xml
└── CMakeLists.txt
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/kinect2/qhd/image_color` | sensor_msgs/Image | RGB input (subscribed) |
| `/kinect2/qhd/image_depth` | sensor_msgs/Image | Depth input (subscribed) |
| `/cv_pipeline/model_request` | std_msgs/String | Model requests (subscribed) |
| `/cv_pipeline/results` | std_msgs/String | JSON results (published) |
| `/cv_pipeline/visualization` | sensor_msgs/Image | Visual output (published) |

## Next Steps

1. **Test SAM3**: Run the pipeline and send test requests
2. **Authenticate HuggingFace**: Login to download SAM3 model
3. **Benchmark Performance**: Test latency with your GPU
4. **Add More Models**: Implement YOLO, DepthAnything, etc.
5. **LLM Integration**: Connect to your LLM system

Happy segmenting! 🎯
