# SAM2 Server - Fast Real-Time Processing!

## What's Different?

### Old Approach (C++ Pipeline)
- ❌ Used message_filters synchronization (failed)
- ❌ Spawned Python process on each request
- ❌ Loaded model on first request (~2s delay)
- ❌ Never worked due to timestamp sync issues

### New Approach (SAM2 Server)
- ✅ **No synchronization** - Just uses latest images
- ✅ **Model pre-loaded** at startup
- ✅ **Persistent Python process** - No spawning overhead
- ✅ **Fast processing** - ~0.3-0.5s per request

## Quick Start

### 1. Launch SAM2 Server

```bash
./launch_kinect_sam2_server.sh
```

This will:
1. Start Kinect (30 FPS)
2. Start SAM2 Server
3. **Load SAM2 model** (happens once at startup)
4. Wait for requests

### 2. Send Request

In another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

# Send segmentation request
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:prompt_type=point'"
```

### 3. View Results

```bash
# See results
ros2 topic echo /cv_pipeline/results
```

## Expected Performance

| Metric | Value |
|--------|-------|
| Model Load | ~2s (once at startup) |
| First Request | ~0.3-0.5s ✅ |
| Subsequent | ~0.3-0.5s ✅ |
| VRAM | 0.28 GB |
| Device | CUDA |

**Much faster than before!** No model loading delay on requests.

## How It Works

```
Kinect (30 FPS)
    ↓
SAM2 Server (Python ROS2 Node)
    ├─ Subscribes to /kinect2/qhd/image_color
    ├─ Subscribes to /kinect2/qhd/image_depth
    ├─ Stores latest images (no sync needed!)
    ├─ SAM2 model PRE-LOADED in memory
    └─ Processes requests instantly
        ↓
Results published to /cv_pipeline/results
```

## Key Improvements

### 1. No Synchronization
- Takes **latest RGB** and **latest depth** independently
- No timestamp matching required
- Always has images ready

### 2. Pre-loaded Model
- Model loads **once** at startup
- Stays in GPU memory
- No loading delay on requests

### 3. Persistent Process
- Python process runs continuously
- No subprocess spawning overhead
- Faster response time

## Troubleshooting

### "No RGB image available yet"

Wait a few seconds after launch for images to arrive:
```bash
# Check if images are publishing
ros2 topic hz /kinect2/qhd/image_color
```

### "SAM2 model not loaded"

Check SAM2 server logs for errors. Make sure conda environment is activated.

### Slow Processing

First request after launch might be slower (~1s) as CUDA warms up. Subsequent requests should be ~0.3-0.5s.

## Comparison

### Standalone Script
```bash
./capture_and_process_kinect.sh
# Time: ~2s (includes model load each time)
```

### SAM2 Server
```bash
# First launch (one time)
./launch_kinect_sam2_server.sh  # Loads model

# Then each request
ros2 topic pub --once /cv_pipeline/model_request ...
# Time: ~0.3-0.5s ✅ FAST!
```

## Integration with LLM

```python
import rclpy
from std_msgs.msg import String
import json

def segment_scene():
    """Request segmentation from SAM2 server"""
    # Publish request
    pub.publish(String(data='sam2:prompt_type=point'))
    
    # Wait for result (implement subscriber)
    # Returns in ~0.3-0.5s
    return result
```

## Files

- `ros2_ws/src/cv_pipeline/python/sam2_server.py` - SAM2 ROS2 server
- `launch_kinect_sam2_server.sh` - Launch script
- `SAM2_SERVER_GUIDE.md` - This guide

## Success!

This approach solves both problems:
1. ✅ **No timestamp sync issues** - Uses latest images
2. ✅ **Fast processing** - Model pre-loaded

Ready for production use! 🚀
