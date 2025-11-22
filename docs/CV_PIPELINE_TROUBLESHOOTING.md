# CV Pipeline Troubleshooting Guide

## Issue: No Results Published

### Symptoms
- Request received: `[INFO] Model request: sam2:prompt_type=point`
- No processing message
- No results on `/cv_pipeline/results`
- RViz visualization empty

### Root Cause
The CV pipeline waits for **synchronized RGB + Depth images** before processing. If images aren't arriving, requests queue but don't process.

### Solution 1: Launch Without SLAM

RTAB-Map might be consuming the images. Try CV pipeline alone:

```bash
# Stop everything
pkill -f kinect2_bridge
pkill -f cv_pipeline
pkill -f rtabmap

# Launch Kinect + CV only (no SLAM)
./launch_kinect_cv_only.sh
```

Then send request:
```bash
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:prompt_type=point'"
```

### Solution 2: Check Image Topics

Verify images are publishing:

```bash
# Check topics exist
ros2 topic list | grep kinect2

# Check image rate
ros2 topic hz /kinect2/qhd/image_color
ros2 topic hz /kinect2/qhd/image_depth

# Should show ~30 Hz
```

### Solution 3: Debug CV Pipeline

Use the debug script:

```bash
./test_cv_pipeline_debug.sh
```

This checks:
1. CV Pipeline running
2. Kinect topics exist
3. Image rate
4. Sends test request
5. Waits for results

### Solution 4: Check Logs

Look for warning messages:

```bash
# In the terminal running launch_kinect_cv_only.sh
# Look for:
[cv_pipeline_node] [WARN] Pending requests but no images yet. Waiting for Kinect...
```

If you see this, images aren't arriving.

### Solution 5: Rebuild with Debug

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select cv_pipeline --symlink-install
source install/setup.bash
```

## Common Issues

### Issue: "Pending requests but no images yet"

**Cause**: Image callback not receiving data

**Fix**:
1. Check Kinect is publishing: `ros2 topic hz /kinect2/qhd/image_color`
2. Check topic names match in launch file
3. Restart Kinect bridge

### Issue: Processing takes too long

**Cause**: SAM2 model loading or processing

**Expected**:
- First request: ~1-2s (model loads)
- Subsequent: ~0.3-0.7s

**If slower**:
- Check GPU: `nvidia-smi`
- Check CUDA: Look for `"device": "cuda"` in results
- Try smaller model: Edit `cv_pipeline_node.cpp`, change `--model-size tiny`

### Issue: Out of memory

**Cause**: 4GB GPU too small

**Fix**:
- Already using tiny model (0.28GB)
- Close other GPU applications
- Check: `nvidia-smi` for memory usage

## Verification Steps

### Step 1: Test SAM2 Worker Standalone

```bash
./test_cv_pipeline_simple.sh
```

Expected output:
```json
{
  "model": "sam2",
  "device": "cuda",
  "processing_time": 0.34
}
```

✅ If this works, SAM2 is fine. Issue is in ROS2 integration.

### Step 2: Test Kinect Alone

```bash
ros2 launch kinect2_bridge kinect2_bridge_launch.yaml
```

In another terminal:
```bash
ros2 topic hz /kinect2/qhd/image_color
```

Expected: ~30 Hz

✅ If this works, Kinect is fine.

### Step 3: Test CV Pipeline with Kinect

```bash
./launch_kinect_cv_only.sh
```

Wait for "System Ready", then:
```bash
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:prompt_type=point'"
```

Check logs for:
```
[INFO] Model request: sam2:prompt_type=point
[INFO] Processing 1 pending request(s)
[INFO] Processing with model: sam2
[INFO] Model sam2 completed in XXX ms
```

### Step 4: Check Results

```bash
ros2 topic echo /cv_pipeline/results
```

Should show JSON with segmentation results.

## Debug Commands

```bash
# Check if CV pipeline is running
pgrep -f cv_pipeline_node

# Check node info
ros2 node info /cv_pipeline_node

# Check topics
ros2 topic list | grep cv_pipeline

# Monitor requests
ros2 topic echo /cv_pipeline/model_request

# Monitor results
ros2 topic echo /cv_pipeline/results

# Check image rate
ros2 topic hz /kinect2/qhd/image_color

# Check GPU
nvidia-smi
```

## Expected Behavior

### Normal Flow

1. **Launch**: Kinect starts, publishes at 30 Hz
2. **CV Pipeline**: Starts, subscribes to images
3. **Images arrive**: CV pipeline receives RGB+Depth
4. **Request sent**: Model request published
5. **Processing**: Request queued, images available, processing starts
6. **SAM2 runs**: Python worker called, model processes image
7. **Results**: JSON published to `/cv_pipeline/results`
8. **Visualization**: Image published to `/cv_pipeline/visualization`

### Timing

- Model load (first time): ~1-2s
- Processing (subsequent): ~0.3-0.7s
- Total latency: ~1-3s for first request, ~0.5-1s after

## Quick Fixes

### Fix 1: Restart Everything

```bash
pkill -f kinect2_bridge
pkill -f cv_pipeline
pkill -f rtabmap
./launch_kinect_cv_only.sh
```

### Fix 2: Use Standalone Test

If ROS2 integration is problematic, use standalone:

```bash
# Capture Kinect frame manually
ros2 run image_view image_saver --ros-args \
  -r image:=/kinect2/qhd/image_color \
  -p filename_format:=/tmp/kinect_rgb.jpg

# Process with SAM2
conda activate howyouseeme
python ros2_ws/src/cv_pipeline/python/sam2_worker.py \
  --rgb /tmp/kinect_rgb.jpg \
  --depth /tmp/kinect_depth.png \
  --params "prompt_type=point" \
  --model-size tiny
```

### Fix 3: Check Python Environment

```bash
conda activate howyouseeme
python -c "import torch; print(f'PyTorch: {torch.__version__}'); print(f'CUDA: {torch.cuda.is_available()}')"
python -c "import sys; sys.path.insert(0, 'sam2'); from sam2.build_sam import build_sam2; print('SAM2 OK')"
```

## Still Not Working?

1. Check `test_cv_pipeline_debug.sh` output
2. Look at CV pipeline terminal for error messages
3. Verify Kinect is publishing: `ros2 topic hz /kinect2/qhd/image_color`
4. Try standalone SAM2 test: `./test_cv_pipeline_simple.sh`
5. Check GPU memory: `nvidia-smi`

## Success Indicators

✅ Kinect publishing at ~30 Hz  
✅ CV pipeline receiving images  
✅ Request processed within 1-3s  
✅ Results published to `/cv_pipeline/results`  
✅ Visualization shows in RViz  

If all these work, the system is operational!
