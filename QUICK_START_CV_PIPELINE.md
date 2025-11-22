# Quick Start: CV Pipeline with SAM2

Get the CV Pipeline running with SAM2 segmentation in 5 minutes.

## Prerequisites

- ✅ Kinect v2 connected
- ✅ ROS2 Jazzy installed
- ✅ CUDA-capable GPU (recommended)
- ✅ Conda environment `howyouseeme` (already set up)

## Step 1: Verify SAM2 Installation

SAM2 is already installed and ready to use!

## Step 2: Test SAM2 (Optional)

```bash
./test_cv_pipeline_simple.sh
```

You should see SAM2 processing with `"device": "cuda"`.

## Step 3: Build the CV Pipeline

```bash
cd ~/Documents/GitHub/HowYouSeeMe/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select cv_pipeline --symlink-install
source install/setup.bash
```

## Step 4: Launch Everything

```bash
cd ~/Documents/GitHub/HowYouSeeMe
./launch_kinect_cv_pipeline.sh
```

This starts:
- Kinect2 bridge (RGB-D camera)
- CV Pipeline node (model manager)

## Step 5: Send a Request

In a new terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Documents/GitHub/HowYouSeeMe/ros2_ws/install/setup.bash

# Segment people in the scene
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam3:prompt=person'"
```

## Step 6: View Results

```bash
# See JSON results
ros2 topic echo /cv_pipeline/results
```

Example output:
```json
{
  "model": "sam2",
  "prompt_type": "point",
  "num_masks": 3,
  "processing_time": 0.73,
  "device": "cuda",
  "scores": [0.96, 0.04, 0.60],
  "mask_stats": [
    {
      "id": 0,
      "area": 237428,
      "bbox": [0, 0, 639, 479],
      "score": 0.96
    }
  ]
}
```

## More Examples

```bash
# Segment with point prompt (default)
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:prompt_type=point'"

# Segment with box prompt
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:prompt_type=box'"
```

## Visualize in RViz

```bash
rviz2
```

In RViz:
1. Click "Add" → "By topic"
2. Select `/cv_pipeline/visualization` → "Image"
3. See the segmentation overlays

## Troubleshooting

### "SAM2 not available - running in mock mode"

**Solution**: Check SAM2 installation
```bash
conda activate howyouseeme
cd sam2 && python -c "from sam2.build_sam import build_sam2; print('OK')"
```

### "CUDA out of memory"

**Solution**: Reduce processing rate
```bash
ros2 launch cv_pipeline cv_pipeline.launch.py max_fps:=3.0
```

### "No RGB image provided"

**Solution**: Check Kinect is running
```bash
ros2 topic list | grep kinect
ros2 topic hz /kinect2/qhd/image_color
```

## Performance

With CUDA on 4GB GPU (RTX 3050):
- **SAM2 Tiny**: ~0.7s per frame (1.4 FPS) ✅ Works!
- **SAM2 Small**: ~1.0s per frame (1 FPS) - May work
- **SAM2 Large**: ❌ Out of memory

**Recommendation**: Use SAM2 Tiny for 4GB GPUs

## Next Steps

1. **Test different prompts**: Try various objects and scenes
2. **Benchmark performance**: Measure latency on your GPU
3. **Add more models**: Implement YOLO, DepthAnything, etc.
4. **LLM Integration**: Connect to your AI system

## Full Documentation

- [CV Pipeline Guide](docs/CV_Pipeline_Guide.md) - Complete documentation
- [Setup Complete](docs/CV_Pipeline_Setup_Complete.md) - Architecture details
- [Main README](README.md) - Project overview

## Quick Commands

```bash
# Build
cd ros2_ws && colcon build --packages-select cv_pipeline

# Launch
./launch_kinect_cv_pipeline.sh

# Test
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String "data: 'sam2:prompt_type=point'"

# View
ros2 topic echo /cv_pipeline/results

# Stop
pkill -f kinect2_bridge && pkill -f cv_pipeline_node
```

That's it! You now have a modular CV pipeline running SAM3 segmentation on Kinect data. 🎉
