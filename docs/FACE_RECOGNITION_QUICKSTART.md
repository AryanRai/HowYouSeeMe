# Face Recognition - Quick Start Guide

## Installation (5 minutes)

```bash
# 1. Install InsightFace
./install_insightface.sh

# 2. Restart CV Pipeline server
pkill -f sam2_server_v2.py
./launch_kinect_sam2_server.sh
```

Wait for "CV Pipeline Server ready!" message.

## Quick Test (2 minutes)

### Test 1: Detect Faces
```bash
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'insightface:mode=detect'"
```

### Test 2: Register Your Face
```bash
# Look at the camera, then:
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'insightface:mode=register,name=YourName'"
```

### Test 3: Recognize Your Face
```bash
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'insightface:mode=detect_recognize'"
```

### Test 4: Check Liveness
```bash
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'insightface:mode=liveness'"
```

## Watch Results

```bash
# Terminal 1: Watch results
ros2 topic echo /cv_pipeline/results

# Terminal 2: View visualization in RViz
rviz2
# Add Image display, topic: /cv_pipeline/visualization
```

## Common Use Cases

### Register Multiple People
```bash
# Person 1
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'insightface:mode=register,name=Alice'"

# Person 2
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'insightface:mode=register,name=Bob'"

# Person 3
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'insightface:mode=register,name=Charlie'"
```

### Continuous Recognition (Streaming)
```bash
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'insightface:mode=detect_recognize,stream=true,duration=60,fps=5'"
```

### Full Analysis with Liveness
```bash
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'insightface:mode=analyze'"
```

## Modes Summary

| Mode | Description | Use Case |
|------|-------------|----------|
| `detect` | Face detection only | Get face locations |
| `recognize` | Recognition only | Pre-detected faces |
| `detect_recognize` | Full pipeline | Complete recognition |
| `register` | Add new person | Build database |
| `liveness` | Anti-spoofing | Verify real person |
| `analyze` | Everything | Maximum info |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `mode` | detect_recognize | Operation mode |
| `name` | - | Person name (register mode) |
| `threshold` | 0.6 | Similarity threshold |
| `det_size` | 640 | Detection size |
| `max_num` | 0 | Max faces (0=unlimited) |

## Troubleshooting

### "Model not loaded"
```bash
# Check installation
python3 -c "import insightface; print('OK')"

# Restart server
pkill -f sam2_server_v2.py
./launch_kinect_sam2_server.sh
```

### "No face detected"
- Move closer to camera
- Ensure good lighting
- Face the camera directly

### "No match found"
- Register the person first
- Lower threshold: `threshold=0.5`
- Register more samples

## Next Steps

1. **Register your team** - Add everyone who needs recognition
2. **Test liveness** - Verify anti-spoofing works
3. **Tune threshold** - Adjust for your security needs
4. **Build pipelines** - Combine with YOLO, SAM2, etc.

## Full Documentation

See `docs/INSIGHTFACE_INTEGRATION.md` for complete details.

## Support

- Check logs in server terminal
- Use `./debug_cv_pipeline.sh` for diagnostics
- See `docs/CV_PIPELINE_TROUBLESHOOTING.md`

Ready to recognize faces! 🎉
