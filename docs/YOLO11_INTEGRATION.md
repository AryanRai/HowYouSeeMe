# YOLO11 Integration Complete! 🚀

## Overview

YOLO11 has been successfully integrated into the CV Pipeline with support for 4 different tasks.

## Supported Tasks

### 1. Object Detection 🔍
Detect objects with bounding boxes and class labels.

**Features:**
- 80 COCO classes (person, car, dog, etc.)
- Confidence scores
- Bounding box coordinates
- Real-time performance

**Usage:**
```bash
Menu → 3 (YOLO11) → 1 (Detection)
Confidence: 0.25
IOU: 0.7
Duration: 0 (single frame)
```

### 2. Instance Segmentation 🎭
Segment objects with pixel-perfect masks.

**Features:**
- Object detection + segmentation masks
- Per-instance masks
- Class labels and confidence
- Better than FastSAM for known classes

**Usage:**
```bash
Menu → 3 (YOLO11) → 2 (Segmentation)
Confidence: 0.25
IOU: 0.7
Duration: 0
```

### 3. Pose Estimation 🧍
Detect human poses with 17 keypoints.

**Features:**
- Human detection
- 17 body keypoints (nose, eyes, shoulders, elbows, wrists, hips, knees, ankles)
- Keypoint confidence scores
- Multiple person support

**Keypoints:**
0. Nose
1-2. Eyes (left, right)
3-4. Ears (left, right)
5-6. Shoulders (left, right)
7-8. Elbows (left, right)
9-10. Wrists (left, right)
11-12. Hips (left, right)
13-14. Knees (left, right)
15-16. Ankles (left, right)

**Usage:**
```bash
Menu → 3 (YOLO11) → 3 (Pose Estimation)
Confidence: 0.25
IOU: 0.7
Duration: -1 (continuous streaming)
FPS: 10
```

### 4. Oriented Bounding Boxes (OBB) 📐
Detect objects with rotated bounding boxes.

**Features:**
- 4-point rotated boxes
- Better for angled objects
- Useful for aerial imagery, text detection
- 15 classes (plane, ship, storage tank, etc.)

**Usage:**
```bash
Menu → 3 (YOLO11) → 4 (OBB)
Confidence: 0.25
IOU: 0.7
Duration: 0
```

## Parameters

### Confidence Threshold (conf)
- Range: 0.0 - 1.0
- Default: 0.25
- Lower = more detections (but more false positives)
- Higher = fewer detections (but more accurate)

### IOU Threshold (iou)
- Range: 0.0 - 1.0
- Default: 0.7
- Used for Non-Maximum Suppression (NMS)
- Lower = more aggressive filtering
- Higher = keep more overlapping boxes

### Duration
- 0 = Single frame
- N > 0 = Stream for N seconds
- -1 = Continuous streaming

### FPS
- Range: 1-30
- Default: 10
- Higher = more frequent updates (more GPU usage)

## Model Sizes

All tasks use YOLO11n (nano) models for speed:
- yolo11n.pt - Detection (~6 MB)
- yolo11n-seg.pt - Segmentation (~7 MB)
- yolo11n-pose.pt - Pose (~7 MB)
- yolo11n-obb.pt - OBB (~6 MB)

## Performance

| Task | Speed | Accuracy | Best For |
|------|-------|----------|----------|
| Detection | ~10ms | High | General object detection |
| Segmentation | ~15ms | High | Precise object boundaries |
| Pose | ~12ms | High | Human pose tracking |
| OBB | ~11ms | Medium | Rotated objects |

## Examples

### Real-time Person Detection
```bash
./cv_menu.sh
→ 3 (YOLO11)
→ 1 (Detection)
→ Conf: 0.5
→ IOU: 0.7
→ Duration: -1
→ FPS: 15
```

### Pose Tracking for Exercise
```bash
./cv_menu.sh
→ 3 (YOLO11)
→ 3 (Pose)
→ Conf: 0.3
→ IOU: 0.7
→ Duration: 30
→ FPS: 10
```

### Instance Segmentation
```bash
./cv_menu.sh
→ 3 (YOLO11)
→ 2 (Segmentation)
→ Conf: 0.25
→ IOU: 0.7
→ Duration: 0
```

## Visualization

YOLO11 uses built-in Ultralytics visualization:
- **Detection**: Colored bounding boxes with labels
- **Segmentation**: Colored masks + bounding boxes
- **Pose**: Skeleton overlay with keypoints
- **OBB**: Rotated bounding boxes

## Comparison with Other Models

| Feature | SAM2 | FastSAM | YOLO11 |
|---------|------|---------|--------|
| Speed | Medium | Fast | Very Fast |
| Classes | Any | Any | 80 COCO |
| Segmentation | ✅ | ✅ | ✅ |
| Detection | ❌ | ❌ | ✅ |
| Pose | ❌ | ❌ | ✅ |
| Text Prompts | ❌ | ✅ | ❌ |
| Point Prompts | ✅ | ✅ | ❌ |
| Real-time | ❌ | ✅ | ✅ |

## Use Cases

### YOLO11 Detection
- Counting objects
- Traffic monitoring
- Inventory management
- Security surveillance

### YOLO11 Segmentation
- Precise object boundaries
- Background removal
- Object measurement
- Quality inspection

### YOLO11 Pose
- Fitness tracking
- Gesture recognition
- Sports analysis
- Human-robot interaction

### YOLO11 OBB
- Aerial imagery analysis
- Document scanning
- Text detection
- Parking lot monitoring

## Tips

1. **For Speed**: Use detection mode
2. **For Accuracy**: Increase confidence threshold
3. **For Multiple Objects**: Lower IOU threshold
4. **For Pose**: Ensure good lighting and full body visible
5. **For Streaming**: Use FPS 10-15 for smooth performance

## Troubleshooting

### No Detections
- Lower confidence threshold
- Check if objects are in COCO classes
- Ensure good lighting

### Too Many False Positives
- Increase confidence threshold
- Increase IOU threshold

### Slow Performance
- Lower FPS
- Use detection instead of segmentation
- Ensure GPU is being used

## Direct ROS2 Commands

```bash
# Detection
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'yolo11:task=detect,conf=0.25,iou=0.7'"

# Segmentation streaming
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'yolo11:task=segment,conf=0.3,iou=0.7,stream=true,duration=10,fps=10'"

# Pose estimation
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'yolo11:task=pose,conf=0.25,iou=0.7'"

# OBB
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'yolo11:task=obb,conf=0.25,iou=0.7'"
```

## System Status

The CV Pipeline now has **4 powerful models**:

1. **SAM2** - Universal segmentation with prompts
2. **FastSAM** - Fast segmentation with text prompts
3. **YOLO11** - Multi-task detection, segmentation, pose, OBB
4. [Future] More models coming soon!

Restart the server to load YOLO11:
```bash
pkill -f sam2_server_v2.py
./launch_kinect_sam2_server.sh
```

Then access via menu:
```bash
./cv_menu.sh
→ 3 (YOLO11)
```

🎉 **YOLO11 is ready to use!**
