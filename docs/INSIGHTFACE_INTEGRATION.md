# InsightFace Integration - Face Recognition System

## Overview

Complete face recognition system with:
- **Face Detection** - SCRFD (fast, accurate)
- **Face Recognition** - ArcFace (99.83% accuracy)
- **Liveness Detection** - Depth-based anti-spoofing
- **Face Database** - Simple file-based storage

## Installation

```bash
./install_insightface.sh
```

Or manually:
```bash
source ~/anaconda3/bin/activate howyouseeme
pip install insightface onnxruntime-gpu
mkdir -p data/faces
```

## Modes

### 1. detect - Face Detection Only
Detect faces and return bounding boxes + landmarks.

**Use case**: Pipeline composition, get face locations for other processing

**Parameters**:
- `det_size`: Detection size (default: 640)
- `max_num`: Max faces to detect (0 = unlimited)

**Example**:
```bash
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'insightface:mode=detect,det_size=640'"
```

**Output**:
```json
{
  "mode": "detect",
  "num_faces": 2,
  "faces": [
    {
      "bbox": [100, 150, 300, 400],
      "det_score": 0.99,
      "landmarks": [[x1,y1], [x2,y2], ...]
    }
  ]
}
```

### 2. recognize - Face Recognition Only
Recognize a face (assumes already cropped/aligned).

**Use case**: Pipeline composition, recognize pre-detected faces

**Parameters**:
- `threshold`: Similarity threshold (default: 0.6)

**Example**:
```bash
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'insightface:mode=recognize,threshold=0.6'"
```

**Output**:
```json
{
  "mode": "recognize",
  "recognized": true,
  "person_id": "person_001",
  "name": "John Doe",
  "similarity": 0.85,
  "confidence": 0.85
}
```

### 3. detect_recognize - Full Pipeline
Detect faces then recognize each one.

**Use case**: Complete face recognition in one step

**Parameters**:
- `threshold`: Similarity threshold (default: 0.6)
- `det_size`: Detection size (default: 640)
- `max_num`: Max faces (0 = unlimited)

**Example**:
```bash
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'insightface:mode=detect_recognize,threshold=0.6'"
```

**Output**:
```json
{
  "mode": "detect_recognize",
  "num_faces": 2,
  "faces": [
    {
      "bbox": [100, 150, 300, 400],
      "det_score": 0.99,
      "landmarks": [[x1,y1], ...],
      "recognized": true,
      "person_id": "person_001",
      "name": "John Doe",
      "similarity": 0.85,
      "age": 25,
      "gender": "M"
    },
    {
      "bbox": [400, 150, 600, 400],
      "recognized": false,
      "person_id": "unknown"
    }
  ]
}
```

### 4. register - Register New Face
Register a new person to the database.

**Use case**: Add new people to recognition system

**Parameters**:
- `name`: Person's name (required)
- `person_id`: Custom ID (optional, auto-generated if not provided)

**Example**:
```bash
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'insightface:mode=register,name=John_Doe'"
```

**Output**:
```json
{
  "mode": "register",
  "success": true,
  "person_id": "person_001",
  "name": "John_Doe",
  "num_samples": 1
}
```

**Best Practices**:
- Register 3-5 samples per person for robustness
- Use different angles and lighting
- Ensure good face quality (frontal, well-lit)

### 5. liveness - Liveness Detection
Check if face is live using depth data.

**Use case**: Anti-spoofing, verify real person vs photo/video

**Parameters**:
- None (uses depth automatically if available)

**Example**:
```bash
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'insightface:mode=liveness'"
```

**Output**:
```json
{
  "mode": "liveness",
  "is_live": true,
  "confidence": 0.92,
  "depth_variance": 1250.5,
  "depth_range": 45.2,
  "depth_mean": 850.3,
  "method": "depth_variance"
}
```

**How it works**:
- Real face: Uneven depth (nose, eyes, chin) → High variance
- Photo/screen: Flat surface → Low variance
- Threshold: variance > 100mm², range > 10mm

### 6. analyze - Full Analysis
Complete analysis: detect + recognize + liveness + attributes.

**Use case**: Maximum information extraction

**Parameters**:
- `threshold`: Similarity threshold (default: 0.6)
- `det_size`: Detection size (default: 640)

**Example**:
```bash
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'insightface:mode=analyze,threshold=0.6'"
```

**Output**:
```json
{
  "mode": "detect_recognize",
  "num_faces": 1,
  "faces": [
    {
      "bbox": [100, 150, 300, 400],
      "recognized": true,
      "person_id": "person_001",
      "name": "John Doe",
      "similarity": 0.85,
      "age": 25,
      "gender": "M"
    }
  ],
  "liveness": {
    "is_live": true,
    "confidence": 0.92,
    "method": "depth_variance"
  }
}
```

## Face Database

### Structure
```
data/faces/
├── face_database.pkl      # Embeddings (512-dim vectors)
└── metadata.json          # Names, timestamps, counts
```

### Database Format
```json
{
  "person_001": {
    "name": "John Doe",
    "first_seen": "2024-11-23 10:30:00",
    "last_seen": "2024-11-23 15:45:00",
    "encounter_count": 15,
    "samples": 3
  }
}
```

### Management

**List registered people**:
```bash
python3 -c "
import pickle, json
with open('data/faces/metadata.json') as f:
    data = json.load(f)
    for pid, info in data.items():
        print(f'{pid}: {info[\"name\"]} ({info[\"samples\"]} samples)')
"
```

**Clear database**:
```bash
rm data/faces/face_database.pkl
rm data/faces/metadata.json
```

## Visualization

All modes produce visualizations on `/cv_pipeline/visualization`:

- **Green boxes**: Recognized faces with name + similarity
- **Orange boxes**: Unknown faces
- **Yellow dots**: Face landmarks (5 points)
- **Text overlay**: Name, age, gender, liveness status

## Pipeline Composition

Combine with other models for advanced workflows:

### Example 1: Human Detection → Face Recognition
```bash
# 1. Detect humans with YOLO
yolo11:task=detect,conf=0.5

# 2. For each human, detect face
insightface:mode=detect

# 3. Recognize face
insightface:mode=recognize
```

### Example 2: Face + Pose + Segmentation
```bash
# 1. Detect face
insightface:mode=detect

# 2. Detect pose
yolo11:task=pose

# 3. Segment person
yolo11:task=segment
```

## Performance

- **Detection**: ~30ms per frame
- **Recognition**: ~50ms per face
- **Liveness**: ~20ms (depth-based)
- **Total (detect_recognize)**: ~80ms for single face

## Streaming Support

All modes support streaming:

```bash
# Stream face recognition at 10 FPS
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'insightface:mode=detect_recognize,stream=true,duration=30,fps=10'"
```

## Troubleshooting

### Model not loading
```bash
# Check InsightFace installation
python3 -c "import insightface; print(insightface.__version__)"

# Check ONNX Runtime
python3 -c "import onnxruntime; print(onnxruntime.get_device())"
```

### No faces detected
- Increase `det_size` (e.g., 1280)
- Check lighting conditions
- Ensure face is frontal and visible

### Low recognition accuracy
- Register more samples (3-5 per person)
- Lower threshold (e.g., 0.5)
- Ensure good quality registration images

### Liveness always fails
- Check depth data is available
- Verify Kinect is working
- Adjust thresholds in `insightface_worker.py`

## Security Considerations

1. **Always use liveness detection** for authentication
2. **Multi-sample registration** (3-5 samples minimum)
3. **Threshold tuning** based on security requirements:
   - High security: threshold = 0.7-0.8
   - Balanced: threshold = 0.6
   - Convenience: threshold = 0.5
4. **Audit logging** - track all recognition attempts
5. **Regular updates** - re-register people periodically

## Future Enhancements

- [ ] RGB anti-spoofing (MiniFASNet)
- [ ] Blink detection
- [ ] Face tracking across frames
- [ ] FAISS for fast similarity search
- [ ] PostgreSQL with pgvector
- [ ] Multi-face tracking
- [ ] Emotion recognition
- [ ] Face clustering

## References

- InsightFace: https://github.com/deepinsight/insightface
- ArcFace Paper: https://arxiv.org/abs/1801.07698
- SCRFD Paper: https://arxiv.org/abs/2105.04714
