# Face Recognition System - Implementation Plan

## Overview

Building a complete face recognition system with:
1. **Face Detection** - YOLO11 (already integrated) + InsightFace SCRFD
2. **Face Recognition** - InsightFace ArcFace (ResNet50)
3. **Liveness Detection** - Depth-based + RGB anti-spoofing (separate mode)
4. **Storage** - Simple JSON/pickle for now, upgradeable to Redis/PostgreSQL later

## Architecture

```
Kinect RGB + Depth
        ↓
┌───────────────────────────────────────┐
│   Face Detection (YOLO11 or SCRFD)   │
│   - Detect face bounding boxes        │
│   - Extract face regions              │
└───────────────────────────────────────┘
        ↓
┌───────────────────────────────────────┐
│   Face Recognition (ArcFace)          │
│   - Generate 512-dim embeddings       │
│   - Compare with database             │
│   - Identify or register new face     │
└───────────────────────────────────────┘
        ↓
┌───────────────────────────────────────┐
│   Liveness Detection (Optional)       │
│   - Depth variance check              │
│   - RGB anti-spoofing model           │
│   - Blink detection                   │
└───────────────────────────────────────┘
        ↓
┌───────────────────────────────────────┐
│   Face Database                       │
│   - Store embeddings                  │
│   - Metadata (name, timestamps)       │
│   - Encounter history                 │
└───────────────────────────────────────┘
```

## Components

### 1. Face Detection
**Options:**
- **YOLO11** (already have) - Fast, good for real-time
- **SCRFD** (InsightFace) - More accurate, optimized for faces

**Decision**: Use YOLO11 for detection, add SCRFD as alternative

### 2. Face Recognition
**Model**: InsightFace ArcFace
- **buffalo_l**: ResNet50@WebFace600K (326MB)
  - 99.83% on LFW
  - 512-dim embeddings
  - Good balance of speed/accuracy

**Process**:
1. Crop face from detection bbox
2. Align face using 5 landmarks
3. Generate embedding (512-dim vector)
4. Compare with database using cosine similarity
5. Match if similarity > threshold (e.g., 0.6)

### 3. Liveness Detection
**Modes**:

#### Mode A: Depth-Based (Simple, Fast)
```python
def check_liveness_depth(face_depth_map):
    # Real face has depth variance
    depth_variance = np.var(face_depth_map)
    depth_range = np.max(face_depth_map) - np.min(face_depth_map)
    
    if depth_variance < threshold or depth_range < 10mm:
        return False  # Likely spoof (flat surface)
    return True  # Likely real
```

#### Mode B: RGB Anti-Spoofing (InsightFace FAS)
- MiniFASNet or FastAntiSpoof
- Detects printed photos, screen replay
- Works with RGB only

#### Mode C: Combined (Best)
- Depth variance check
- RGB anti-spoofing
- Blink detection (optional)

### 4. Face Database Structure

```python
{
    "person_id_001": {
        "name": "John Doe",
        "embeddings": [
            np.array([...]),  # Multiple samples for robustness
            np.array([...]),
            np.array([...])
        ],
        "metadata": {
            "first_seen": "2024-11-23T10:30:00",
            "last_seen": "2024-11-23T15:45:00",
            "encounter_count": 15,
            "confidence_scores": [0.95, 0.92, 0.98, ...],
            "locations": ["living_room", "kitchen", ...]
        },
        "liveness_verified": True
    }
}
```

## Integration with CV Pipeline

### New Model: `insightface`

**Modes**:
1. `detect` - Face detection only (SCRFD)
2. `recognize` - Full recognition pipeline
3. `register` - Register new face
4. `liveness` - Liveness detection only
5. `analyze` - Full analysis (detect + recognize + liveness)

**Parameters**:
- `mode`: detect|recognize|register|liveness|analyze
- `name`: Person name (for registration)
- `threshold`: Similarity threshold (default 0.6)
- `det_size`: Detection size (default 640)
- `liveness_check`: true|false

**Example Requests**:
```bash
# Detect faces
insightface:mode=detect,det_size=640

# Recognize faces
insightface:mode=recognize,threshold=0.6

# Register new person
insightface:mode=register,name=John_Doe

# Check liveness
insightface:mode=liveness

# Full analysis
insightface:mode=analyze,liveness_check=true
```

## Implementation Steps

### Phase 1: Basic Recognition (Now)
1. ✅ Install InsightFace
2. ✅ Create `insightface_worker.py`
3. ✅ Integrate with ModelManager
4. ✅ Basic face detection + recognition
5. ✅ Simple file-based storage

### Phase 2: Liveness Detection (Next)
1. Implement depth-based liveness
2. Add RGB anti-spoofing
3. Combine both methods
4. Add to menu as separate mode

### Phase 3: Advanced Features (Later)
1. Upgrade to Redis/PostgreSQL
2. Add FAISS for fast similarity search
3. Face tracking across frames
4. Multi-face recognition
5. Age/gender estimation
6. Emotion recognition

## File Structure

```
ros2_ws/src/cv_pipeline/python/
├── insightface_worker.py          # Main InsightFace worker
├── face_database.py                # Database management
├── liveness_detector.py            # Liveness detection
└── face_utils.py                   # Utility functions

data/
└── faces/
    ├── database.pkl                # Face embeddings database
    ├── metadata.json               # Face metadata
    └── images/                     # Optional: store face images
        ├── person_001/
        ├── person_002/
        └── ...
```

## Dependencies

```bash
pip install insightface onnxruntime-gpu
```

## Performance Targets

- **Detection**: < 50ms per frame
- **Recognition**: < 100ms per face
- **Liveness**: < 50ms (depth) or < 200ms (RGB+depth)
- **Total**: < 300ms for full pipeline

## Security Considerations

1. **Liveness Detection**: Mandatory for authentication
2. **Threshold Tuning**: Balance false accepts vs false rejects
3. **Multi-sample Registration**: Require 3-5 samples per person
4. **Confidence Scoring**: Track and log confidence levels
5. **Audit Trail**: Log all recognition attempts

## Future Enhancements

1. **Vector Database**: FAISS or Milvus for fast search
2. **Persistent Storage**: PostgreSQL with pgvector
3. **Face Tracking**: Track faces across frames
4. **Re-identification**: Match faces across sessions
5. **Privacy**: Face anonymization options
6. **Edge Deployment**: Optimize for Jetson

## Testing Plan

1. **Unit Tests**: Each component separately
2. **Integration Tests**: Full pipeline
3. **Performance Tests**: Speed benchmarks
4. **Accuracy Tests**: Recognition accuracy
5. **Liveness Tests**: Spoof detection rate
6. **Stress Tests**: Multiple faces, long sessions

## Next Steps

1. Install InsightFace
2. Create basic worker
3. Test with Kinect images
4. Integrate with menu
5. Add liveness detection

Ready to start implementation! 🚀
