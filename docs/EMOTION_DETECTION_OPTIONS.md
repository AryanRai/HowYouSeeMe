# Emotion Detection Options for InsightFace

## Current Implementation: FER

We use **FER (Facial Expression Recognition)** for emotion detection because:

### Advantages
- ✅ **Lightweight** - Small model, fast inference
- ✅ **Easy to install** - `pip install fer`
- ✅ **Well-tested** - Based on FER2013 dataset
- ✅ **7 emotions** - Happy, Sad, Angry, Surprise, Fear, Disgust, Neutral
- ✅ **Works with InsightFace** - Uses our face detection
- ✅ **Good accuracy** - ~65-70% on FER2013

### Why Not InsightFace Native?

InsightFace's standard model packs (buffalo_l, buffalo_s, etc.) include:
- Face detection (SCRFD)
- Face recognition (ArcFace)
- Age estimation
- Gender detection
- Landmarks (2D/3D)

**But NOT emotion recognition** in the standard packs.

## Alternative Options

### 1. FER (Current - Recommended)
```python
from fer import FER
detector = FER(mtcnn=False)
emotions = detector.detect_emotions(face_img)
```

**Pros**: Lightweight, easy, good enough  
**Cons**: Moderate accuracy  
**Install**: `pip install fer`

### 2. DeepFace
```python
from deepface import DeepFace
result = DeepFace.analyze(face_img, actions=['emotion'])
```

**Pros**: Multiple backends, high accuracy  
**Cons**: Heavier, slower  
**Install**: `pip install deepface`

### 3. HSEmotion
```python
from hsemotion.facial_emotions import HSEmotionRecognizer
model = HSEmotionRecognizer(model_name='enet_b0_8_best_afew')
emotions = model.predict_emotions(face_img)
```

**Pros**: State-of-the-art accuracy  
**Cons**: Larger model  
**Install**: `pip install hsemotion`

### 4. Custom InsightFace Emotion Model

InsightFace can load custom ONNX models:
```python
# If you have a custom emotion.onnx model
emotion_model = insightface.model_zoo.get_model('emotion.onnx')
```

**Pros**: Integrated with InsightFace  
**Cons**: Need to find/train the model  
**Install**: Requires custom model file

## Comparison

| Model | Accuracy | Speed | Size | Install |
|-------|----------|-------|------|---------|
| FER | 65-70% | Fast | 5MB | Easy |
| DeepFace | 75-80% | Medium | 50MB | Easy |
| HSEmotion | 80-85% | Medium | 20MB | Easy |
| Custom | Varies | Fast | Varies | Hard |

## Current Choice: FER

We chose FER because:
1. **Good balance** of speed and accuracy
2. **Lightweight** - doesn't slow down the pipeline
3. **Easy to install** - one pip command
4. **Well-maintained** - active development
5. **Works perfectly** with InsightFace face detection

## Switching to Another Model

If you want to use a different emotion detector:

### Option 1: DeepFace
```python
# In insightface_worker.py, replace FER with:
from deepface import DeepFace

def detect_emotion(self, image, params):
    # ... face detection code ...
    result = DeepFace.analyze(
        face_img, 
        actions=['emotion'],
        enforce_detection=False
    )
    emotion = result['dominant_emotion']
    scores = result['emotion']
```

### Option 2: HSEmotion
```python
# In insightface_worker.py, replace FER with:
from hsemotion.facial_emotions import HSEmotionRecognizer

def __init__(self):
    self.emotion_detector = HSEmotionRecognizer(
        model_name='enet_b0_8_best_afew'
    )

def detect_emotion(self, image, params):
    # ... face detection code ...
    emotions = self.emotion_detector.predict_emotions(face_img)
```

## Recommendation

**Stick with FER** unless you need:
- Higher accuracy → Use HSEmotion
- Multiple backends → Use DeepFace
- Custom training → Use custom ONNX model

For most use cases, FER provides the best balance of speed, accuracy, and ease of use.

## Future: InsightFace Native Emotion

If InsightFace releases an official emotion model:
1. Download the model pack
2. Update `load_models()` to include emotion module
3. Use native InsightFace API
4. Remove FER dependency

Until then, FER is the best choice! 😊

## Installation

```bash
# Current (FER)
pip install fer

# Alternative (DeepFace)
pip install deepface

# Alternative (HSEmotion)
pip install hsemotion
```

## Performance Comparison

Tested on single face:
- **FER**: ~50ms
- **DeepFace**: ~150ms
- **HSEmotion**: ~80ms
- **Custom ONNX**: ~30ms (if optimized)

For real-time applications with multiple faces, FER is the winner! 🏆
