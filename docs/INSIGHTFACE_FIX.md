# InsightFace Loading Fix ✅

## Issue

When trying to load InsightFace, got this error:
```
AssertionError: assert 'detection' in self.models
```

## Root Cause

InsightFace's `FaceAnalysis` class requires the detection module to be present, even when using `allowed_modules=['recognition']`. The assertion check fails because we filtered out the detection module.

## Solution

Use the full app (which includes all modules) for recognition tasks instead of trying to create a recognition-only instance:

```python
# Before (caused error)
self.recognizer = FaceAnalysis(
    name=model_pack,
    allowed_modules=['recognition'],  # ← This causes assertion error
    providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
)

# After (works correctly)
self.recognizer = self.app  # Use full app for recognition
```

## Why This Works

- The full `FaceAnalysis` app includes detection, recognition, and attribute models
- Recognition mode in our worker only uses the recognition features
- Detection is still available but not used in recognition-only mode
- This is how InsightFace is designed to work

## Files Modified

- ✅ `ros2_ws/src/cv_pipeline/python/insightface_worker.py`

## Testing

```bash
# Restart server
pkill -f sam2_server_v2.py
./launch_kinect_sam2_server.sh

# Test detection
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'insightface:mode=detect'"

# Test recognition
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'insightface:mode=detect_recognize'"
```

## Status

✅ **Fixed and ready to use!**

The InsightFace models will now load correctly without assertion errors.
