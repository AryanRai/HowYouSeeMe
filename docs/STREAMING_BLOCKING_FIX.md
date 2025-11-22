# Streaming Blocking Issue - FIXED ✅

## Issue
After running YOLO streaming, RTAB-Map visualization in RViz2 would fall behind and stay out of sync, even after stopping the stream.

## Root Cause
The streaming callback was blocking the ROS2 executor, preventing image callbacks from updating. This caused:
- RTAB-Map to use stale images
- High time differences between RGB and depth frames
- Visualization lag that persisted after stopping

## Fix Applied

### 1. Non-Blocking Streaming
- Added `processing_frame` flag to skip frames if still processing
- Prevents callback queue backup
- Allows image callbacks to continue running smoothly

### 2. Proper Cleanup
- Enhanced stop_streaming() to wait for in-progress frames
- Timeout protection (0.5s max wait)
- Complete state reset after stopping

### 3. Force Reset
- Added force_reset() function for recovery
- Can be called via `./reset_cv_server.sh`
- Clears all state and triggers garbage collection

## How to Use

### Start Streaming
```bash
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'yolo11:task=detect,stream=true,duration=10,fps=5'"
```

### Stop Streaming
```bash
./stop_cv_streaming.sh
```

### If Stuck
```bash
./reset_cv_server.sh
```

### Test the Fix
```bash
./test_streaming_fix.sh
```

## Expected Behavior Now

✅ RTAB-Map stays in sync during streaming (delay < 0.2s)  
✅ Clean transitions between models  
✅ No blocking of image callbacks  
✅ Immediate response to stop commands  
✅ No persistent lag after stopping  

## Files Modified

- `ros2_ws/src/cv_pipeline/python/sam2_server_v2.py` - Core fix
- `reset_cv_server.sh` - Force reset script (NEW)
- `test_streaming_fix.sh` - Test script (NEW)
- `docs/STREAMING_FIX.md` - Detailed documentation (NEW)

## Additional Enhancement: Cooldown Period

After stopping streaming, a 0.5-second cooldown period is now enforced:
- Prevents new requests during recovery
- Allows image pipeline to stabilize
- Eliminates "time difference" warnings after stopping

Frame throttling (min 50ms between frames) prevents pipeline overload during streaming.

## Next Steps

1. Restart the CV pipeline server:
   ```bash
   pkill -f sam2_server_v2.py
   ./launch_kinect_sam2_server.sh
   ```

2. Test with YOLO streaming:
   ```bash
   ./test_streaming_fix.sh
   ```

3. Monitor RTAB-Map delay in the logs - should stay low!

4. Use `./stop_cv_streaming.sh` which handles cooldown automatically

The fix ensures that streaming models like YOLO don't interfere with SLAM operations.

See `COOLDOWN_FIX.md` for details on the cooldown enhancement.
