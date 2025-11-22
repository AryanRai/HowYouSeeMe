# CV Pipeline Streaming Fix

## Problem
After running YOLO streaming, the RTAB-Map visualization in RViz2 would fall behind and become out of sync, even after stopping the stream. This was caused by the streaming callback blocking the ROS2 executor, preventing image callbacks from updating.

## Root Cause
The `stream_callback()` was processing frames synchronously, which blocked the executor thread. This prevented:
- Image callbacks from receiving new frames
- RTAB-Map from getting fresh data
- Proper cleanup when stopping streams

## Solution

### 1. Non-Blocking Frame Processing
Added a `processing_frame` flag that:
- Skips new frames if previous frame is still processing
- Prevents callback queue from backing up
- Allows image callbacks to continue running

```python
# Skip if still processing previous frame (non-blocking)
if self.processing_frame:
    self.get_logger().debug('Still processing previous frame, skipping...')
    return
```

### 2. Proper Cleanup on Stop
Enhanced `stop_streaming()` to:
- Wait for in-progress frames to complete
- Timeout after 0.5 seconds if stuck
- Reset all state variables
- Ensure timer is properly destroyed

```python
# Wait for any in-progress frame to complete
max_wait = 50  # 50 * 0.01 = 0.5 seconds max
wait_count = 0
while self.processing_frame and wait_count < max_wait:
    time.sleep(0.01)
    wait_count += 1
```

### 3. Force Reset Command
Added `force_reset()` function that can be called if server gets stuck:
- Forcefully stops streaming
- Clears all state
- Triggers garbage collection
- Can be called via: `./reset_cv_server.sh`

## Usage

### Normal Operation
```bash
# Start streaming
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'yolo11:task=detect,stream=true,duration=10,fps=5'"

# Stop streaming
./stop_cv_streaming.sh
```

### If Server Gets Stuck
```bash
# Force reset
./reset_cv_server.sh

# Or restart server
pkill -f sam2_server_v2.py
./launch_kinect_sam2_server.sh
```

### Testing the Fix
```bash
# Run automated test
./test_streaming_fix.sh
```

## Key Changes

1. **sam2_server_v2.py**:
   - Added `processing_frame` flag
   - Modified `stream_callback()` to skip if busy
   - Enhanced `stop_streaming()` with proper wait
   - Added `force_reset()` function
   - Added error handling in `request_callback()`

2. **New Scripts**:
   - `reset_cv_server.sh` - Force reset server state
   - `test_streaming_fix.sh` - Test streaming behavior

## Benefits

- ✅ RTAB-Map stays in sync during streaming
- ✅ Clean transitions between models
- ✅ No blocking of image callbacks
- ✅ Proper cleanup on stop
- ✅ Recovery mechanism if stuck
- ✅ Better error handling

## Verification

After applying the fix, you should see:
1. RTAB-Map delay stays low (< 0.2s) during streaming
2. Smooth transitions between different models
3. No warnings about high time differences
4. Immediate response to stop commands

Check the logs:
```bash
# Should see "Still processing previous frame, skipping..." if frames are dropped
# Should NOT see long delays or blocking behavior
ros2 topic echo /cv_pipeline/results
```
