# Cooldown Period Fix - Enhanced Streaming Stop

## Additional Issue Found
After stopping YOLO streaming, RTAB-Map would show "time difference" warnings for a few seconds, indicating the image pipeline was still recovering from the heavy processing load.

## Root Cause
During streaming, the CV pipeline processes images rapidly, which can cause:
- Image callback queue buildup
- Timing desynchronization between RGB and depth frames
- Residual processing effects after stopping

## Enhanced Solution

### 1. Cooldown Period After Stopping
When streaming stops, the server now enters a 0.5-second cooldown period:
- Prevents new requests during recovery
- Allows image callbacks to stabilize
- Lets RTAB-Map catch up with fresh frames

```python
# Set cooldown period to let image pipeline stabilize
self.cooldown_until = time.time() + 0.5  # 500ms cooldown
```

### 2. Frame Throttling During Streaming
Added minimum time between processed frames (50ms):
- Prevents overwhelming the pipeline
- Reduces callback queue buildup
- Maintains smooth operation

```python
# Throttle processing to avoid overwhelming the pipeline
if current_time - self.last_processed_time < 0.05:  # Min 50ms between frames
    return
```

### 3. Processing Time Tracking
Track when frames were last processed:
- Helps with throttling
- Prevents rapid-fire processing
- Ensures stable frame rate

## Usage

### Stop Streaming (Automatic Cooldown)
```bash
./stop_cv_streaming.sh
```

The script now:
1. Sends stop command
2. Waits 0.6 seconds for cooldown
3. Confirms ready for new requests

### Switch Between Streaming Models
```bash
# Stop current stream
./stop_cv_streaming.sh

# Start new stream immediately (bypasses cooldown)
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'yolo11:task=pose,stream=true,duration=10,fps=5'"
```

### Manual Stop
```bash
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'sam2:stop=true'"

# For single-frame processing, wait 0.5 seconds
sleep 0.5

# For streaming, no wait needed (bypasses cooldown)
```

## Benefits

✅ **No more time difference warnings** after stopping  
✅ **Smooth pipeline recovery** with cooldown period  
✅ **Throttled processing** prevents overload  
✅ **Stable frame rates** during streaming  
✅ **Clean transitions** between operations  

## Technical Details

### State Variables Added
```python
self.last_processed_time = 0      # Track last processing time
self.cooldown_until = 0           # Cooldown period after stopping
```

### Cooldown Check
```python
# Check cooldown period
if time.time() < self.cooldown_until:
    remaining = self.cooldown_until - time.time()
    self.get_logger().warn(f'In cooldown period, wait {remaining:.1f}s more')
    return
```

### Throttling Check
```python
# Throttle processing to avoid overwhelming the pipeline
current_time = time.time()
if current_time - self.last_processed_time < 0.05:  # Min 50ms between frames
    self.get_logger().debug('Throttling: too soon since last frame')
    return
```

## Expected Behavior

### Before Cooldown Fix
```
[Stop Command] ──► [Streaming Stops] ──► [Time Warnings for 2-3s] ──► [Stabilizes]
```

### After Cooldown Fix
```
[Stop Command] ──► [Streaming Stops] ──► [0.5s Cooldown] ──► [Ready!]
                                          (No warnings)
```

## Monitoring

### Good Signs ✅
- "Streaming stopped, entering 0.5s cooldown period"
- No time difference warnings after stopping
- RTAB-Map delay stays low throughout
- Smooth transitions

### If You See Warnings
- Wait for cooldown to complete (0.5s)
- Check if trying to send requests too quickly
- Use `./stop_cv_streaming.sh` which handles timing automatically

## Testing

```bash
# Test with automatic cooldown handling
./test_no_blocking.sh

# Or manual test
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'yolo11:task=detect,stream=true,duration=5,fps=5'"
sleep 6
./stop_cv_streaming.sh  # Handles cooldown automatically
```

## Summary

The cooldown period ensures that after intensive streaming operations, the image pipeline has time to stabilize before accepting new requests. Combined with frame throttling during streaming, this provides smooth, stable operation without overwhelming the system.

**Result**: No more time difference warnings, clean stops, and stable SLAM operation! 🎉
