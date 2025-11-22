# Cooldown Bypass for Streaming - Fixed! ✅

## Issue
After stopping a stream, trying to start a new stream would fail because the cooldown period blocked ALL requests, including streaming requests.

## Root Cause
The cooldown check happened before parsing the request type, so it blocked:
- ❌ Single-frame processing (intended)
- ❌ Streaming requests (NOT intended!)
- ❌ Stop commands (NOT intended!)
- ❌ Reset commands (NOT intended!)

## Solution
Moved the cooldown check to AFTER parsing the request, so it only applies to single-frame processing:

```python
# Parse request first
parts = msg.data.split(':')
model_name = parts[0]
params = parse_params(parts[1])

# Handle commands that bypass cooldown
if params.get('stop') == 'true':
    self.stop_streaming()  # ✅ Bypass cooldown
    return

if 'stream' in params:
    self.start_streaming(...)  # ✅ Bypass cooldown
    return

# NOW check cooldown (only for single-frame)
if time.time() < self.cooldown_until:
    return  # Only blocks single-frame processing
```

## What Bypasses Cooldown

✅ **Streaming requests** - `stream=true`  
✅ **Stop commands** - `stop=true`  
✅ **Reset commands** - `reset=true`  
✅ **List models** - `list_models=true`  
✅ **Model info** - `model_info=true`  

❌ **Single-frame processing** - Regular requests (respects cooldown)

## Usage

### Switch Streaming Models (Now Works!)
```bash
# Stop current stream
./stop_cv_streaming.sh

# Start new stream IMMEDIATELY (no wait needed!)
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'yolo11:task=pose,stream=true,duration=10,fps=5'"
```

### Single-Frame After Streaming
```bash
# Stop streaming
./stop_cv_streaming.sh

# Wait for cooldown (0.5s)
sleep 0.5

# Now run single-frame
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'sam2:mode=everything'"
```

### Or Use the Script (Handles Timing)
```bash
# Stop streaming
./stop_cv_streaming.sh  # Waits 0.6s automatically

# Now ready for anything
```

## Benefits

✅ **Seamless streaming transitions** - Switch models instantly  
✅ **No artificial delays** - Streaming handles its own cleanup  
✅ **Protected single-frame** - Still gets cooldown protection  
✅ **Emergency commands work** - Stop/reset always available  

## Technical Details

### Request Processing Order
```
1. Parse request
2. Check special commands (bypass cooldown)
   - stop
   - reset
   - list_models
   - model_info
3. Check streaming (bypass cooldown)
4. Check cooldown (only for single-frame)
5. Process single-frame
```

### Why Streaming Bypasses Cooldown
Streaming requests have their own cleanup logic:
- `start_streaming()` stops existing streams
- Waits for cleanup (0.6s)
- Clears cooldown flag
- Starts new stream

So streaming doesn't need the cooldown protection!

## Testing

```bash
# Test rapid streaming switches
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'yolo11:task=detect,stream=true,duration=5,fps=5'"

sleep 2

# Stop and switch immediately
./stop_cv_streaming.sh
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'yolo11:task=pose,stream=true,duration=5,fps=5'"

# Should work instantly!
```

## Summary

The cooldown period now only applies to single-frame processing, allowing seamless transitions between streaming models while still protecting the pipeline during recovery from intensive operations.

**Result**: You can now switch between streaming models instantly! 🎉
