# 🚀 Restart Server and Test - All Fixes Applied!

## What's Fixed

✅ **Non-blocking streaming** - RTAB-Map stays in sync  
✅ **Frame throttling** - Prevents pipeline overload  
✅ **Cooldown period** - Clean recovery after streaming  
✅ **Cooldown bypass** - Streaming requests work instantly  
✅ **Proper cleanup** - No stuck states  
✅ **Force reset** - Emergency recovery  

---

## Apply All Fixes NOW

### Step 1: Restart Server
```bash
pkill -f sam2_server_v2.py
./launch_kinect_sam2_server.sh
```

Wait for: `CV Pipeline Server ready!`

### Step 2: Test Streaming Transitions
```bash
# Start detection
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'yolo11:task=detect,stream=true,duration=5,fps=5'"

# Wait a bit
sleep 3

# Stop
./stop_cv_streaming.sh

# Start pose IMMEDIATELY (no wait!)
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'yolo11:task=pose,stream=true,duration=5,fps=5'"
```

### Step 3: Verify
✅ Both streams should work  
✅ No cooldown blocking  
✅ RTAB-Map delay stays low  
✅ Clean transitions  

---

## Quick Test Commands

### Test 1: Rapid Streaming Switch
```bash
# Detection for 3 seconds
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'yolo11:task=detect,stream=true,duration=3,fps=5'"

sleep 4

# Pose for 3 seconds (instant switch!)
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'yolo11:task=pose,stream=true,duration=3,fps=5'"
```

### Test 2: Stop and Switch
```bash
# Start detection
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'yolo11:task=detect,stream=true,duration=999,fps=5'"

# Stop after a few seconds
./stop_cv_streaming.sh

# Start pose immediately
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'yolo11:task=pose,stream=true,duration=999,fps=5'"
```

### Test 3: Single Frame After Streaming
```bash
# Stream for 5 seconds
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'yolo11:task=detect,stream=true,duration=5,fps=5'"

sleep 6

# Single frame (respects cooldown)
./stop_cv_streaming.sh  # Handles cooldown automatically
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
    "data: 'sam2:mode=everything'"
```

---

## Expected Behavior

### Streaming → Streaming (Instant!)
```
Stop Stream → Start New Stream (0s wait)
```

### Streaming → Single Frame (0.5s cooldown)
```
Stop Stream → Wait 0.5s → Single Frame
```

### Using stop_cv_streaming.sh (Automatic)
```
./stop_cv_streaming.sh → Handles timing → Ready!
```

---

## Monitoring

### Good Signs ✅
```
Request received: yolo11:task=pose,stream=true...
🎬 Started streaming: yolo11 for X.0s @ Y FPS
Still processing previous frame, skipping...
Throttling: too soon since last frame
Streaming stopped, entering 0.5s cooldown period
```

### Should NOT See ❌
```
In cooldown period, wait X.Xs more  ← Only for single-frame after streaming
```

---

## If Something Goes Wrong

### Server Not Responding
```bash
./reset_cv_server.sh
```

### Complete Restart
```bash
pkill -f sam2_server_v2.py
pkill -f kinect2_bridge_node
pkill -f rtabmap
./launch_kinect_sam2_server.sh
```

### Check Logs
Look at the terminal running the server for detailed messages.

---

## Summary of Changes

1. **Non-blocking processing** - Skips frames if busy
2. **Frame throttling** - Min 50ms between frames
3. **Cooldown period** - 0.5s after stopping (single-frame only)
4. **Cooldown bypass** - Streaming requests work instantly
5. **Proper cleanup** - Waits for in-progress frames
6. **Force reset** - Emergency recovery

---

## Ready to Go!

```bash
# Restart server
pkill -f sam2_server_v2.py && ./launch_kinect_sam2_server.sh

# Test it
# Use the menu: ./cv_menu.sh
# Or manual commands above
```

All fixes applied - enjoy seamless streaming! 🎉
