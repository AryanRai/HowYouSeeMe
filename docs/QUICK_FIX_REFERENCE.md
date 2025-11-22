# Quick Fix Reference - Streaming Blocking Issue

## Problem Summary
YOLO streaming was blocking RTAB-Map, causing visualization lag that persisted after stopping.

## Solution Applied ✅
Modified `sam2_server_v2.py` to use non-blocking streaming with proper cleanup.

---

## Quick Commands

### Restart Server (Apply Fix)
```bash
pkill -f sam2_server_v2.py
./launch_kinect_sam2_server.sh
```

### Test the Fix
```bash
./test_no_blocking.sh
```

### If Server Gets Stuck
```bash
./reset_cv_server.sh
```

### Stop Streaming
```bash
./stop_cv_streaming.sh
```

---

## What Changed

1. **Non-blocking processing**: Skips frames if still busy
2. **Proper cleanup**: Waits for in-progress frames before stopping
3. **Force reset**: Recovery mechanism if stuck
4. **Error handling**: Auto-stops on errors

---

## Expected Behavior

### Before Fix ❌
- RTAB-Map delay increases during YOLO streaming
- Lag persists after stopping
- "Time difference" warnings
- Blocking behavior

### After Fix ✅
- RTAB-Map delay stays low (< 0.2s)
- Clean transitions between models
- No persistent lag
- Smooth operation

---

## Monitoring

### Check Server Status
```bash
pgrep -f sam2_server_v2.py
```

### Watch RTAB-Map Delay
Look for lines like:
```
delay=0.1234s  ← Should stay low!
```

### Check for Frame Skipping (Normal)
Server logs may show:
```
Still processing previous frame, skipping...
```
This is GOOD - it means the fix is working!

---

## Troubleshooting

### Server Not Responding
```bash
./reset_cv_server.sh
```

### Still Having Issues
```bash
# Full restart
pkill -f sam2_server_v2.py
pkill -f kinect2_bridge_node
pkill -f rtabmap
./launch_kinect_sam2_server.sh
```

### Check Logs
```bash
# Run server in foreground to see all logs
python3 ros2_ws/src/cv_pipeline/python/sam2_server_v2.py
```

---

## Files Modified
- `ros2_ws/src/cv_pipeline/python/sam2_server_v2.py` ← Main fix

## New Files
- `reset_cv_server.sh` ← Force reset
- `test_no_blocking.sh` ← Comprehensive test
- `test_streaming_fix.sh` ← Quick test
- `docs/STREAMING_FIX.md` ← Detailed docs
- `STREAMING_BLOCKING_FIX.md` ← Summary
- `QUICK_FIX_REFERENCE.md` ← This file

---

## Next Steps

1. **Restart server** to apply fix
2. **Run test** to verify: `./test_no_blocking.sh`
3. **Monitor** RTAB-Map delay during normal use
4. **Report** if any issues persist

The fix ensures streaming models don't interfere with SLAM operations! 🎉
