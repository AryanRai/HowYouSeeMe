# Quick Stop Reference

## 🛑 Stop Streaming NOW

### Fastest Methods

**1. From Menu:**
```
Press 9 → Enter
```

**2. Ctrl+C in Menu:**
```
Ctrl+C → 1 → Enter
```

**3. From Terminal:**
```bash
./stop_cv_streaming.sh
```

**4. Direct Command:**
```bash
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String "data: 'sam2:stop=true'"
```

## 📋 All Stop Scripts

| Script | Description |
|--------|-------------|
| `stop_cv_streaming.sh` | Main stop script (recommended) |
| `stop_sam2_stream.sh` | Legacy SAM2 stop script |
| Menu Option 9 | Interactive stop from menu |
| Ctrl+C in menu | Emergency stop with options |

## ✅ Verify Stopped

```bash
# Check if still streaming
ros2 topic hz /cv_pipeline/results

# Should show "no new messages"
```

## 🔧 If Stop Fails

```bash
# 1. Try again
./stop_cv_streaming.sh

# 2. Check server
pgrep -f sam2_server_v2.py

# 3. Restart server
pkill -f sam2_server_v2.py
./launch_kinect_sam2_server.sh
```

## 💡 Tips

- **Duration = -1**: Requires manual stop
- **Duration > 0**: Auto-stops after time
- **Duration = 0**: No streaming (nothing to stop)

## 🚀 Quick Actions

```bash
# Stop and check
./stop_cv_streaming.sh && sleep 2 && ros2 topic hz /cv_pipeline/results

# Force stop (send 3 times)
for i in {1..3}; do ./stop_cv_streaming.sh; sleep 1; done

# Stop and restart server
./stop_cv_streaming.sh && pkill -f sam2_server_v2.py && ./launch_kinect_sam2_server.sh
```
