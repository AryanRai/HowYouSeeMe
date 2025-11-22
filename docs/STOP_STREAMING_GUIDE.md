# How to Stop CV Pipeline Streaming

There are multiple ways to stop an active streaming session in the CV Pipeline.

## Methods

### 1. Using the Menu (Recommended)
```bash
# While in the menu
Press 9 → Stop Active Streaming
```

**Advantages:**
- Interactive feedback
- Shows success/failure status
- Returns to menu for next action

### 2. Using Ctrl+C in Menu
```bash
# While in the menu, press Ctrl+C
Ctrl+C → Option 1 (Stop streaming and return to menu)
```

**Advantages:**
- Quick keyboard shortcut
- Offers multiple options (stop, exit, cancel)
- No need to navigate menu

### 3. Using Standalone Script
```bash
./stop_cv_streaming.sh
# or
./stop_sam2_stream.sh
```

**Advantages:**
- Works from any terminal
- Can be called from other scripts
- Quick one-liner

### 4. Using Direct ROS2 Command
```bash
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:stop=true'"
```

**Advantages:**
- Direct control
- Can be integrated into custom scripts
- No dependencies on menu scripts

## Verification

After stopping, verify the stream has stopped:

```bash
# Check if results are still being published
ros2 topic hz /cv_pipeline/results

# Should show "no new messages" or very low rate
```

## Troubleshooting

### Stream Won't Stop

**Problem:** Stream continues after stop command

**Solutions:**
1. Send stop command again
2. Check if server is responding:
   ```bash
   ros2 topic list | grep cv_pipeline
   ```
3. Restart the CV Pipeline server:
   ```bash
   pkill -f sam2_server_v2.py
   ./launch_kinect_sam2_server.sh
   ```

### Stop Command Fails

**Problem:** "Failed to send stop command"

**Solutions:**
1. Check if CV Pipeline server is running:
   ```bash
   pgrep -f sam2_server_v2.py
   ```
2. Verify ROS2 is working:
   ```bash
   ros2 topic list
   ```
3. Restart ROS2 daemon:
   ```bash
   ros2 daemon stop
   ros2 daemon start
   ```

### Multiple Streams Running

**Problem:** Multiple streaming sessions active

**Solution:**
Send stop command multiple times or restart the server:
```bash
# Send stop 3 times
for i in {1..3}; do
    ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
      "data: 'sam2:stop=true'"
    sleep 1
done
```

## Best Practices

1. **Always stop streaming before starting a new one**
   - Prevents resource conflicts
   - Ensures clean state

2. **Use menu option 9 for interactive stopping**
   - Provides feedback
   - Confirms success

3. **Use Ctrl+C for emergency stops**
   - Quick and accessible
   - Offers multiple options

4. **Monitor streaming status**
   ```bash
   # In a separate terminal
   ros2 topic echo /cv_pipeline/results
   ```

5. **Check for active streams before exiting**
   - Prevents orphaned processes
   - Saves GPU resources

## Duration Modes Recap

- **Duration = 0**: Single frame (no streaming, nothing to stop)
- **Duration > 0**: Timed streaming (auto-stops after duration)
- **Duration = -1**: Continuous streaming (requires manual stop)

## Quick Reference

| Method | Command | Use Case |
|--------|---------|----------|
| Menu | Option 9 | Interactive use |
| Ctrl+C | In menu | Quick stop |
| Script | `./stop_cv_streaming.sh` | From terminal |
| ROS2 | `ros2 topic pub...` | Automation |

## Examples

### Stop from Menu
```
./cv_menu.sh
→ 9 (Stop Active Streaming)
→ ✅ Confirmed
```

### Stop with Ctrl+C
```
./cv_menu.sh
[Ctrl+C]
→ 1 (Stop streaming)
→ ✅ Stopped, back to menu
```

### Stop from Terminal
```bash
# Quick stop
./stop_cv_streaming.sh

# Or direct command
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:stop=true'"
```

### Stop and Verify
```bash
# Stop
./stop_cv_streaming.sh

# Wait a moment
sleep 2

# Verify
ros2 topic hz /cv_pipeline/results
# Should show no activity
```

## Integration with Scripts

You can integrate stop commands into your own scripts:

```bash
#!/bin/bash
# Your custom script

# Start streaming
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:prompt_type=everything,stream=true,duration=-1,fps=5'"

# Do some work...
sleep 30

# Stop streaming
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:stop=true'"
```

## See Also

- `MENU_GUIDE.md` - Complete menu guide
- `CV_PIPELINE_V2_GUIDE.md` - System overview
- `sam2_modes_guide.sh` - SAM2 modes reference
