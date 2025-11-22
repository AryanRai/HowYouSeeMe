# CV Pipeline Interactive Menu Guide

## Quick Start

```bash
# Launch the menu (with system check)
./cv_menu.sh

# Or launch menu directly
./cv_pipeline_menu.sh
```

## Menu Features

### 🎯 Main Menu Options

1. **Select Model** - Choose which CV model to use
   - SAM2 (currently available)
   - Future: Depth Anything, YOLO, DINO

2. **Select Mode** - Choose processing mode for the model
   - Point Mode
   - Box Mode
   - Multiple Points Mode
   - Everything Mode

3. **Configure Parameters** - Set mode-specific parameters
   - Point: X, Y coordinates
   - Box: Bounding box (x1,y1,x2,y2)
   - Points: Multiple points and labels
   - Everything: Grid size

4. **Toggle Streaming Mode** - Enable/disable continuous processing
   - Set duration (seconds)
   - Set FPS (frames per second)

5. **Send Request** - Execute the configured request

6. **View Results** - Monitor results in real-time

7. **List Available Models** - Query available models

8. **Get Model Info** - Get information about a model

9. **Stop Streaming** - Stop active streaming

0. **Exit** - Close the menu

## SAM2 Modes

### Point Mode
Segment an object at a specific point.

**Parameters:**
- X coordinate (default: 480)
- Y coordinate (default: 270)

**Example:**
- Select mode: Point Mode
- Configure: X=640, Y=360
- Send request

### Box Mode
Segment everything inside a bounding box.

**Parameters:**
- Box coordinates: x1,y1,x2,y2 (default: 200,150,700,450)

**Example:**
- Select mode: Box Mode
- Configure: 300,200,600,400
- Send request

### Multiple Points Mode
Use multiple foreground/background points for refinement.

**Parameters:**
- Points: x1,y1,x2,y2,... (default: 480,270)
- Labels: 1,1,0,... (1=foreground, 0=background)

**Example:**
- Select mode: Multiple Points
- Configure points: 400,300,500,350,200,100
- Configure labels: 1,1,0
- Send request

### Everything Mode
Automatically segment all objects in the frame.

**Parameters:**
- Grid size: 16-64 (default: 32)

**Example:**
- Select mode: Everything Mode
- Configure: Grid size = 24
- Send request

## Streaming Mode

Enable continuous processing for video-like segmentation.

**Configuration:**
1. Toggle Streaming Mode (option 4)
2. Enable streaming
3. Set duration (e.g., 10 seconds)
4. Set FPS (e.g., 5 frames per second)
5. Send request (option 5)

**Stop Streaming:**
- Use option 9 to stop active streaming

## Workflow Examples

### Example 1: Quick Point Segmentation
```
1. Launch menu: ./cv_menu.sh
2. Select option 5 (Send Request)
   - Uses defaults: SAM2, Point mode, center point
3. View in RViz: /cv_pipeline/visualization
```

### Example 2: Custom Box Segmentation
```
1. Launch menu
2. Option 2: Select Mode → Box Mode
3. Option 3: Configure Parameters → 300,200,600,400
4. Option 5: Send Request
5. View results in RViz
```

### Example 3: Streaming Everything Mode
```
1. Launch menu
2. Option 2: Select Mode → Everything Mode
3. Option 3: Configure Parameters → Grid size = 16
4. Option 4: Toggle Streaming → Enable, 15s, 3 FPS
5. Option 5: Send Request
6. Watch live stream in RViz
7. Option 9: Stop Streaming (when done)
```

### Example 4: Multiple Points Refinement
```
1. Launch menu
2. Option 2: Select Mode → Multiple Points
3. Option 3: Configure Parameters
   - Points: 480,270,450,250,500,300
   - Labels: 1,1,1 (all foreground)
4. Option 5: Send Request
5. View refined segmentation
```

## Tips

### Before Using the Menu
- Make sure the CV Pipeline server is running
- Use `./cv_menu.sh` to auto-check and launch if needed
- Or manually run: `./launch_kinect_sam2_server.sh`

### Viewing Results
- **RViz**: Add Image display for `/cv_pipeline/visualization`
- **Terminal**: Use option 6 to monitor JSON results
- **Single result**: `ros2 topic echo /cv_pipeline/results --once`

### Parameter Ranges
- **X coordinate**: 0 to 960 (QHD width)
- **Y coordinate**: 0 to 540 (QHD height)
- **Box**: x1,y1,x2,y2 within image bounds
- **Grid size**: 16-64 (higher = more points, slower)
- **FPS**: 1-30 (higher = more frequent, more GPU usage)

### Troubleshooting

**Menu shows "Request failed":**
- Check if CV Pipeline server is running
- Verify ROS2 topics: `ros2 topic list | grep cv_pipeline`

**No visualization in RViz:**
- Send a request first (option 5)
- Check topic: `ros2 topic hz /cv_pipeline/visualization`

**Streaming not working:**
- Make sure streaming is enabled (option 4)
- Check duration and FPS are reasonable
- Stop any existing stream first (option 9)

## Keyboard Shortcuts

- **Enter**: Confirm selection / Continue
- **Ctrl+C**: Stop viewing results (option 6)
- **0**: Back / Exit from any submenu

## Color Legend

- 🟢 **Green**: Success messages
- 🔵 **Blue**: Information
- 🟡 **Yellow**: Warnings / Current values
- 🔴 **Red**: Errors
- 🔷 **Cyan**: Headers / Titles

## Advanced Usage

### Combining with Other Tools

```bash
# Terminal 1: Launch system
./launch_full_system_rviz.sh

# Terminal 2: Use menu
./cv_menu.sh

# Terminal 3: Monitor results
ros2 topic echo /cv_pipeline/results
```

### Scripting with Menu Values

The menu builds ROS2 commands that you can also use directly:

```bash
# Point mode
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:prompt_type=point,x=640,y=360'"

# Streaming
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'sam2:prompt_type=everything,stream=true,duration=10,fps=5'"
```

## Future Models

When new models are added, they will appear in the "Select Model" menu:

- **Depth Anything**: Depth estimation modes
- **YOLO**: Object detection modes
- **DINO**: Feature extraction modes
- **GroundingDINO**: Open-vocabulary detection

Each model will have its own modes and parameters!

## See Also

- `CV_PIPELINE_V2_GUIDE.md` - Complete system guide
- `sam2_modes_guide.sh` - SAM2 modes reference
- `ADD_NEW_MODEL_GUIDE.md` - Adding new models
