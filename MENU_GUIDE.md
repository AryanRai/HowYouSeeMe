# CV Pipeline Interactive Menu Guide

## Quick Start

```bash
# Launch the menu (with system check)
./cv_menu.sh

# Or launch menu directly
./cv_pipeline_menu.sh
```

## Menu Structure

### 📋 Main Menu (Model Selection)
```
1) SAM2 - Segment Anything Model 2
2) [Future] Depth Anything
3) [Future] YOLO Object Detection
4) [Future] DINO Features
8) List Available Models
9) Stop Active Streaming
0) Exit
```

### 🎯 SAM2 Submenu (Mode Selection)
```
1) Point Mode - Segment object at a point
2) Box Mode - Segment region in bounding box
3) Multiple Points - Foreground/background points
4) Everything Mode - Segment all objects
5) Prompt Mode - Interactive prompting
0) Back to Model Selection
```

## How It Works

1. **Select Model** → Opens model-specific submenu
2. **Select Mode** → Prompts for parameters
3. **Enter Parameters** → Asks for duration & FPS
4. **Auto-Execute** → Sends request immediately
5. **Return to Menu** → Ready for next request

## Streaming Modes

- **Duration = 0**: Single frame (default)
- **Duration > 0**: Stream for N seconds
- **Duration = -1**: Continuous streaming (until stopped)

## SAM2 Modes

### 1. Point Mode 📍
Segment an object at a specific point.

**Prompts:**
- X coordinate (0-960, default: 480)
- Y coordinate (0-540, default: 270)
- Duration (-1/0/N seconds)
- FPS (if duration ≠ 0)

**Example Flow:**
```
Main Menu → 1 (SAM2) → 1 (Point Mode)
X: 640
Y: 360
Duration: 0 (single frame)
→ Auto-executes and returns to SAM2 menu
```

### 2. Box Mode 📦
Segment everything inside a bounding box.

**Prompts:**
- Box: x1,y1,x2,y2 (default: 200,150,700,450)
- Duration (-1/0/N seconds)
- FPS (if duration ≠ 0)

**Example Flow:**
```
Main Menu → 1 (SAM2) → 2 (Box Mode)
Box: 300,200,600,400
Duration: 10 (stream for 10s)
FPS: 5
→ Auto-executes and returns to SAM2 menu
```

### 3. Multiple Points Mode 🎯
Use multiple foreground/background points for refinement.

**Prompts:**
- Points: x1,y1,x2,y2,... (default: 480,270)
- Labels: 1,1,0,... (1=fg, 0=bg, default: 1)
- Duration (-1/0/N seconds)
- FPS (if duration ≠ 0)

**Example Flow:**
```
Main Menu → 1 (SAM2) → 3 (Multiple Points)
Points: 400,300,500,350,200,100
Labels: 1,1,0
Duration: 0
→ Auto-executes and returns to SAM2 menu
```

### 4. Everything Mode 🌐
Automatically segment all objects in the frame.

**Prompts:**
- Grid size: 16-64 (default: 32)
- Duration (-1/0/N seconds)
- FPS (if duration ≠ 0)

**Example Flow:**
```
Main Menu → 1 (SAM2) → 4 (Everything Mode)
Grid size: 24
Duration: -1 (continuous streaming)
FPS: 3
→ Auto-executes and returns to SAM2 menu
```

### 5. Prompt Mode 💬
Interactive prompting with custom parameters.

**Prompts:**
- Custom parameters (e.g., prompt_type=point,x=500,y=300)
- Duration (-1/0/N seconds)
- FPS (if duration ≠ 0)

**Example Flow:**
```
Main Menu → 1 (SAM2) → 5 (Prompt Mode)
Parameters: prompt_type=box,box=100,100,800,400
Duration: 5
FPS: 10
→ Auto-executes and returns to SAM2 menu
```

## Duration Modes

- **0**: Single frame (one-shot processing)
- **N > 0**: Stream for N seconds, then stop
- **-1**: Continuous streaming (until manually stopped)

## Workflow Examples

### Example 1: Quick Point Segmentation (Single Frame)
```
./cv_menu.sh
→ 1 (SAM2)
→ 1 (Point Mode)
→ X: [Enter] (use default 480)
→ Y: [Enter] (use default 270)
→ Duration: 0
→ ✅ Auto-executes, returns to SAM2 menu
```

### Example 2: Custom Box Segmentation (10 Second Stream)
```
./cv_menu.sh
→ 1 (SAM2)
→ 2 (Box Mode)
→ Box: 300,200,600,400
→ Duration: 10
→ FPS: 5
→ ✅ Auto-executes, streams for 10s
```

### Example 3: Continuous Everything Mode
```
./cv_menu.sh
→ 1 (SAM2)
→ 4 (Everything Mode)
→ Grid size: 16
→ Duration: -1
→ FPS: 3
→ ✅ Auto-executes, streams continuously
→ [Later] Main Menu → 9 (Stop Streaming)
```

### Example 4: Multiple Points Refinement
```
./cv_menu.sh
→ 1 (SAM2)
→ 3 (Multiple Points)
→ Points: 480,270,450,250,500,300
→ Labels: 1,1,1
→ Duration: 0
→ ✅ Auto-executes single frame
```

### Example 5: Custom Prompt Mode
```
./cv_menu.sh
→ 1 (SAM2)
→ 5 (Prompt Mode)
→ Parameters: prompt_type=point,x=700,y=400
→ Duration: 5
→ FPS: 10
→ ✅ Auto-executes, streams for 5s
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
