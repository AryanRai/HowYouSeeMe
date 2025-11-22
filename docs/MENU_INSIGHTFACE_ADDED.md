# InsightFace Added to CV Menu ✅

## What Was Added

InsightFace face recognition has been fully integrated into the CV Pipeline menu system!

### Main Menu Update

```
Select a Model:

  1) 🎯 SAM2 - Segment Anything Model 2
  2) ⚡ FastSAM - Faster SAM with Text Prompts
  3) 🔍 YOLO11 - Detection, Pose, Segmentation, OBB
  4) 👤 InsightFace - Face Recognition & Liveness  ← NEW!
  5) 📊 [Future] Depth Anything
  6) 🧠 [Future] DINO Features
```

### InsightFace Submenu

```
InsightFace - Face Recognition & Liveness

Select Mode:

  1) 🔍 Detect Faces - Detection only
  2) 👤 Recognize Faces - Full recognition
  3) ➕ Register New Person - Add to database
  4) 🛡️  Check Liveness - Anti-spoofing
  5) 📊 Full Analysis - Everything
  6) 🎬 Stream Recognition - Continuous

  0) ⬅️  Back to Main Menu
```

## Features

### 1. Detect Faces
- Face detection only
- Returns bounding boxes + landmarks
- Configurable detection size
- Max faces limit

### 2. Recognize Faces
- Full detection + recognition pipeline
- Adjustable similarity threshold
- Shows recognized (green) and unknown (orange) faces
- Includes age and gender

### 3. Register New Person
- Add people to database
- Interactive name input
- Instructions for best results
- Tips for multi-sample registration

### 4. Check Liveness
- Depth-based anti-spoofing
- Detects photos/screens
- Real-time verification
- Visual feedback (LIVE/SPOOF)

### 5. Full Analysis
- Complete face analysis
- Detection + recognition + liveness
- Age and gender estimation
- Maximum information extraction

### 6. Stream Recognition
- Continuous face recognition
- Configurable FPS (1-30)
- Duration control
- Real-time monitoring

## Usage

### Quick Start

```bash
# 1. Launch menu
./cv_menu.sh

# 2. Select option 4 (InsightFace)

# 3. Choose a mode:
#    - Option 3 to register yourself
#    - Option 2 to recognize faces
#    - Option 4 to check liveness
#    - Option 6 for streaming
```

### Example Workflow

1. **Register Team Members**
   - Select option 4 (InsightFace)
   - Select option 3 (Register)
   - Enter name for each person
   - Register 3-5 samples per person

2. **Test Recognition**
   - Select option 2 (Recognize)
   - Adjust threshold if needed
   - View results in RViz

3. **Stream Recognition**
   - Select option 6 (Stream)
   - Set FPS and duration
   - Monitor continuous recognition

4. **Check Liveness**
   - Select option 4 (Liveness)
   - Verify anti-spoofing works
   - Test with photos/screens

## Menu Features

### User-Friendly
- Clear descriptions for each mode
- Interactive parameter input
- Default values provided
- Instructions and tips

### Visual Feedback
- Color-coded messages
- Success/error indicators
- Progress information
- Result monitoring tips

### Flexible Configuration
- Adjustable thresholds
- Detection size control
- FPS configuration
- Duration settings

## Integration

### With Existing Menu
- Seamlessly integrated
- Consistent UI/UX
- Same navigation patterns
- Unified streaming control

### With CV Pipeline
- Uses same server
- Shares visualization topic
- Compatible with other models
- Supports streaming

## Files Modified

- ✅ `cv_pipeline_menu.sh` - Added InsightFace menu and functions

## New Functions Added

1. `show_insightface_menu()` - Main submenu
2. `insightface_detect()` - Face detection
3. `insightface_recognize()` - Face recognition
4. `insightface_register()` - Register new person
5. `insightface_liveness()` - Liveness check
6. `insightface_analyze()` - Full analysis
7. `insightface_stream()` - Streaming recognition

## Testing

```bash
# Test the menu
./cv_menu.sh

# Navigate to InsightFace
# Select option 4

# Try each mode:
# 1. Detect - Test face detection
# 2. Recognize - Test recognition
# 3. Register - Add yourself
# 4. Liveness - Check anti-spoofing
# 5. Analyze - Full analysis
# 6. Stream - Continuous recognition
```

## Tips

### For Best Results

1. **Registration**
   - Good lighting
   - Frontal face
   - Multiple samples (3-5)
   - Different angles

2. **Recognition**
   - Start with threshold 0.6
   - Lower for more matches
   - Higher for more security

3. **Liveness**
   - Ensure depth data available
   - Test with photos/screens
   - Verify real-time detection

4. **Streaming**
   - Use 5-10 FPS for smooth operation
   - Monitor RTAB-Map delay
   - Stop when done (option 9)

## Next Steps

1. **Install InsightFace** (if not done)
   ```bash
   ./install_insightface.sh
   ```

2. **Restart Server**
   ```bash
   pkill -f sam2_server_v2.py
   ./launch_kinect_sam2_server.sh
   ```

3. **Launch Menu**
   ```bash
   ./cv_menu.sh
   ```

4. **Test InsightFace**
   - Select option 4
   - Try each mode
   - Register yourself
   - Test recognition

## Summary

✅ **InsightFace fully integrated** into CV menu  
✅ **6 modes available** with interactive configuration  
✅ **User-friendly interface** with clear instructions  
✅ **Consistent with existing menu** design  
✅ **Ready to use** - just launch and test!  

The menu now provides easy access to all face recognition features! 🎉
