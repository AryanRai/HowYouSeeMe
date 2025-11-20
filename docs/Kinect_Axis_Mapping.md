# Kinect v2 Axis Mapping Guide

## Understanding Kinect Coordinate System

The Kinect v2 uses a **camera-centric coordinate system** which is different from typical robotics conventions.

### Kinect v2 Coordinate System

When looking at the Kinect from behind (user's perspective):

```
        Kinect Front View
        ┌─────────────┐
        │   KINECT    │
        │  ●     ●    │  ← Cameras
        └─────────────┘
             ↑
             │ +Y (UP)
             │
        ─────┼───── +X (RIGHT)
            /
           / +Z (FORWARD into scene)
```

**Axes:**
- **+X**: Points to the RIGHT
- **+Y**: Points UP
- **+Z**: Points FORWARD (into the scene)

### ROS Standard (REP-103)

ROS robotics convention:

```
        Robot Top View
             ↑ +X (FORWARD)
             │
        ─────┼───── +Y (LEFT)
            /
           / +Z (UP)
```

**Axes:**
- **+X**: Points FORWARD
- **+Y**: Points LEFT
- **+Z**: Points UP

### The Mismatch

| Movement | Kinect Axis | ROS Axis | What You See in RViz |
|----------|-------------|----------|----------------------|
| Move Kinect forward | +Z | +X | Moves in +Z direction |
| Move Kinect right | +X | -Y | Moves in +X direction |
| Move Kinect up | +Y | +Z | Moves in +Y direction |
| Rotate left (yaw) | Around +Y | Around +Z | Rotates around +Y |
| Tilt up (pitch) | Around +X | Around +Y | Rotates around +X |

## Why This Happens

The Kinect follows **camera conventions** (used in computer vision):
- Z-axis points where the camera is looking
- X-axis is horizontal (right)
- Y-axis is vertical (up)

ROS follows **robotics conventions** (used for mobile robots):
- X-axis points where the robot moves forward
- Y-axis points to the robot's left
- Z-axis points up

## Solutions

### Option 1: Mental Remapping (Easiest)

Just remember the mapping:
- **Kinect forward** = **RViz Z-axis**
- **Kinect right** = **RViz X-axis**  
- **Kinect up** = **RViz Y-axis**

The SLAM works perfectly fine, you just need to mentally map the axes.

### Option 2: Rotate RViz View

In RViz, rotate the view so it matches your mental model:
1. In RViz, use mouse to rotate the view
2. Set "Fixed Frame" to `kinect2_link`
3. Adjust camera angle to match your perspective

### Option 3: Use Kinect Horizontally

Mount the Kinect rotated 90° so its Z-axis aligns with robot's X-axis:
- Requires physical mounting solution
- Need to update TF transforms
- Most "correct" but requires hardware changes

### Option 4: Accept It (Recommended)

The coordinate system doesn't affect SLAM quality:
- ✅ Odometry works correctly
- ✅ Map building works correctly
- ✅ Loop closures work correctly
- ⚠️ Just visualization axes are different

**The SLAM doesn't care about axis names, only relative transformations!**

## Practical Impact

### What Works Fine
- ✅ **SLAM accuracy**: Not affected at all
- ✅ **Odometry quality**: Works perfectly
- ✅ **Map building**: Correct geometry
- ✅ **Loop closures**: Detected properly
- ✅ **Navigation**: Can navigate correctly

### What's Confusing
- ⚠️ **Visualization**: Axes don't match intuition
- ⚠️ **Debugging**: Need to remember mapping
- ⚠️ **Integration**: Other nodes need to know frame

## Testing Your Setup

Run this to see the actual coordinate system:

```bash
# Start the bridge
./test_kinect2_ros2.sh

# In another terminal, check TF
ros2 run tf2_ros tf2_echo map kinect2_link

# Move the Kinect and watch the transform change
# - Move forward: Z translation increases
# - Move right: X translation increases  
# - Move up: Y translation increases
```

## For Navigation/Integration

If you're integrating with Nav2 or other ROS packages:

1. **Document your frame**: Tell other packages you're using `kinect2_link`
2. **Provide transforms**: Publish TF from `base_link` to `kinect2_link`
3. **Use frame_id**: Always specify `frame_id:=kinect2_link` in launch files

Example TF for robot integration:
```bash
# If Kinect is mounted on a robot
ros2 run tf2_ros static_transform_publisher \
    0.1 0 0.5 0 0 0 \
    base_link kinect2_link
```

## Summary

**The "wrong" orientation is actually correct for a camera!**

- Kinect uses camera conventions (Z forward)
- ROS uses robot conventions (X forward)
- Both are valid, just different standards
- SLAM quality is unaffected
- Just remember the axis mapping when visualizing

**Recommendation**: Use the system as-is. The coordinate system doesn't affect SLAM performance, only how you interpret the visualization.

## References

- **REP-103**: ROS coordinate conventions
  - https://www.ros.org/reps/rep-0103.html
- **OpenCV**: Uses same convention as Kinect
  - https://docs.opencv.org/master/d9/d0c/group__calib3d.html
- **Kinect v2 SDK**: Microsoft's coordinate system documentation
