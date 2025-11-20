# Kinect v2 Coordinate Frame Correction

## Problem

The Kinect v2's optical frame doesn't match ROS coordinate conventions, causing confusing behavior:

### Symptoms
- **Yaw in real life** → Pitch in visualization
- **Moving up/down** → Forward/backward in map
- **Rotating left/right** → Tilting up/down
- Map orientation doesn't match physical movement

### Root Cause

**Kinect Optical Frame Convention:**
```
X-axis: Points RIGHT
Y-axis: Points DOWN  
Z-axis: Points FORWARD (into the scene)
```

**ROS Standard Convention (REP-103):**
```
X-axis: Points FORWARD
Y-axis: Points LEFT
Z-axis: Points UP
```

These don't match! The Kinect's Z-axis (forward) should be ROS's X-axis.

## Solution

Add a static transform to rotate the Kinect's optical frame to match ROS conventions.

### Transform Calculation

To convert from Kinect optical frame to ROS convention:

1. **Rotate -90° around X-axis** (pitch down)
   - Makes Y point left instead of down
   - Makes Z point forward instead of up

2. **Rotate -90° around Z-axis** (yaw left)  
   - Makes X point forward instead of right
   - Makes Y point left (already correct)

**Combined rotation (in radians):**
- Roll (X): -π/2 = -1.5707963267948966
- Pitch (Y): 0
- Yaw (Z): -π/2 = -1.5707963267948966

### Implementation

```bash
ros2 run tf2_ros static_transform_publisher \
    0 0 0 \
    -1.5707963267948966 0 -1.5707963267948966 \
    kinect2_rgb_optical_frame kinect2_corrected_optical_frame
```

This creates a new frame `kinect2_corrected_optical_frame` that follows ROS conventions.

## Usage

### Option 1: Use Fixed Launch Script (Recommended)

```bash
./launch_kinect2_ros2_slam_fixed_tf.sh
```

This script:
1. Starts the TF correction transform
2. Launches the Kinect bridge
3. Starts RTAB-Map using the corrected frame
4. Launches RViz

### Option 2: Manual Correction

If using the original launch script, add the transform manually:

```bash
# In one terminal
ros2 run tf2_ros static_transform_publisher \
    0 0 0 -1.5707963267948966 0 -1.5707963267948966 \
    kinect2_rgb_optical_frame kinect2_corrected_optical_frame

# In another terminal  
./launch_kinect2_ros2_slam.sh
```

Then update RTAB-Map to use `frame_id:=kinect2_corrected_optical_frame`

## Verification

### Check TF Tree

```bash
# View TF tree
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo kinect2_rgb_optical_frame kinect2_corrected_optical_frame
```

### Test Movement

With corrected frames:

| Real Movement | Map Movement | Status |
|---------------|--------------|--------|
| Move forward | X increases | ✅ Correct |
| Move left | Y increases | ✅ Correct |
| Move up | Z increases | ✅ Correct |
| Yaw left | Rotate around Z | ✅ Correct |
| Pitch up | Rotate around Y | ✅ Correct |
| Roll right | Rotate around X | ✅ Correct |

### Visual Check in RViz

1. Point Kinect forward → Point cloud should extend in +X direction
2. Move Kinect up → Odometry should show +Z movement
3. Rotate Kinect left → Should rotate around Z-axis

## Technical Details

### Rotation Matrix

The combined rotation can be represented as:

```
R = Rz(-90°) * Rx(-90°)

R = [0  0  1]
    [1  0  0]
    [0  1  0]
```

This transforms:
- Kinect X (right) → ROS Y (left) [negated]
- Kinect Y (down) → ROS Z (up) [negated]
- Kinect Z (forward) → ROS X (forward)

### Quaternion Representation

The rotation as a quaternion:
```
q = [x, y, z, w]
  = [-0.5, -0.5, -0.5, 0.5]
```

Or in Euler angles (roll, pitch, yaw):
```
roll  = -90° = -π/2
pitch = 0°
yaw   = -90° = -π/2
```

## Alternative Approaches

### Approach 1: Modify Bridge (Not Recommended)

You could modify the kinect2_bridge to publish with corrected frames, but:
- ❌ Requires changing upstream code
- ❌ Breaks compatibility
- ❌ Harder to maintain

### Approach 2: Use TF Remapping (Complex)

You could use TF remapping in launch files, but:
- ⚠️ More complex configuration
- ⚠️ Harder to debug
- ✅ More flexible for multiple sensors

### Approach 3: Static Transform (Recommended)

Using `static_transform_publisher`:
- ✅ Simple and clear
- ✅ No code changes needed
- ✅ Easy to adjust
- ✅ Standard ROS practice

## Troubleshooting

### Issue: Transform not found

**Symptom:**
```
Could not find transform from kinect2_corrected_optical_frame to map
```

**Solution:**
Make sure the static transform publisher is running:
```bash
ros2 topic echo /tf_static | grep kinect2_corrected
```

### Issue: Still wrong orientation

**Symptom:**
Movements still don't match expected directions.

**Solution:**
1. Check which frame RTAB-Map is using:
   ```bash
   ros2 param get /rtabmap/rtabmap frame_id
   ```

2. Verify it's set to `kinect2_corrected_optical_frame`

3. Check TF tree:
   ```bash
   ros2 run tf2_tools view_frames
   evince frames.pdf
   ```

### Issue: Multiple transforms conflict

**Symptom:**
```
TF_REPEATED_DATA ignoring data with redundant timestamp
```

**Solution:**
Make sure you're not running multiple static transform publishers for the same frames.

## References

- **REP-103**: Standard Units of Measure and Coordinate Conventions
  - https://www.ros.org/reps/rep-0103.html
  
- **REP-105**: Coordinate Frames for Mobile Platforms
  - https://www.ros.org/reps/rep-0105.html

- **TF2 Documentation**:
  - http://wiki.ros.org/tf2

- **Kinect v2 Specifications**:
  - Optical frame follows camera conventions (Z forward)
  - ROS expects robotics conventions (X forward)

## Summary

The Kinect v2's optical frame uses camera conventions (Z forward) while ROS uses robotics conventions (X forward). Adding a static transform that rotates -90° around X and -90° around Z corrects this mismatch, making movements in the real world match movements in the map.

**Use the fixed launch script** (`launch_kinect2_ros2_slam_fixed_tf.sh`) for correct coordinate frame behavior!
