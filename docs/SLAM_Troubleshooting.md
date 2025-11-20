# RTAB-Map SLAM Troubleshooting Guide

## Common Issues and Solutions

### Issue 1: Node IDs Jumping Around (e.g., 33 → 1150 → 33)

**Symptom:**
In RViz, you see node IDs jumping to high numbers (like 1150) then back to sequential numbers.

**Cause:**
This is RTAB-Map's **loop closure detection** attempting to match current location with previously visited areas. The warning message confirms this:
```
[ WARN] Rtabmap.cpp:4603::process() Republishing data of requested node(s) 440 439
```

**Solutions:**

1. **If loop closures are correct** (you actually returned to a previous location):
   - This is normal behavior - RTAB-Map is working correctly
   - The jumps help verify you're in the same place

2. **If loop closures are false** (you're in a new location):
   - Increase loop closure threshold: `--Rtabmap/LoopThr 0.15` (higher = stricter)
   - Increase loop ratio: `--Rtabmap/LoopRatio 0.9` (higher = more confident)
   - These are now set in the launch script

### Issue 2: Low Odometry Quality (< 50)

**Symptom:**
```
Odom: quality=25, std dev=93.390707m|0.084852rad
Odom: quality=39, std dev=44.494574m|0.074679rad
```

Quality should be 70-120 for good tracking. Low quality (< 50) with high std dev (> 30m) indicates poor tracking.

**Common Causes:**

1. **Poor Environment:**
   - ❌ Blank walls (no depth features)
   - ❌ Reflective surfaces (windows, mirrors, shiny floors)
   - ❌ Too close (< 0.5m) or too far (> 4m)
   - ❌ Moving too fast
   - ✅ Textured surfaces with depth variation
   - ✅ Furniture, objects, corners
   - ✅ 0.5-4 meter range
   - ✅ Slow, smooth movements

2. **Bad Depth Frames:**
   ```
   [ WARN] util3d.cpp:664::cloudFromDepthRGB() Cloud with only NaN values created!
   ```
   This means the Kinect captured a frame with no valid depth data. Causes:
   - Pointing at a window or mirror
   - Too much sunlight/IR interference
   - USB bandwidth issues
   - Kinect overheating

3. **Camera Movement:**
   - Moving too quickly for ICP to track
   - Rotating too fast
   - Solution: Move slowly and deliberately

**Solutions:**

**Immediate fixes:**
- Point Kinect at textured surfaces with depth
- Stay 1-2 meters from objects
- Move slowly (< 0.5 m/s)
- Avoid windows and blank walls

**Parameter adjustments** (already applied in launch script):
```bash
--Icp/Iterations 30                    # More iterations for better convergence
--Icp/PointToPlaneK 20                 # More neighbors for robustness
--Icp/PointToPlaneRadius 0.5           # Larger search radius
--Icp/MaxTranslation 0.3               # Allow larger movements
--Icp/CorrespondenceRatio 0.2          # Accept 20% correspondence
--Odom/GuessMotion true                # Use motion model
--Odom/FilteringStrategy 1             # Enable Kalman filtering
```

### Issue 3: RViz Slowing Down Over Time

**Symptom:**
RViz becomes progressively slower and eventually crashes after 5-10 minutes.

**Cause:**
Visualizing the entire accumulated point cloud map.

**Solution:**
See [SLAM Performance Optimization](SLAM_Performance_Optimization.md) - this has been fixed with:
- Memory management (STM size limit)
- Local map visualization instead of full cloud
- Grid-based occupancy map

### Issue 4: Odometry Lost / Reset

**Symptom:**
```
Odom: quality=0, std dev=inf
```
Or sudden jumps in position.

**Causes:**
- Complete loss of depth features
- Kinect disconnected/reconnected
- Too many consecutive bad frames

**Solutions:**
1. **Immediate:** Stop moving, let odometry recover
2. **Prevention:** 
   - Ensure good USB 3.0 connection
   - Keep Kinect cool (it can overheat)
   - Maintain visual features in view
   - Use `--Odom/ResetCountdown 1` (already set)

### Issue 5: Map Drift

**Symptom:**
Over time, the map becomes distorted or doesn't align properly.

**Causes:**
- Accumulated odometry errors
- No loop closures to correct drift
- Poor ICP convergence

**Solutions:**
1. **Enable loop closures** (already enabled):
   ```bash
   --RGBD/ProximityBySpace true
   --RGBD/ProximityMaxGraphDepth 50
   ```

2. **Revisit previous locations** to trigger loop closures

3. **Improve odometry quality** (see Issue 2)

4. **Use graph optimization:**
   ```bash
   --RGBD/OptimizeFromGraphEnd false    # Optimize entire graph
   ```

## Monitoring Health

### Good Odometry:
```
Odom: quality=95, std dev=7.967447m|0.032761rad, update time=0.015198s
```
- Quality: 70-120
- Std dev position: < 20m
- Std dev rotation: < 0.08 rad
- Update time: < 0.02s

### Poor Odometry:
```
Odom: quality=25, std dev=93.390707m|0.084852rad, update time=0.005684s
```
- Quality: < 50
- Std dev position: > 40m
- Std dev rotation: > 0.08 rad

### RTAB-Map Processing:
```
rtabmap (202): Rate=1.00s, RTAB-Map=0.0083s, delay=0.0201s (local map=6, WM=202)
```
- Rate: Should be ~1 Hz
- RTAB-Map time: Should be < 0.1s
- Local map: Number of nearby nodes (should be < 50)
- WM: Total nodes in database (grows continuously)

## Best Practices

### Environment Setup:
1. **Good lighting** (but not direct sunlight)
2. **Textured surfaces** with depth variation
3. **Avoid reflective surfaces**
4. **Stay in 0.5-4m range**

### Movement:
1. **Start slow** - let odometry initialize
2. **Smooth motions** - no jerky movements
3. **Overlap** - keep some features in view between frames
4. **Revisit** - return to previous areas for loop closures

### Hardware:
1. **USB 3.0** connection (blue port)
2. **Good cable** - short, high quality
3. **Cooling** - ensure Kinect has airflow
4. **Power** - dedicated USB port, not a hub

## Parameter Tuning

### For Better Quality (Slower):
```bash
--Icp/Iterations 50                    # More iterations
--Icp/PointToPlaneK 30                 # More neighbors
--OdomF2M/MaxSize 3000                 # More points
```

### For Better Speed (Lower Quality):
```bash
--Icp/Iterations 10                    # Fewer iterations
--Icp/PointToPlaneK 10                 # Fewer neighbors
--OdomF2M/MaxSize 1000                 # Fewer points
--decimation:=4                        # Downsample more
```

### For Difficult Environments:
```bash
--Icp/MaxCorrespondenceDistance 0.2    # Larger matching distance
--Icp/CorrespondenceRatio 0.1          # Accept 10% matches
--Odom/FilteringStrategy 1             # Enable filtering
--Odom/GuessMotion true                # Use motion prediction
```

## Getting Help

If issues persist:
1. Check the full terminal output for errors
2. Verify Kinect is working: `./test_kinect_simple.sh`
3. Test without SLAM: `ros2 run kinect2_simple_publisher kinect2_simple_publisher_node`
4. Review logs in `~/.ros/log/`
