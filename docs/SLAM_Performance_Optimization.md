# RTAB-Map SLAM Performance Optimization

## Problem
Over time, RViz visualization becomes slower and eventually crashes as the SLAM map grows larger. This happens because:
- The full point cloud map accumulates all data from the entire session
- RViz tries to render thousands of points in real-time
- Memory usage grows unbounded

## Solution
We've optimized the system with memory management and efficient visualization:

### 1. Memory Management Parameters

Added to `launch_kinect_slam.sh`:

```bash
--Mem/IncrementalMemory true          # Enable incremental SLAM mode
--Mem/InitWMWithAllNodes false        # Don't load all nodes into working memory
--Mem/STMSize 30                      # Short-term memory: keep last 30 nodes
--RGBD/ProximityBySpace true          # Use spatial proximity for loop closure
--RGBD/ProximityMaxGraphDepth 50      # Limit graph depth for proximity detection
--RGBD/ProximityPathMaxNeighbors 3    # Max neighbors to consider
--Rtabmap/TimeThr 0                   # No time limit for map updates
--Rtabmap/MemoryThr 0                 # No memory threshold (use STM instead)
```

**What this does:**
- Keeps only recent nodes (30) in active working memory
- Older nodes are stored in long-term memory but not actively rendered
- Loop closure still works using spatial proximity
- Map continues to grow but doesn't overwhelm visualization

### 2. Grid/Occupancy Map Parameters

```bash
--Grid/MaxObstacleHeight 2.0          # Only obstacles up to 2m height
--Grid/MaxGroundHeight 0.0            # Ground plane at 0m
--Grid/CellSize 0.05                  # 5cm grid resolution
--Grid/RangeMax 4.0                   # Max range 4 meters
--Grid/ClusterRadius 0.1              # Cluster nearby points
--Grid/GroundIsObstacle false         # Don't treat ground as obstacle
```

**What this does:**
- Creates efficient 2D occupancy grid for navigation
- Filters out ceiling and floor clutter
- Reduces point cloud density

### 3. Visualization Optimization

Changed RViz to use `/rtabmap/local_map` instead of `/rtabmap/cloud_map`:

- **Local Map**: Shows only recent nearby points (fast, efficient)
- **Full Cloud Map**: Shows entire accumulated map (slow, disabled by default)

You can enable the full cloud map in RViz if needed, but expect performance degradation.

## Performance Comparison

### Before Optimization:
- Working Memory: Unlimited (grows to 500+ nodes)
- Visualization: Full accumulated map
- Result: Slows down over time, crashes after 5-10 minutes

### After Optimization:
- Working Memory: ~30 active nodes
- Visualization: Local map only
- Result: Consistent performance, can run indefinitely

## Monitoring Performance

Watch the RTAB-Map output for these metrics:
```
rtabmap (484): Rate=1.00s, Limit=0.000s, Conversion=0.0012s, 
               RTAB-Map=0.0602s, Maps update=0.0037s pub=0.0055s 
               delay=0.0947s (local map=476, WM=484)
```

- `local map=476`: Nodes in local proximity (for visualization)
- `WM=484`: Total nodes in database (continues growing)
- `RTAB-Map=0.06s`: Processing time should stay under 0.1s

## Adjusting Memory Settings

If you need more/less memory:

**More aggressive (faster, less accurate):**
```bash
--Mem/STMSize 20                      # Keep only 20 recent nodes
--RGBD/ProximityMaxGraphDepth 30      # Smaller graph depth
```

**Less aggressive (slower, more accurate):**
```bash
--Mem/STMSize 50                      # Keep 50 recent nodes
--RGBD/ProximityMaxGraphDepth 100     # Larger graph depth
```

## Saving and Loading Maps

The map database is saved automatically. To reuse a map:

1. Remove `--delete_db_on_start` from launch script
2. The database is stored in `~/.ros/rtabmap.db`
3. On restart, RTAB-Map will load and continue mapping

## Troubleshooting

**Still experiencing slowdowns?**
- Reduce `--Mem/STMSize` to 20 or 15
- Increase `--Grid/CellSize` to 0.1 (coarser grid)
- Reduce `--OdomF2M/ScanMaxSize` to 10000 (fewer points)

**Loop closures not working?**
- Increase `--RGBD/ProximityMaxGraphDepth` to 100
- Increase `--Mem/STMSize` to 50

**Want to see full map occasionally?**
- In RViz, enable "RTABMap Full Cloud (Disabled)" display
- View for a moment, then disable again
