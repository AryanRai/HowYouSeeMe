# SLAM Precision Improvements

## Problem

When building a 3D map with the Kinect v2 + RTABMap SLAM pipeline, the same
physical objects appeared multiple times in the map, each copy shifted slightly
to the side ("ghosting"). The overall layout was recognisable, but individual
surfaces were blurry and overlapping.

**Root cause:** accumulated pose-graph drift.  
RTABMap builds the map by placing each node's depth image at the node's
estimated camera pose. If the pose estimate drifts over time (because each
ICP registration introduces a small error that compounds frame-by-frame), the
same wall/object is placed at slightly different positions across nodes,
producing the characteristic "smearing" or "ghost overlay" effect.

---

## Changes Made

### 1. Tighter ICP Odometry (`odom_args`)

| Parameter | Before | After | Effect |
|---|---|---|---|
| `--Icp/VoxelSize` | 0.05 m | **0.025 m** | 2.5 cm voxels give the ICP engine more geometric detail to work with, so it converges to a more accurate pose |
| `--Icp/MaxCorrespondenceDistance` | 0.15 m | **0.08 m** | A tighter match radius rejects wrong point correspondences that pull the pose in the wrong direction |
| `--Icp/Iterations` | 30 | **50** | More iterations allow the solver to converge to a lower-error solution |
| `--Icp/Epsilon` | 1e-3 | **1e-4** | Tighter termination criterion — the solver keeps refining until the improvement really is negligible |
| `--Icp/PointToPlaneK` | 20 | **30** | More neighbours for local plane estimation → more accurate normal vectors |
| `--Icp/PointToPlaneRadius` | 0.5 m | **0.3 m** | Tighter radius for plane fitting → rejects distant noisy neighbours |
| `--Icp/CorrespondenceRatio` | 0.20 | **0.35** | Requires 35 % of points to have valid correspondences before accepting a registration — rejects degenerate frames |
| `--Icp/MaxTranslation` | 0.30 m | **0.20 m** | Guards against runaway jumps in a single ICP step |
| `--Icp/PMOutlierRatio` | 0.85 | **0.65** | A lower outlier ratio means the solver is less tolerant of noise-contaminated points |
| `--OdomF2M/ScanSubtractRadius` | 0.05 m | **0.025 m** | Finer scan subtraction keeps the reference map denser and more precise |
| `--OdomF2M/ScanMaxSize` | 15 000 | **20 000** | A larger reference map gives ICP more structure to lock onto |

### 2. Better Loop-Closure Detection (`rtabmap_args`)

| Parameter | Before | After | Effect |
|---|---|---|---|
| `--Rtabmap/LoopThr` | 0.15 | **0.11** | More sensitive loop-closure detection — RTABMap will recognise previously visited places sooner, correcting drift earlier |
| `--Rtabmap/LoopRatio` | 0.90 | **0.95** | A loop closure only fires when 95 % of visual features agree → fewer false positives that distort the map |
| `--RGBD/OptimizeMaxError` | *(off)* | **1.0 m** | Rejects any loop closure whose pose-graph optimisation residual exceeds 1 m — prevents a single bad closure from corrupting the whole map |
| `--Mem/STMSize` | 30 | **50** | Keeps 50 recent nodes in the working-memory search window, giving loop closure a wider neighbourhood to match against |
| `--RGBD/AngularUpdate` | 0.05 rad | **0.03 rad** | A new node is created after a 3° rotation instead of 5° — more keyframes = more opportunities for loop closure |
| `--RGBD/LinearUpdate` | 0.05 m | **0.03 m** | Similarly, a new node after 3 cm of translation |

### 3. Robust Pose-Graph Optimisation

| Parameter | Before | After | Effect |
|---|---|---|---|
| `--Optimizer/Robust` | false | **true** | Switches from least-squares to a robust M-estimator (Cauchy kernel). A single outlier loop closure can no longer pull all poses away from ground truth |
| `--Optimizer/Iterations` | *(default)* | **20** | More optimisation passes per cycle |
| `--RGBD/OptimizeFromGraphEnd` | false | **true** | Anchor optimisation at the most recent node — local accuracy near the current position is preserved while correcting older parts of the graph |
| `--RGBD/NeighborLinkRefining` | false | **true** | After creating a link between two consecutive nodes, RTABMap runs an additional ICP pass to refine the relative transform. This is the single most direct fix for frame-to-frame drift |

### 4. Improved Visual Feature Matching

| Parameter | Before | After | Effect |
|---|---|---|---|
| `--Vis/MaxFeatures` | *(default ~500)* | **1 000** | More GFTT/ORB features detected per frame → more inliers during loop-closure verification |
| `--Vis/MinInliers` | *(default)* | **25** | A loop closure requires at least 25 matching features before it is accepted |
| `--Vis/InlierDistance` | *(default)* | **0.05 m** | Tight reprojection error threshold for RANSAC inlier classification |

### 5. Finer Occupancy Grid

| Parameter | Before | After | Effect |
|---|---|---|---|
| `--Grid/CellSize` | 0.05 m | **0.025 m** | 2.5 cm cells in the 2-D occupancy map — walls and obstacles are represented at twice the linear resolution |
| `--Grid/ClusterRadius` | 0.10 m | **0.05 m** | Tighter cluster radius when building the grid from the 3-D cloud |

### 6. Point Cloud Denoiser (`pointcloud_denoiser.py`)

A new standalone ROS 2 Python node that runs alongside RTABMap and cleans
up the raw Kinect point cloud for both better visualisation and (if wired to
the SLAM) cleaner input data.

**Pipeline (in order):**

1. **Range filter** — discard points closer than 0.3 m (sensor noise, self-
   reflections) and farther than 4 m (weak depth signal).

2. **Voxel-grid downsampling** — keep one representative point per 2 cm³
   voxel.  Reduces cloud density uniformly so the visualisation stays
   responsive and downstream algorithms see fewer redundant points.

3. **Statistical Outlier Removal (SOR)** — for each point compute the mean
   distance to its 20 nearest neighbours; remove points whose mean distance
   exceeds `μ + 1.5 σ`.  Eliminates isolated sensor noise speckles that
   create "floating" ghost surfaces around objects.

Published on `/kinect2/qhd/points_filtered` — enabled in `full_system_rviz.rviz`
as the primary point-cloud display (raw cloud kept but disabled by default).

---

## Expected Improvements

| Metric | Before | After (estimated) |
|---|---|---|
| Frame-to-frame ICP residual | ~0.3–0.5 cm | ~0.1–0.2 cm |
| Map drift after 5 m trajectory | visible ghosting | sharply single surfaces |
| Loop-closure detection rate | moderate | noticeably higher |
| 2-D occupancy map resolution | 5 cm/cell | 2.5 cm/cell |
| Point-cloud visualisation | noisy, dense | clean, denoised |

---

## Tuning Tips

### If odometry is lost frequently (fast motion)
- Increase `--Icp/MaxCorrespondenceDistance` back to 0.12 m
- Decrease `--Icp/CorrespondenceRatio` to 0.25
- Increase `--Odom/ParticleSize` to 600

### If ghosting still visible after loop closure
- Lower `--Rtabmap/LoopThr` further to 0.09
- Increase `--Mem/STMSize` to 70
- Verify that `--RGBD/NeighborLinkRefining true` is active (check RTAB-Map log)

### If processing is too slow (< 10 Hz)
- Increase `--Icp/VoxelSize` to 0.03 m
- Decrease `--OdomF2M/ScanMaxSize` to 15 000
- Reduce `--Vis/MaxFeatures` to 600

### Saving the map for reuse
Remove `--delete_db_on_start` from the launch script.  The database is stored
at `~/.ros/rtabmap.db` and will be loaded automatically on the next run.

---

## Files Changed

| File | Change |
|---|---|
| `launch_full_system_rviz.sh` | Shared `RTABMAP_PRECISION_ARGS` + `ODOM_PRECISION_ARGS` variables; denoiser step added |
| `launch_kinect2_ros2_slam_fixed_tf.sh` | Same precision parameters applied |
| `launch_kinect2_slam_with_imu.sh` | Same precision parameters applied |
| `full_system_rviz.rviz` | Added filtered cloud display; tighter SLAM map voxel/decimation |
| `pointcloud_denoiser.py` | New: real-time range + voxel + SOR filter node |
| `docs/SLAM_PRECISION_IMPROVEMENTS.md` | This file |
