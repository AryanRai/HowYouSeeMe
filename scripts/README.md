# Scripts Directory

This directory contains all operational scripts for the HowYouSeeMe SLAM + CV Pipeline system.

## Main Scripts

### Launch & Control
- **run_complete_slam_system.sh** - Launch the complete system (Kinect + ORB-SLAM3 + TSDF + Semantic + CV Pipeline + RViz)
- **run_phase2_3.sh** - Launch just Kinect + ORB-SLAM3 + TSDF (used by complete system)
- **kill_all_slam.sh** - Stop all SLAM-related processes
- **cv_pipeline_menu.sh** - Interactive menu for CV model selection (YOLO, SAM2, InsightFace, etc.)

### Testing & Diagnostics
- **test_complete_system.sh** - Comprehensive system health check
- **diagnose_markers.sh** - Debug marker visibility in RViz
- **debug_semantic.sh** - Debug semantic projection pipeline

### Utilities
- **restart_semantic_only.sh** - Quick restart of semantic projection node
- **QUICK_REFERENCE.txt** - Complete command reference guide

## Setup Scripts

### ORB-SLAM3 Setup
- **setup_orb_slam3.sh** - Build ORB-SLAM3 from source
- **build_phase2_3.sh** - Build ROS2 packages for Phase 2-3
- **fix_orb_slam3_cpp14.sh** - Apply C++14 compatibility patches
- **patch_orb_slam3_cmake.sh** - Patch CMake files for ROS2 compatibility

### CV Pipeline Setup
- **install_hsemotion.sh** - Install HSEmotion for emotion detection
- **install_insightface.sh** - Install InsightFace for face recognition

### Verification
- **verify_orb_slam3.sh** - Verify ORB-SLAM3 installation
- **verify_bluelily_bridge.sh** - Verify IMU bridge setup
- **check_phase2_3_deps.sh** - Check all Phase 2-3 dependencies

## Calibration Scripts (Phase 1)

- **extract_kinect_intrinsics.sh** - Extract camera intrinsics from Kinect
- **record_kalibr_bag.sh** - Record ROS2 bag for Kalibr calibration
- **kalibr_to_orb_slam3.py** - Convert Kalibr results to ORB-SLAM3 format
- **kalibr_to_tf2.py** - Convert Kalibr results to TF2 static transforms

## Quick Start

```bash
# 1. Launch the complete system
./scripts/run_complete_slam_system.sh

# 2. From the CV menu that opens, start YOLO detection:
#    Select: 3) YOLO11 → 1) Detection → Stream mode (5 FPS)

# 3. View results in RViz (opens automatically)

# 4. Stop everything
./scripts/kill_all_slam.sh
```

## Troubleshooting

```bash
# Check system health
./scripts/test_complete_system.sh

# Debug markers not showing
./scripts/diagnose_markers.sh

# Debug semantic projection
./scripts/debug_semantic.sh

# View quick reference
cat scripts/QUICK_REFERENCE.txt
```

## Archive

The `archive/` subdirectory contains old/deprecated scripts kept for reference. These are not needed for normal operation.

## Notes

- All scripts assume you're running from the workspace root directory
- ROS2 Jazzy environment is sourced automatically by launch scripts
- Logs are written to `/tmp/*.log` files
- World state is saved to `/tmp/world_state.json`
