# RViz Configuration Files

This directory contains RViz2 configuration files for visualizing the SLAM + CV Pipeline system.

## Active Configuration

### tsdf_rviz.rviz
The main RViz configuration for the complete system. Displays:

- **Grid** - Reference grid in the map frame
- **TF** - All coordinate frames (map, camera_link, object frames)
- **Kinect RGB** - Live camera feed from `/kinect2/hd/image_color`
- **TSDF Point Cloud** - 3D reconstruction from `/tsdf/pointcloud`
- **Semantic Labels** - 3D text markers for detected objects from `/semantic/markers`
- **ORB-SLAM3 Path** - Camera trajectory from `/orb_slam3/path`

**Fixed Frame:** `map` (ORB-SLAM3 world frame)

## Usage

Launch with the complete system:
```bash
./scripts/run_complete_slam_system.sh
```

Or manually:
```bash
rviz2 -d rviz_configs/tsdf_rviz.rviz
```

## Display Configuration

### TF Frames
- `map` - World frame (ORB-SLAM3 origin)
- `camera_link` / `camera_pose` - Camera in world frame
- `yolo_<class>_<id>` - Each YOLO detection
- `face_<name>_<id>` - Each detected face
- `sam2_seg_<id>` - Each SAM2 segmentation

### Marker Namespaces
Semantic labels are organized by source:
- `semantic/yolo_detect` - YOLO object detections (yellow)
- `semantic/yolo_segment` - YOLO segmentation (cyan)
- `semantic/yolo_pose` - YOLO pose estimation (orange)
- `semantic/sam2` - SAM2 segmentation (sky blue)
- `semantic/fastsam` - FastSAM segmentation (blue)
- `semantic/insightface` - Face detection (green)
- `semantic/emotion` - Emotion detection (magenta)

## Customization

To modify the configuration:
1. Launch RViz with the config
2. Adjust displays, colors, sizes as needed
3. Save: File → Save Config As → `rviz_configs/tsdf_rviz.rviz`

## Archive

The `archive/` subdirectory contains old RViz configurations kept for reference.
