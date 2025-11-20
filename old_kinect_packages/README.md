# Old Kinect Packages - Archive

This folder contains old/reference versions of Kinect packages that have been replaced by the working `kinect2_ros2_cuda` fork.

## Contents

### iai_kinect2/
- **Source**: https://github.com/code-iai/iai_kinect2
- **Type**: Original ROS1 implementation
- **Status**: Reference only (ROS1, not compatible with ROS2)
- **Kept for**: Historical reference

### libfreenect-new/
- **Type**: Alternative libfreenect2 build attempt
- **Status**: Not used
- **Kept for**: Reference

### kinect2_ros2_original/
- **Source**: https://github.com/krepa098/kinect2_ros2 (original, unmodified)
- **Type**: ROS2 port with bugs
- **Issues**: 
  - CPU registration broken (missing DEPTH_REG_CPU definition)
  - Library linking errors
  - GLX BadAccess errors
- **Status**: Replaced by kinect2_ros2_cuda fork
- **Kept for**: Comparison and reference

### kinect2_bridge/ kinect2_calibration/ kinect2_registration/
- **Type**: Old standalone packages from initial attempts
- **Status**: Superseded by kinect2_ros2_cuda
- **Kept for**: Reference

### kinect2_simple_publisher/
- **Type**: Custom minimal ROS2 publisher
- **Features**: Basic SD resolution, manual calibration
- **Status**: Replaced by kinect2_ros2_cuda (better features)
- **Kept for**: Reference and comparison

## Current Active Package

**Location**: `~/Documents/GitHub/HowYouSeeMe/ros2_ws/src/kinect2_ros2_cuda/`

**Source**: Fork of krepa098/kinect2_ros2 with fixes

**Improvements**:
- ✅ CPU registration working
- ✅ Library linking fixed
- ✅ No GLX errors
- ✅ Comprehensive documentation
- ✅ Ready for CUDA acceleration

## Can I Delete This Folder?

**Recommendation**: Keep for 1-2 weeks while you verify the new fork works perfectly.

**After verification**, you can safely delete this entire folder:
```bash
rm -rf ~/Documents/GitHub/HowYouSeeMe/old_kinect_packages
```

## Migration History

1. **Initial**: kinect2_simple_publisher (custom minimal implementation)
2. **Attempted**: iai_kinect2 (ROS1, not compatible)
3. **Attempted**: kinect2_ros2 original (ROS2 but broken)
4. **Current**: kinect2_ros2_cuda (ROS2, fixed and working) ✅

## Disk Space

This folder uses approximately:
- iai_kinect2: ~5 MB
- libfreenect-new: ~500 MB
- kinect2_ros2_original: ~1 MB
- kinect2_simple_publisher: ~1 MB
- Other packages: ~1 MB
- **Total**: ~508 MB

## Date Archived

November 21, 2025

## Notes

All functionality from these packages has been replaced by the working `kinect2_ros2_cuda` fork. This archive is kept purely for reference and comparison purposes.
