# CV Pipeline Synchronization Fix

## Problem

The CV pipeline uses `message_filters` to synchronize RGB and Depth images. If the timestamps don't match closely enough, the callback never triggers.

## Quick Test

Restart the pipeline and look for this message:
```
[cv_pipeline_node] [INFO] ✅ Image callback working - receiving images!
```

If you DON'T see this message, the synchronizer is failing.

## Solution: Increase Sync Tolerance

The synchronizer needs looser timing constraints. Edit the C++ node:

```cpp
// Change from:
sync_ = std::make_shared<Synchronizer>(SyncPolicy(10), rgb_sub_, depth_sub_);

// To:
sync_ = std::make_shared<Synchronizer>(SyncPolicy(100), rgb_sub_, depth_sub_);  // Larger queue
sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.1));  // 100ms tolerance
```

## Alternative: Use Separate Callbacks

If synchronization keeps failing, use separate callbacks:

```cpp
// Remove message_filters
rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    rgb_topic_, 10,
    std::bind(&CVPipelineNode::rgbCallback, this, std::placeholders::_1));

depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    depth_topic_, 10,
    std::bind(&CVPipelineNode::depthCallback, this, std::placeholders::_1));
```

Then process when both are available (they don't need perfect sync for SAM2).

## Temporary Workaround

Test SAM2 standalone while we fix the sync issue:

```bash
# Capture a frame
ros2 run image_view image_saver --ros-args \
  -r image:=/kinect2/qhd/image_color \
  -p filename_format:=/tmp/kinect_rgb.jpg

# Process with SAM2
conda activate howyouseeme
python ros2_ws/src/cv_pipeline/python/sam2_worker.py \
  --rgb /tmp/kinect_rgb.jpg \
  --depth /tmp/kinect_depth.png \
  --params "prompt_type=point" \
  --model-size tiny
```

This proves SAM2 works - it's just the ROS2 integration that needs fixing.
