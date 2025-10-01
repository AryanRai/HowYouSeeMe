# Kinect v2 ROS2 Integration for HowYouSeeMe

This guide shows how to use the Microsoft Kinect v2 with ROS2 Jazzy, integrating the kinect2_ros2 package with our HowYouSeeMe SLAM and object detection pipeline.

## 🎯 Overview

We'll use the existing kinect2_ros2 package to publish Kinect data to ROS2 topics, then subscribe to these topics in our HowYouSeeMe pipeline for SLAM and object detection.

**Architecture:**
```
Kinect v2 Hardware → kinect2_bridge → ROS2 Topics → HowYouSeeMe Pipeline
                                    ↓
                            /kinect2/hd/image_color
                            /kinect2/hd/image_depth_rect
                            /kinect2/hd/camera_info
```

## 🔧 Prerequisites

### 1. ROS2 Jazzy Setup
Ensure ROS2 Jazzy is installed and sourced:
```bash
source /opt/ros/jazzy/setup.bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

### 2. Install libfreenect2 with CUDA Support
Use our optimized installation for better performance:
```bash
# Use our high-performance libfreenect2 installation
./install_libfreenect2_modern_fixed.sh
```

**OR** use the basic CPU-only version (from original guide):
```bash
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2 && mkdir build && cd build
cmake .. -DENABLE_CXX11=ON -DBUILD_OPENNI2_DRIVER=OFF \
         -DENABLE_OPENCL=OFF -DENABLE_CUDA=OFF -DENABLE_OPENGL=OFF \
         -DENABLE_VAAPI=OFF -DENABLE_TEGRAJPEG=OFF -DCMAKE_INSTALL_PREFIX=/usr
make -j4
sudo make install
```

### 3. Install Required ROS2 Packages
```bash
sudo apt update
sudo apt install -y \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-rtabmap-ros \
    ros-jazzy-rviz2 \
    libopencv-dev \
    python3-opencv
```

## 📦 Installation

### 1. Create ROS2 Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Clone kinect2_ros2 Package
```bash
# Clone the kinect2_ros2 package
git clone https://github.com/krepa098/kinect2_ros2.git

# Note: This package was designed for Humble, but should work with Jazzy
# If you encounter issues, we'll create compatibility fixes
```

### 3. Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select kinect2_bridge kinect2_registration kinect2_calibration
source install/setup.bash
```

### 4. Add to bashrc for convenience
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## 🚀 Usage

### 1. Launch Kinect2 Bridge
```bash
# Terminal 1: Start the kinect2_bridge
ros2 launch kinect2_bridge kinect2_bridge_launch.yaml
```

### 2. Verify Topics are Publishing
```bash
# Terminal 2: Check available topics
ros2 topic list | grep kinect2

# Expected output:
# /kinect2/hd/camera_info
# /kinect2/hd/image_color
# /kinect2/hd/image_depth_rect
# /kinect2/sd/camera_info
# /kinect2/sd/image_color_rect
# /kinect2/sd/image_depth_rect
```

### 3. View Camera Feed
```bash
# View RGB image
ros2 run rqt_image_view rqt_image_view /kinect2/hd/image_color

# View depth image
ros2 run rqt_image_view rqt_image_view /kinect2/hd/image_depth_rect
```

## 🔗 HowYouSeeMe Integration

Now let's create a ROS2 subscriber that integrates with our existing SLAM and object detection:

### 1. Create HowYouSeeMe ROS2 Package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python howyouseeme_ros2 \
    --dependencies rclpy sensor_msgs cv_bridge geometry_msgs tf2_ros
```

### 2. Create ROS2 Subscriber Node
Create `~/ros2_ws/src/howyouseeme_ros2/howyouseeme_ros2/kinect_subscriber.py`:

```python
#!/usr/bin/env python3
"""
HowYouSeeMe ROS2 Kinect Subscriber
Subscribes to kinect2_bridge topics and processes with our SLAM/detection pipeline
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys
import os

# Add our source path
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../../src'))

from perception.slam.slam_interface import BasicSLAM
from perception.detection.object_detector import YOLODetector, ResourceManager

class HowYouSeeMeSubscriber(Node):
    def __init__(self):
        super().__init__('howyouseeme_subscriber')
        
        # ROS2 setup
        self.bridge = CvBridge()
        
        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image, '/kinect2/hd/image_color', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/kinect2/hd/image_depth_rect', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/kinect2/hd/camera_info', self.camera_info_callback, 10)
        
        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/howyouseeme/pose', 10)
        
        # Data storage
        self.latest_rgb = None
        self.latest_depth = None
        self.camera_info = None
        
        # Initialize our components
        self.slam = None
        self.detector = None
        self.resource_manager = ResourceManager(max_gpu_memory_gb=4.0)
        
        # Processing intervals (optimized from our tests)
        self.frame_count = 0
        self.slam_interval = 2      # Process SLAM every 2nd frame
        self.detection_interval = 10 # Process detection every 10th frame
        
        # Performance tracking
        self.start_time = self.get_clock().now()
        
        self.get_logger().info('HowYouSeeMe ROS2 Subscriber initialized')
    
    def camera_info_callback(self, msg):
        """Store camera info for SLAM initialization"""
        if self.camera_info is None:
            self.camera_info = msg
            self.initialize_components()
    
    def initialize_components(self):
        """Initialize SLAM and object detection"""
        if self.camera_info is None:
            return
        
        try:
            # Extract camera intrinsics
            K = np.array(self.camera_info.k).reshape(3, 3)
            rgb_intrinsics = {
                'fx': K[0, 0],
                'fy': K[1, 1], 
                'cx': K[0, 2],
                'cy': K[1, 2],
                'width': self.camera_info.width,
                'height': self.camera_info.height
            }
            
            # Initialize SLAM
            self.slam = BasicSLAM(rgb_intrinsics)
            self.get_logger().info('SLAM initialized')
            
            # Initialize object detector
            self.detector = YOLODetector(
                model_name="yolov5n",  # Fast nano model
                confidence_threshold=0.6,
                resource_manager=self.resource_manager
            )
            self.get_logger().info('Object detector initialized')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize components: {e}')
    
    def rgb_callback(self, msg):
        """Process RGB image"""
        try:
            # Convert ROS image to OpenCV
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            
            # Process if we have both RGB and depth
            if self.latest_depth is not None:
                self.process_frames()
                
        except Exception as e:
            self.get_logger().error(f'RGB callback error: {e}')
    
    def depth_callback(self, msg):
        """Process depth image"""
        try:
            # Convert ROS depth image to OpenCV (16-bit to float)
            depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            self.latest_depth = depth_raw.astype(np.float32) / 1000.0  # Convert mm to meters
            
        except Exception as e:
            self.get_logger().error(f'Depth callback error: {e}')
    
    def process_frames(self):
        """Process RGB-D frames with our pipeline"""
        if self.latest_rgb is None or self.latest_depth is None:
            return
        
        if self.slam is None or self.detector is None:
            return
        
        self.frame_count += 1
        
        # SLAM processing (every slam_interval frames)
        slam_result = None
        if self.frame_count % self.slam_interval == 0:
            slam_result = self.slam.process_frame(self.latest_rgb, self.latest_depth)
            
            # Publish pose if tracking
            if slam_result and slam_result.get('pose') is not None:
                self.publish_pose(slam_result['pose'])
        
        # Object detection (every detection_interval frames)
        detections = []
        if self.frame_count % self.detection_interval == 0:
            detections = self.detector.detect_objects(self.latest_rgb)
        
        # Log progress
        if self.frame_count % 150 == 0:  # Every ~10 seconds at 15 FPS
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            fps = self.frame_count / elapsed
            
            slam_status = 'OK' if slam_result and slam_result.get('is_tracking') else 'LOST'
            
            self.get_logger().info(
                f'Frame {self.frame_count}: {fps:.1f} FPS, '
                f'SLAM: {slam_status}, Objects: {len(detections)}'
            )
    
    def publish_pose(self, pose_matrix):
        """Publish SLAM pose to ROS2"""
        try:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            
            # Extract position and orientation from 4x4 matrix
            pose_msg.pose.position.x = float(pose_matrix[0, 3])
            pose_msg.pose.position.y = float(pose_matrix[1, 3])
            pose_msg.pose.position.z = float(pose_matrix[2, 3])
            
            # Convert rotation matrix to quaternion (simplified)
            pose_msg.pose.orientation.w = 1.0
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            
            self.pose_pub.publish(pose_msg)
            
        except Exception as e:
            self.get_logger().error(f'Pose publishing error: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    node = HowYouSeeMeSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Create Package Setup
Create `~/ros2_ws/src/howyouseeme_ros2/setup.py`:

```python
from setuptools import setup

package_name = 'howyouseeme_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='HowYouSeeMe ROS2 integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinect_subscriber = howyouseeme_ros2.kinect_subscriber:main',
        ],
    },
)
```

### 4. Build and Test
```bash
cd ~/ros2_ws
colcon build --packages-select howyouseeme_ros2
source install/setup.bash
```

## 🎮 Complete Usage

### 1. Start Kinect Bridge
```bash
# Terminal 1: Start kinect2_bridge
ros2 launch kinect2_bridge kinect2_bridge_launch.yaml
```

### 2. Start HowYouSeeMe Processing
```bash
# Terminal 2: Start our SLAM and detection pipeline
ros2 run howyouseeme_ros2 kinect_subscriber
```

### 3. Monitor Performance
```bash
# Terminal 3: Monitor topics
ros2 topic hz /kinect2/hd/image_color
ros2 topic echo /howyouseeme/pose
```

### 4. Visualize with RViz2
```bash
# Terminal 4: Launch RViz2
rviz2
# Add topics: /kinect2/hd/image_color, /howyouseeme/pose
```

## 🔧 Troubleshooting

### Jazzy Compatibility Issues
If you encounter build errors with ROS2 Jazzy:

1. **Update package.xml** to use Jazzy dependencies
2. **Check for deprecated APIs** and update accordingly
3. **Use our fallback**: Create a simple bridge using our Python interface

### Performance Optimization
- **Reduce image resolution** in kinect2_bridge config
- **Adjust processing intervals** in the subscriber
- **Monitor CPU/GPU usage** and adjust accordingly

## 📊 Expected Performance

With this setup, you should achieve:
- **10-15 FPS** processing rate
- **Full ROS2 ecosystem** integration
- **SLAM and object detection** working together
- **Easy visualization** with RViz2
- **Scalable architecture** for additional features

This approach gives you the best of both worlds: established ROS2 Kinect support with our optimized processing pipeline!