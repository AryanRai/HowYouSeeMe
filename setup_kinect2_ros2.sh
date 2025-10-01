#!/bin/bash
# HowYouSeeMe Kinect2 ROS2 Setup Script
# Installs kinect2_ros2 package and creates HowYouSeeMe integration

set -e  # Exit on any error

echo "🚀 HowYouSeeMe Kinect2 ROS2 Setup"
echo "=================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

# Check if ROS2 is installed
print_step "Checking ROS2 installation..."
if ! command -v ros2 &> /dev/null; then
    print_error "ROS2 not found. Please install ROS2 Jazzy first."
    print_status "Install with: https://docs.ros.org/en/jazzy/Installation.html"
    exit 1
fi

# Source ROS2
print_status "Sourcing ROS2 Jazzy..."
source /opt/ros/jazzy/setup.bash

# Check ROS2 version
ROS_VERSION=$(ros2 --version 2>/dev/null | head -n1 || echo "unknown")
print_status "Found ROS2: $ROS_VERSION"

# Install required system packages
print_step "Installing system dependencies..."
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
    ros-jazzy-rqt-image-view \
    libopencv-dev \
    python3-opencv \
    python3-colcon-common-extensions \
    git

print_status "✅ System dependencies installed"

# Install libfreenect2 if not already installed
print_step "Checking libfreenect2 installation..."
if [ ! -f "/usr/local/lib/libfreenect2.so" ] && [ ! -f "$HOME/libfreenect2-modern/lib/libfreenect2.so" ]; then
    print_warning "libfreenect2 not found. Installing..."
    
    # Ask user which version to install
    echo "Choose libfreenect2 installation:"
    echo "1) High-performance CUDA version (recommended)"
    echo "2) Basic CPU-only version"
    read -p "Enter choice (1 or 2): " choice
    
    case $choice in
        1)
            print_status "Installing high-performance libfreenect2 with CUDA..."
            if [ -f "./install_libfreenect2_modern_fixed.sh" ]; then
                chmod +x ./install_libfreenect2_modern_fixed.sh
                ./install_libfreenect2_modern_fixed.sh
            else
                print_error "install_libfreenect2_modern_fixed.sh not found in current directory"
                exit 1
            fi
            ;;
        2)
            print_status "Installing basic libfreenect2..."
            cd /tmp
            git clone https://github.com/OpenKinect/libfreenect2.git
            cd libfreenect2
            mkdir build && cd build
            cmake .. -DENABLE_CXX11=ON -DBUILD_OPENNI2_DRIVER=OFF \
                     -DENABLE_OPENCL=OFF -DENABLE_CUDA=OFF -DENABLE_OPENGL=OFF \
                     -DENABLE_VAAPI=OFF -DENABLE_TEGRAJPEG=OFF -DCMAKE_INSTALL_PREFIX=/usr/local
            make -j$(nproc)
            sudo make install
            cd ~
            ;;
        *)
            print_error "Invalid choice. Exiting."
            exit 1
            ;;
    esac
else
    print_status "✅ libfreenect2 already installed"
fi

# Create ROS2 workspace
print_step "Setting up ROS2 workspace..."
ROS2_WS="$HOME/ros2_ws"
mkdir -p $ROS2_WS/src
cd $ROS2_WS/src

# Clone kinect2_ros2 package
print_step "Cloning kinect2_ros2 package..."
if [ ! -d "kinect2_ros2" ]; then
    git clone https://github.com/krepa098/kinect2_ros2.git
    print_status "✅ kinect2_ros2 cloned"
else
    print_status "✅ kinect2_ros2 already exists"
fi

# Create HowYouSeeMe ROS2 package
print_step "Creating HowYouSeeMe ROS2 integration package..."
if [ ! -d "howyouseeme_ros2" ]; then
    ros2 pkg create --build-type ament_python howyouseeme_ros2 \
        --dependencies rclpy sensor_msgs cv_bridge geometry_msgs tf2_ros
    print_status "✅ howyouseeme_ros2 package created"
else
    print_status "✅ howyouseeme_ros2 package already exists"
fi

# Create the subscriber node
print_step "Creating HowYouSeeMe subscriber node..."
mkdir -p howyouseeme_ros2/howyouseeme_ros2

cat > howyouseeme_ros2/howyouseeme_ros2/kinect_subscriber.py << 'EOF'
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

# Add our source path - adjust this path as needed
current_dir = os.path.dirname(os.path.abspath(__file__))
howyouseeme_src = os.path.join(current_dir, '../../../../src')
if os.path.exists(howyouseeme_src):
    sys.path.append(howyouseeme_src)

try:
    from perception.slam.slam_interface import BasicSLAM
    from perception.detection.object_detector import YOLODetector, ResourceManager
    COMPONENTS_AVAILABLE = True
except ImportError as e:
    print(f"Warning: HowYouSeeMe components not available: {e}")
    COMPONENTS_AVAILABLE = False

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
        
        if COMPONENTS_AVAILABLE:
            self.resource_manager = ResourceManager(max_gpu_memory_gb=4.0)
        
        # Processing intervals (optimized from our tests)
        self.frame_count = 0
        self.slam_interval = 2      # Process SLAM every 2nd frame
        self.detection_interval = 10 # Process detection every 10th frame
        
        # Performance tracking
        self.start_time = self.get_clock().now()
        
        status = "with full pipeline" if COMPONENTS_AVAILABLE else "in basic mode (no SLAM/detection)"
        self.get_logger().info(f'HowYouSeeMe ROS2 Subscriber initialized {status}')
    
    def camera_info_callback(self, msg):
        """Store camera info for SLAM initialization"""
        if self.camera_info is None:
            self.camera_info = msg
            if COMPONENTS_AVAILABLE:
                self.initialize_components()
    
    def initialize_components(self):
        """Initialize SLAM and object detection"""
        if self.camera_info is None or not COMPONENTS_AVAILABLE:
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
        
        self.frame_count += 1
        
        # Basic processing (always available)
        rgb_shape = self.latest_rgb.shape
        depth_shape = self.latest_depth.shape
        
        # Advanced processing (if components available)
        slam_result = None
        detections = []
        
        if COMPONENTS_AVAILABLE and self.slam is not None and self.detector is not None:
            # SLAM processing (every slam_interval frames)
            if self.frame_count % self.slam_interval == 0:
                slam_result = self.slam.process_frame(self.latest_rgb, self.latest_depth)
                
                # Publish pose if tracking
                if slam_result and slam_result.get('pose') is not None:
                    self.publish_pose(slam_result['pose'])
            
            # Object detection (every detection_interval frames)
            if self.frame_count % self.detection_interval == 0:
                detections = self.detector.detect_objects(self.latest_rgb)
        
        # Log progress
        if self.frame_count % 150 == 0:  # Every ~10 seconds at 15 FPS
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            fps = self.frame_count / elapsed
            
            if COMPONENTS_AVAILABLE and slam_result:
                slam_status = 'OK' if slam_result.get('is_tracking') else 'LOST'
                self.get_logger().info(
                    f'Frame {self.frame_count}: {fps:.1f} FPS, '
                    f'SLAM: {slam_status}, Objects: {len(detections)}'
                )
            else:
                self.get_logger().info(
                    f'Frame {self.frame_count}: {fps:.1f} FPS, '
                    f'RGB: {rgb_shape}, Depth: {depth_shape}'
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
EOF

# Make the node executable
chmod +x howyouseeme_ros2/howyouseeme_ros2/kinect_subscriber.py

# Create __init__.py
touch howyouseeme_ros2/howyouseeme_ros2/__init__.py

# Update setup.py
cat > howyouseeme_ros2/setup.py << 'EOF'
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
    maintainer='HowYouSeeMe',
    maintainer_email='contact@howyouseeme.com',
    description='HowYouSeeMe ROS2 Kinect integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinect_subscriber = howyouseeme_ros2.kinect_subscriber:main',
        ],
    },
)
EOF

print_status "✅ HowYouSeeMe subscriber node created"

# Build the packages
print_step "Building ROS2 packages..."
cd $ROS2_WS

# First try to build kinect2 packages
print_status "Building kinect2_bridge..."
if colcon build --packages-select kinect2_bridge kinect2_registration --continue-on-error; then
    print_status "✅ kinect2 packages built successfully"
else
    print_warning "⚠️  kinect2 packages build had issues, but continuing..."
fi

# Build our package
print_status "Building howyouseeme_ros2..."
if colcon build --packages-select howyouseeme_ros2; then
    print_status "✅ howyouseeme_ros2 built successfully"
else
    print_error "❌ Failed to build howyouseeme_ros2"
    exit 1
fi

# Source the workspace
source install/setup.bash

# Add to bashrc
print_step "Updating bashrc..."
if ! grep -q "source $ROS2_WS/install/setup.bash" ~/.bashrc; then
    echo "source $ROS2_WS/install/setup.bash" >> ~/.bashrc
    print_status "✅ Added ROS2 workspace to bashrc"
fi

# Create launch scripts
print_step "Creating launch scripts..."
mkdir -p ~/howyouseeme_scripts

# Kinect bridge launcher
cat > ~/howyouseeme_scripts/start_kinect_bridge.sh << 'EOF'
#!/bin/bash
echo "🚀 Starting Kinect2 Bridge..."
source ~/ros2_ws/install/setup.bash
ros2 launch kinect2_bridge kinect2_bridge_launch.yaml
EOF

# HowYouSeeMe launcher
cat > ~/howyouseeme_scripts/start_howyouseeme.sh << 'EOF'
#!/bin/bash
echo "🧠 Starting HowYouSeeMe ROS2 Subscriber..."
source ~/ros2_ws/install/setup.bash
ros2 run howyouseeme_ros2 kinect_subscriber
EOF

# Combined launcher
cat > ~/howyouseeme_scripts/start_complete_system.sh << 'EOF'
#!/bin/bash
echo "🚀 Starting Complete HowYouSeeMe System..."
source ~/ros2_ws/install/setup.bash

# Start kinect bridge in background
echo "Starting Kinect2 Bridge..."
ros2 launch kinect2_bridge kinect2_bridge_launch.yaml &
KINECT_PID=$!

# Wait a bit for kinect to initialize
sleep 5

# Start HowYouSeeMe subscriber
echo "Starting HowYouSeeMe Subscriber..."
ros2 run howyouseeme_ros2 kinect_subscriber &
HOWYOUSEEME_PID=$!

# Wait for user interrupt
echo "System running. Press Ctrl+C to stop..."
trap "echo 'Stopping...'; kill $KINECT_PID $HOWYOUSEEME_PID; exit" INT
wait
EOF

chmod +x ~/howyouseeme_scripts/*.sh
print_status "✅ Launch scripts created in ~/howyouseeme_scripts/"

# Create RViz config
print_step "Creating RViz configuration..."
mkdir -p ~/howyouseeme_scripts/config

cat > ~/howyouseeme_scripts/config/howyouseeme.rviz << 'EOF'
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /Image1
      Splitter Ratio: 0.5
    Tree Height: 557
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /2D Pose Estimate1
      - /2D Nav Goal1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
Preferences:
  PromptSaveOnExit: true
Toolbars:
  toolButtonStyle: 2
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz_default_plugins/Image
      Enabled: true
      Image Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /kinect2/hd/image_color
      Max Value: 1
      Median window: 5
      Min Value: 0
      Name: Image
      Normalize Range: true
      Queue Size: 2
      Transport Hint: raw
      Unreliable: false
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Default Light: true
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
    - Class: rviz_default_plugins/SetInitialPose
      Covariance x: 0.25
      Covariance y: 0.25
      Covariance yaw: 0.06853891909122467
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /goal_pose
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /clicked_point
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 10
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.785398006439209
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.785398006439209
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 846
  Hide Left Dock: false
  Hide Right Dock: false
  Image:
    collapsed: false
  QMainWindow State: 000000ff00000000fd000000040000000000000156000002b0fc0200000009fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d000001d1000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500200053007400650072006500006f0200000000000000640000030000000000fb0000000c004b0069006e0065006300740200000000000000640000030000000000fb0000000a0049006d006100670065010000020e000000d90000002800ffffff000000010000010f000002b0fc0200000003fb0000001e0054006f006f006c002000500072006f007000650072007400690065007301000000410000006f0000005c00fffffffb0000000a00560069006500770073010000010000000238000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004420000003efc0100000002fb0000000800540069006d00650100000000000004420000000000000000fb0000000800540069006d006501000000000000045000000000000000000000023f000002b000000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Selection:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: false
  Width: 1088
  X: 72
  Y: 60
EOF

print_status "✅ RViz configuration created"

# Final instructions
print_step "Setup complete! 🎉"
echo ""
echo "📋 Next Steps:"
echo "=============="
echo ""
echo "1. Test Kinect connection:"
echo "   ~/howyouseeme_scripts/start_kinect_bridge.sh"
echo ""
echo "2. In another terminal, test HowYouSeeMe integration:"
echo "   ~/howyouseeme_scripts/start_howyouseeme.sh"
echo ""
echo "3. Or start everything at once:"
echo "   ~/howyouseeme_scripts/start_complete_system.sh"
echo ""
echo "4. Visualize with RViz2:"
echo "   rviz2 -d ~/howyouseeme_scripts/config/howyouseeme.rviz"
echo ""
echo "5. Monitor topics:"
echo "   ros2 topic list | grep kinect2"
echo "   ros2 topic hz /kinect2/hd/image_color"
echo ""
echo "📁 Files created:"
echo "   - ROS2 workspace: ~/ros2_ws"
echo "   - Launch scripts: ~/howyouseeme_scripts/"
echo "   - RViz config: ~/howyouseeme_scripts/config/howyouseeme.rviz"
echo ""
print_status "✅ HowYouSeeMe Kinect2 ROS2 setup completed successfully!"
echo ""
echo "🔄 Please run: source ~/.bashrc"
echo "   Or open a new terminal to use the new environment"