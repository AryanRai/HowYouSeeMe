#!/usr/bin/env python3
"""
Phase 4: Semantic Projection Node
Back-projects YOLO detections into 3D world space using depth + TF2
Maintains persistent world state and publishes floating text labels in RViz
"""

import rclpy
from rclpy.node import Node
import json
import time
import numpy as np
import message_filters
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial.transform import Rotation


class SemanticProjection(Node):
    def __init__(self):
        super().__init__('semantic_projection')
        
        # Camera intrinsics (Kinect v2 HD)
        self.declare_parameter('fx', 1081.37)
        self.declare_parameter('fy', 1081.37)
        self.declare_parameter('cx', 959.5)
        self.declare_parameter('cy', 539.5)
        self.declare_parameter('world_state_path', '/tmp/world_state.json')
        self.declare_parameter('marker_lifetime', 30.0)
        self.declare_parameter('conf_threshold', 0.4)
        self.declare_parameter('depth_trunc', 5.0)
        
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value
        
        self.bridge = CvBridge()
        self.world_state = {}
        self.marker_id = 0
        self.latest_pose = None
        
        # Subscribe to ORB-SLAM3 pose
        self.pose_sub = self.create_subscription(
            PoseStamped, '/orb_slam3/pose', self.pose_callback, 10)
        
        # Subscribers with time synchronization
        self.yolo_sub = message_filters.Subscriber(
            self, String, '/cv_pipeline/results')
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/kinect2/hd/image_depth_rect')
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.yolo_sub, self.depth_sub], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self.detection_cb)
        
        # Publishers
        self.marker_pub = self.create_publisher(
            MarkerArray, '/semantic/markers', 10)
        self.state_pub = self.create_publisher(
            String, '/semantic/world_state', 10)
        
        # Timer to save world state
        self.create_timer(5.0, self.publish_world_state)
        
        self.get_logger().info('Semantic projection ready')
        self.get_logger().info(f'Camera intrinsics: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}')
        self.get_logger().info(f'Confidence threshold: {self.get_parameter("conf_threshold").value}')
    
    def pose_callback(self, msg):
        """Store latest ORB-SLAM3 pose"""
        self.latest_pose = msg
    
    def pixel_to_3d(self, u, v, depth_image):
        """Back-project pixel to 3D camera coordinates"""
        depth_trunc = self.get_parameter('depth_trunc').value
        
        # Bounds check
        h, w = depth_image.shape
        if not (0 <= int(v) < h and 0 <= int(u) < w):
            return None
        
        # Get depth in meters
        Z = float(depth_image[int(v), int(u)]) / 1000.0
        
        # Validate depth
        if Z <= 0.0 or Z > depth_trunc:
            return None
        
        # Back-project to 3D
        X = (u - self.cx) * Z / self.fx
        Y = (v - self.cy) * Z / self.fy
        
        return np.array([X, Y, Z])
    
    def camera_to_world(self, xyz_camera, stamp):
        """Transform 3D point from camera frame to world frame using ORB-SLAM3 pose"""
        if self.latest_pose is None:
            self.get_logger().warn('No ORB-SLAM3 pose available yet', throttle_duration_sec=5.0)
            return None
        
        # Get camera pose in world frame (Twc = camera-to-world transform)
        pose = self.latest_pose.pose
        
        # Extract rotation (quaternion to matrix)
        from scipy.spatial.transform import Rotation
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        R = Rotation.from_quat(q).as_matrix()
        
        # Extract translation
        t = np.array([pose.position.x, pose.position.y, pose.position.z])
        
        # Transform point: p_world = R * p_camera + t
        xyz_world = R @ xyz_camera + t
        
        return xyz_world
    
    def detection_cb(self, yolo_msg, depth_msg):
        """Process YOLO detections and back-project to 3D"""
        try:
            detections = json.loads(yolo_msg.data)
            depth = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')
        except Exception as e:
            self.get_logger().warn(f'Parse error: {e}')
            return
        
        markers = MarkerArray()
        thresh = self.get_parameter('conf_threshold').value
        
        for det in detections.get('detections', []):
            label = det.get('class_name', 'unknown')
            conf = det.get('confidence', 0.0)
            
            if conf < thresh:
                continue
            
            bbox = det.get('bbox', [])
            if len(bbox) < 4:
                continue
            
            # Get bbox centroid
            u = (bbox[0] + bbox[2]) / 2
            v = (bbox[1] + bbox[3]) / 2
            
            # Back-project to 3D camera coordinates
            xyz_cam = self.pixel_to_3d(u, v, depth)
            if xyz_cam is None:
                continue
            
            # Transform to world coordinates
            xyz_world = self.camera_to_world(xyz_cam, depth_msg.header.stamp)
            if xyz_world is None:
                continue
            
            # Create unique object ID based on label and coarse position
            obj_id = f'{label}_{int(xyz_world[0]*10)}_{int(xyz_world[1]*10)}'
            
            # Update world state
            if obj_id in self.world_state:
                self.world_state[obj_id]['count'] += 1
                self.world_state[obj_id]['last_seen'] = time.time()
                self.world_state[obj_id]['confidence'] = max(
                    self.world_state[obj_id]['confidence'], conf)
            else:
                self.world_state[obj_id] = {
                    'label': label,
                    'position': xyz_world.tolist(),
                    'confidence': conf,
                    'last_seen': time.time(),
                    'count': 1
                }
            
            # Create text marker
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'semantic'
            m.id = self.marker_id
            self.marker_id += 1
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x = float(xyz_world[0])
            m.pose.position.y = float(xyz_world[1])
            m.pose.position.z = float(xyz_world[2]) + 0.15  # Offset above object
            m.scale.z = 0.12  # Text height
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 1.0
            m.text = f'{label} ({conf:.2f})'
            
            lifetime = self.get_parameter('marker_lifetime').value
            m.lifetime.sec = int(lifetime)
            m.lifetime.nanosec = int((lifetime - int(lifetime)) * 1e9)
            
            markers.markers.append(m)
        
        if len(markers.markers) > 0:
            self.marker_pub.publish(markers)
            self.get_logger().info(
                f'Published {len(markers.markers)} semantic markers',
                throttle_duration_sec=2.0)
    
    def publish_world_state(self):
        """Save world state to JSON and publish"""
        path = self.get_parameter('world_state_path').value
        
        try:
            with open(path, 'w') as f:
                json.dump(self.world_state, f, indent=2)
            
            msg = String()
            msg.data = json.dumps(self.world_state)
            self.state_pub.publish(msg)
            
            if len(self.world_state) > 0:
                self.get_logger().info(
                    f'World state: {len(self.world_state)} objects tracked',
                    throttle_duration_sec=10.0)
        except Exception as e:
            self.get_logger().error(f'Failed to save world state: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SemanticProjection()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
