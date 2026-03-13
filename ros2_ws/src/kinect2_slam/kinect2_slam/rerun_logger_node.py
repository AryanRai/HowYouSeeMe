#!/usr/bin/env python3
"""
Rerun Visualisation Node for HowYouSeeMe
Mirrors all ROS topics into Rerun for timeline scrubbing and session replay.

Can be run with conda python for numpy 2.x compatibility:
    ~/anaconda3/envs/howyouseeme/bin/python rerun_logger_node.py
"""

import sys
import os

# Add ROS2 to path when running from conda
if 'PYTHONPATH' in os.environ:
    for path in os.environ['PYTHONPATH'].split(':'):
        if path not in sys.path:
            sys.path.append(path)

import rclpy
import rerun as rr
import numpy as np
import json
from datetime import datetime
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String, Bool

# Import cv_bridge only if numpy < 2 (system python)
try:
    from cv_bridge import CvBridge
    HAS_CV_BRIDGE = True
except ImportError:
    HAS_CV_BRIDGE = False
    print("Warning: cv_bridge not available, using manual image conversion")

BEST_EFFORT = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)


class RerunLogger(Node):
    def __init__(self):
        super().__init__('rerun_logger')
        
        # Parameters
        self.declare_parameter('recording_name', 'howyouseeme')
        self.declare_parameter('spawn_viewer', True)
        self.declare_parameter('save_path', f'/tmp/howyouseeme_{datetime.now().strftime("%Y%m%d_%H%M%S")}.rrd')
        self.declare_parameter('rgb_downsample', 2)
        self.declare_parameter('pc_max_points', 50000)
        
        name = self.get_parameter('recording_name').value
        spawn = self.get_parameter('spawn_viewer').value
        save = self.get_parameter('save_path').value
        
        rr.init(name, spawn=spawn)
        if save:
            rr.save(save)
        
        self.bridge = CvBridge() if HAS_CV_BRIDGE else None
        self.ds = self.get_parameter('rgb_downsample').value
        self.max_pts = self.get_parameter('pc_max_points').value
        
        # Camera RGB
        self.create_subscription(
            Image, '/kinect2/hd/image_color', self.rgb_cb, BEST_EFFORT)
        
        # Depth
        self.create_subscription(
            Image, '/kinect2/hd/image_depth_rect', self.depth_cb, BEST_EFFORT)
        
        # ORB-SLAM3 pose
        self.create_subscription(
            PoseStamped, '/orb_slam3/pose', self.pose_cb, BEST_EFFORT)
        
        # TSDF point cloud
        self.create_subscription(
            PointCloud2, '/tsdf/pointcloud', self.tsdf_cb, BEST_EFFORT)
        
        # CV detections
        self.create_subscription(
            String, '/cv_pipeline/results', self.yolo_cb, BEST_EFFORT)
        
        # World state
        self.create_subscription(
            String, '/semantic/world_state', self.world_state_cb, BEST_EFFORT)
        
        # Semantic markers
        self.create_subscription(
            MarkerArray, '/semantic/markers', self.markers_cb, BEST_EFFORT)
        
        self.get_logger().info(f'Rerun logger ready: {name}')
        if save:
            self.get_logger().info(f'Recording to: {save}')
    
    def _stamp_to_ns(self, header):
        return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec
    
    def _set_time(self, header):
        """Set timeline time for Rerun 0.30.x API"""
        timestamp_ns = self._stamp_to_ns(header)
        rr.set_time("ros_time", timestamp_ns)
    
    def _convert_image(self, msg, encoding='rgb8'):
        """Manual image conversion when cv_bridge unavailable"""
        if self.bridge:
            return self.bridge.imgmsg_to_cv2(msg, encoding)
        
        # Manual conversion for common encodings
        if encoding == 'rgb8':
            img = np.frombuffer(msg.data, dtype=np.uint8)
            img = img.reshape((msg.height, msg.width, 3))
            return img
        elif encoding == '16UC1':
            img = np.frombuffer(msg.data, dtype=np.uint16)
            img = img.reshape((msg.height, msg.width))
            return img
        else:
            raise ValueError(f"Unsupported encoding: {encoding}")
    
    def rgb_cb(self, msg):
        self._set_time(msg.header)
        img = self._convert_image(msg, 'rgb8')
        if self.ds > 1:
            img = img[::self.ds, ::self.ds]
        rr.log('camera/rgb', rr.Image(img))
    
    def depth_cb(self, msg):
        self._set_time(msg.header)
        depth = self._convert_image(msg, '16UC1')
        if self.ds > 1:
            depth = depth[::self.ds, ::self.ds]
        rr.log('camera/depth', rr.DepthImage(depth, meter=1000.0))
    
    def pose_cb(self, msg):
        self._set_time(msg.header)
        p = msg.pose.position
        q = msg.pose.orientation
        
        rr.log('robot/pose', rr.Transform3D(
            translation=[p.x, p.y, p.z],
            rotation=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w])
        ))
        
        # Trajectory trail
        rr.log('robot/trajectory', rr.Points3D(
            [[p.x, p.y, p.z]],
            colors=[[0, 200, 100]],
            radii=[0.03]
        ))
    
    def tsdf_cb(self, msg):
        self._set_time(msg.header)
        pts_gen = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = np.array(list(pts_gen), dtype=np.float32)
        
        if len(pts) == 0:
            return
        
        if len(pts) > self.max_pts:
            idx = np.random.choice(len(pts), self.max_pts, replace=False)
            pts = pts[idx]
        
        rr.log('map/tsdf', rr.Points3D(
            pts[:, :3],
            colors=[[180, 180, 220]] * len(pts),
            radii=[0.015]
        ))
    
    def yolo_cb(self, msg):
        try:
            data = json.loads(msg.data)
            detections = data.get('detections', [])
            if not detections:
                return
            
            boxes, labels, colors = [], [], []
            for det in detections:
                bbox = det.get('bbox', [])
                if len(bbox) < 4:
                    continue
                
                x1, y1, x2, y2 = bbox[:4]
                boxes.append([
                    x1 / self.ds, y1 / self.ds,
                    (x2 - x1) / self.ds, (y2 - y1) / self.ds
                ])
                
                label = det.get('class_name', '?')
                conf = det.get('confidence', 0.0)
                labels.append(f"{label} {conf:.2f}")
                colors.append(self._hash_color(label))
            
            rr.log('camera/detections', rr.Boxes2D(
                array=boxes,
                array_format=rr.Box2DFormat.XYWH,
                labels=labels,
                colors=colors
            ))
        except Exception as e:
            self.get_logger().warn(f'YOLO log error: {e}')
    
    def _hash_color(self, label):
        """Generate consistent color from label"""
        h = hash(label) % 360
        r = int(128 + 127 * np.sin(np.radians(h)))
        g = int(128 + 127 * np.sin(np.radians(h + 120)))
        b = int(128 + 127 * np.sin(np.radians(h + 240)))
        return [r, g, b]
    
    def world_state_cb(self, msg):
        try:
            data = json.loads(msg.data)
            pts, labels, colors = [], [], []
            
            # Log each tracked object
            for obj_id, obj in data.items():
                pos = obj.get('position', [])
                if len(pos) < 3:
                    continue
                
                pts.append(pos)
                label = obj.get('label', '?')
                conf = obj.get('confidence', 0.0)
                count = obj.get('count', 1)
                labels.append(f"{label} ({conf:.2f}) x{count}")
                
                # Color by source
                source = obj.get('source', '')
                if 'face' in source or 'emotion' in source:
                    colors.append([50, 200, 255])
                elif 'yolo' in source:
                    colors.append([255, 220, 50])
                else:
                    colors.append([200, 200, 200])
            
            if pts:
                rr.log('world/objects', rr.Points3D(
                    pts,
                    labels=labels,
                    colors=colors,
                    radii=[0.06]
                ))
        except Exception as e:
            self.get_logger().warn(f'World state log error: {e}')
    
    def markers_cb(self, msg):
        """Log semantic markers as 3D text labels"""
        try:
            pts, labels, colors = [], [], []
            for marker in msg.markers:
                if marker.type != 9:  # TEXT_VIEW_FACING
                    continue
                
                p = marker.pose.position
                pts.append([p.x, p.y, p.z])
                labels.append(marker.text)
                colors.append([
                    int(marker.color.r * 255),
                    int(marker.color.g * 255),
                    int(marker.color.b * 255)
                ])
            
            if pts:
                rr.log('world/markers', rr.Points3D(
                    pts,
                    labels=labels,
                    colors=colors,
                    radii=[0.05]
                ))
        except Exception as e:
            self.get_logger().warn(f'Markers log error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = RerunLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
