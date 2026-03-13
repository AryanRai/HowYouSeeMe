#!/usr/bin/env python3
"""
Phase 4: Semantic Projection Node
Back-projects ALL CV pipeline detections into 3D world space using depth + TF2.

Handles outputs from:
  - YOLO11 (detect, segment, pose, obb)
  - SAM2 / FastSAM (segmentation masks)
  - InsightFace (face detection & recognition)
  - HSEmotion (emotion recognition via InsightFace analyze mode)

For every detected entity the node:
  1. Back-projects the 2D bbox centroid to a 3D point in the camera frame.
  2. Transforms to the world frame using the ORB-SLAM3 camera pose.
  3. Broadcasts a TF2 child frame for the entity (e.g. ``yolo_person_0``).
  4. Broadcasts the camera pose itself as TF2 frame ``camera_link``.
  5. Publishes a coloured RViz TEXT_VIEW_FACING marker.
  6. Maintains a persistent world-state dict and saves it as JSON every 5 s.
"""

import threading
import rclpy
from rclpy.node import Node
import json
import time
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial.transform import Rotation
import tf2_ros


# ── Per-source marker colours (R, G, B) ──────────────────────────────────────
_MODEL_COLORS = {
    'yolo11':      (1.0, 1.0, 0.0),   # yellow
    'yolo':        (1.0, 1.0, 0.0),
    'detect':      (1.0, 1.0, 0.0),
    'segment':     (0.0, 1.0, 1.0),   # cyan
    'pose':        (1.0, 0.5, 0.0),   # orange
    'obb':         (0.5, 1.0, 0.0),   # lime
    'sam2':        (0.0, 0.8, 1.0),   # sky-blue
    'fastsam':     (0.0, 0.5, 1.0),   # blue
    'insightface': (0.0, 1.0, 0.0),   # green
    'face':        (0.0, 1.0, 0.0),
    'emotion':     (1.0, 0.0, 1.0),   # magenta
}

_DEFAULT_COLOR = (1.0, 1.0, 0.0)


def _color_for(source: str):
    for key, rgb in _MODEL_COLORS.items():
        if key in source.lower():
            return rgb
    return _DEFAULT_COLOR


class SemanticProjection(Node):
    def __init__(self):
        super().__init__('semantic_projection')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('fx', 1081.37)
        self.declare_parameter('fy', 1081.37)
        self.declare_parameter('cx', 959.5)
        self.declare_parameter('cy', 539.5)
        self.declare_parameter('world_state_path', '/tmp/world_state.json')
        self.declare_parameter('marker_lifetime', 30.0)
        self.declare_parameter('conf_threshold', 0.4)
        self.declare_parameter('depth_trunc', 5.0)
        self.declare_parameter('merge_threshold', 0.5)
        self.declare_parameter('debug_projection', False)

        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value

        self.bridge = CvBridge()
        self.world_state = {}
        self.marker_id = 0
        self.latest_pose = None
        self._pose_lock = threading.Lock()

        # ── TF2 broadcaster ───────────────────────────────────────────────────
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── Subscribers ───────────────────────────────────────────────────────
        # ORB-SLAM3 camera pose (latched; not time-synced with detections)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/orb_slam3/pose', self.pose_callback, 10)

        # CV pipeline results (String messages don't have timestamps)
        # Store latest depth image separately
        self.latest_depth = None
        self.depth_lock = threading.Lock()
        
        self.depth_sub = self.create_subscription(
            Image, '/kinect2/hd/image_depth_rect', self.depth_callback, 10)
        
        self.cv_sub = self.create_subscription(
            String, '/cv_pipeline/results', self.detection_cb, 10)

        # ── Publishers ────────────────────────────────────────────────────────
        self.marker_pub = self.create_publisher(
            MarkerArray, '/semantic/markers', 10)
        self.state_pub = self.create_publisher(
            String, '/semantic/world_state', 10)

        # ── Timers ────────────────────────────────────────────────────────────
        self.create_timer(5.0, self.publish_world_state)

        self.get_logger().info('Semantic projection ready (all CV models)')
        self.get_logger().info(
            f'Camera intrinsics: fx={self.fx}, fy={self.fy}, '
            f'cx={self.cx}, cy={self.cy}')
        self.get_logger().info(
            f'Confidence threshold: {self.get_parameter("conf_threshold").value}')

    # ── ORB-SLAM3 pose callback ───────────────────────────────────────────────

    def pose_callback(self, msg: PoseStamped):
        """Store latest ORB-SLAM3 pose and broadcast it as TF2 camera_link."""
        with self._pose_lock:
            self.latest_pose = msg

        # Broadcast map → camera_link
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'camera_link'
        tf.transform.translation.x = msg.pose.position.x
        tf.transform.translation.y = msg.pose.position.y
        tf.transform.translation.z = msg.pose.position.z
        tf.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(tf)

    def depth_callback(self, msg: Image):
        """Store latest depth image."""
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            with self.depth_lock:
                self.latest_depth = cv_depth
        except Exception as e:
            self.get_logger().error(f'Depth callback error: {e}')

    # ── Geometry helpers ─────────────────────────────────────────────────────

    def pixel_to_3d(self, u, v, depth_image):
        """Back-project pixel (u, v) to a 3-D point in the camera frame."""
        depth_trunc = self.get_parameter('depth_trunc').value
        h, w = depth_image.shape
        if not (0 <= int(v) < h and 0 <= int(u) < w):
            return None
        Z = float(depth_image[int(v), int(u)]) / 1000.0   # mm → m
        if Z <= 0.0 or Z > depth_trunc:
            return None
        X = (u - self.cx) * Z / self.fx
        Y = (v - self.cy) * Z / self.fy
        return np.array([X, Y, Z])

    def camera_to_world(self, xyz_camera):
        """Transform a 3-D camera-frame point to the world frame."""
        with self._pose_lock:
            pose_snapshot = self.latest_pose
        if pose_snapshot is None:
            self.get_logger().warn(
                'No ORB-SLAM3 pose yet', throttle_duration_sec=5.0)
            return None
        pose = pose_snapshot.pose
        q = [pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w]
        R = Rotation.from_quat(q).as_matrix()
        t = np.array([pose.position.x, pose.position.y, pose.position.z])
        return R @ xyz_camera + t

    def _get_bbox_3d_position(self, u, v, bbox, depth_image):
        """
        Get robust 3D position using median depth from bbox region.
        Falls back to center point if bbox is None.
        """
        if bbox is None:
            return self.pixel_to_3d(u, v, depth_image)
        
        depth_trunc = self.get_parameter('depth_trunc').value
        h, w = depth_image.shape
        
        # Sample depth values in a region around the bbox center
        # Use 25% of bbox size as sampling region
        x1, y1, x2, y2 = bbox
        sample_w = max(10, int((x2 - x1) * 0.25))
        sample_h = max(10, int((y2 - y1) * 0.25))
        
        u_min = max(0, int(u - sample_w // 2))
        u_max = min(w, int(u + sample_w // 2))
        v_min = max(0, int(v - sample_h // 2))
        v_max = min(h, int(v + sample_h // 2))
        
        # Extract depth patch and get valid depths
        depth_patch = depth_image[v_min:v_max, u_min:u_max]
        valid_depths = depth_patch[depth_patch > 0] / 1000.0  # mm → m
        valid_depths = valid_depths[valid_depths < depth_trunc]
        
        if len(valid_depths) == 0:
            # Fall back to center pixel
            return self.pixel_to_3d(u, v, depth_image)
        
        # Use median depth for robustness against outliers
        Z = float(np.median(valid_depths))
        X = (u - self.cx) * Z / self.fx
        Y = (v - self.cy) * Z / self.fy
        
        return np.array([X, Y, Z])

    def _broadcast_object_tf(self, frame_id: str, xyz_world, stamp):
        """Publish a TF2 frame for a detected object at ``xyz_world``."""
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'map'
        tf.child_frame_id = frame_id
        tf.transform.translation.x = float(xyz_world[0])
        tf.transform.translation.y = float(xyz_world[1])
        tf.transform.translation.z = float(xyz_world[2])
        # Identity rotation – object frames are axis-aligned with the world
        tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf)

    # ── Marker builder ───────────────────────────────────────────────────────

    def _make_marker(self, xyz_world, label: str, source: str,
                     conf: float, stamp) -> Marker:
        lifetime = self.get_parameter('marker_lifetime').value
        r, g, b = _color_for(source)
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = stamp
        m.ns = f'semantic/{source}'
        m.id = self.marker_id
        self.marker_id += 1
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = float(xyz_world[0])
        m.pose.position.y = float(xyz_world[1])
        m.pose.position.z = float(xyz_world[2]) + 0.15
        m.pose.orientation.w = 1.0
        m.scale.z = 0.12
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 1.0
        if conf > 0.0:
            m.text = f'{label} ({conf:.2f})'
        else:
            m.text = label
        m.lifetime.sec = int(lifetime)
        m.lifetime.nanosec = int((lifetime - int(lifetime)) * 1e9)
        return m

    # ── World-state update ───────────────────────────────────────────────────

    def _update_world_state(self, obj_id: str, label: str, xyz_world,
                            conf: float, source: str, extra: dict | None = None):
        if obj_id in self.world_state:
            self.world_state[obj_id]['count'] += 1
            self.world_state[obj_id]['last_seen'] = time.time()
            self.world_state[obj_id]['confidence'] = max(
                self.world_state[obj_id]['confidence'], conf)
        else:
            entry = {
                'label': label,
                'source': source,
                'position': xyz_world.tolist(),
                'confidence': conf,
                'last_seen': time.time(),
                'count': 1,
            }
            if extra:
                entry.update(extra)
            self.world_state[obj_id] = entry

    # ── Unified detection callback ────────────────────────────────────────────

    def detection_cb(self, cv_msg: String):
        """
        Receive CV pipeline results.
        Dispatches to the appropriate parser based on the ``model`` or
        ``mode`` field inside the JSON payload.
        """
        # Get latest depth image
        with self.depth_lock:
            depth = self.latest_depth
        
        if depth is None:
            self.get_logger().warn('No depth image available yet', 
                                  throttle_duration_sec=5.0)
            return
        
        try:
            data = json.loads(cv_msg.data)
        except Exception as e:
            self.get_logger().warn(f'Parse error: {e}')
            return

        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()

        # Detect which model produced this result
        model = data.get('model', data.get('mode', '')).lower()

        if model in ('sam2', 'fastsam'):
            self._process_mask_results(data, depth, stamp, markers, model)
        elif 'face' in model or model == 'insightface' or 'faces' in data:
            self._process_face_results(data, depth, stamp, markers)
        else:
            # YOLO (detect / segment / pose / obb) or unknown — look for
            # 'detections' key which is common to all YOLO tasks
            self._process_detection_results(data, depth, stamp, markers)

        if markers.markers:
            self.marker_pub.publish(markers)
            self.get_logger().info(
                f'Published {len(markers.markers)} markers from [{model}]',
                throttle_duration_sec=2.0)

    # ── Per-model parsers ─────────────────────────────────────────────────────

    def _process_detection_results(self, data: dict, depth, stamp, markers):
        """Handle YOLO detect / segment / pose / obb results."""
        task = data.get('task', data.get('model', 'detect'))
        thresh = self.get_parameter('conf_threshold').value
        debug = self.get_parameter('debug_projection').value

        for det in data.get('detections', []):
            conf = det.get('confidence', 0.0)
            if conf < thresh:
                continue

            label = det.get('class_name', 'unknown')

            # For OBB the bounding box is stored as four corner points
            obb = det.get('obb')
            if obb:
                pts = np.array(obb)
                u = float(np.mean(pts[:, 0]))
                v = float(np.mean(pts[:, 1]))
            else:
                bbox = det.get('bbox', [])
                if len(bbox) < 4:
                    continue
                # Use center of bounding box
                u = (bbox[0] + bbox[2]) / 2.0
                v = (bbox[1] + bbox[3]) / 2.0
                
                if debug:
                    self.get_logger().info(
                        f'Detection: {label} bbox=[{bbox[0]:.0f},{bbox[1]:.0f},{bbox[2]:.0f},{bbox[3]:.0f}] '
                        f'center=({u:.0f},{v:.0f})',
                        throttle_duration_sec=2.0)

            # Get 3D position using median depth in bbox region for robustness
            xyz_cam = self._get_bbox_3d_position(u, v, bbox if not obb else None, depth)
            if xyz_cam is None:
                continue
            xyz_world = self.camera_to_world(xyz_cam)
            if xyz_world is None:
                continue

            source = f'yolo_{task}'
            obj_id = (f'{label}_{int(xyz_world[0]*10)}_'
                      f'{int(xyz_world[1]*10)}')

            extra = {}
            # Pose keypoints → store in world state for downstream use
            if task == 'pose' and 'keypoints' in det:
                extra['keypoints_2d'] = det['keypoints']
            # Segmentation mask_id reference
            if 'mask_id' in det:
                extra['mask_id'] = det['mask_id']

            self._update_world_state(obj_id, label, xyz_world,
                                     conf, source, extra)
            tf_id = f'yolo_{label}_{obj_id[-8:]}'.replace(' ', '_')
            self._broadcast_object_tf(tf_id, xyz_world, stamp)
            markers.markers.append(
                self._make_marker(xyz_world, label, source, conf, stamp))

    def _process_mask_results(self, data: dict, depth, stamp, markers,
                              model: str):
        """Handle SAM2 / FastSAM mask-stats results."""
        for stat in data.get('mask_stats', []):
            bbox = stat.get('bbox', [])   # [x, y, w, h]
            if len(bbox) < 4 or (bbox[2] == 0 and bbox[3] == 0):
                continue
            u = bbox[0] + bbox[2] / 2.0
            v = bbox[1] + bbox[3] / 2.0
            score = stat.get('score', 0.0)

            xyz_cam = self.pixel_to_3d(u, v, depth)
            if xyz_cam is None:
                continue
            xyz_world = self.camera_to_world(xyz_cam)
            if xyz_world is None:
                continue

            mask_id = stat.get('id', 0)
            label = f'{model}_seg_{mask_id}'
            obj_id = (f'{model}_{mask_id}_'
                      f'{int(xyz_world[0]*10)}_{int(xyz_world[1]*10)}')

            self._update_world_state(obj_id, label, xyz_world,
                                     score, model,
                                     {'area': stat.get('area', 0)})
            tf_id = f'{model}_seg_{mask_id}'.replace(' ', '_')
            self._broadcast_object_tf(tf_id, xyz_world, stamp)
            markers.markers.append(
                self._make_marker(xyz_world, label, model, score, stamp))

    def _process_face_results(self, data: dict, depth, stamp, markers):
        """Handle InsightFace (detect / detect_recognize / analyze) results."""
        thresh = self.get_parameter('conf_threshold').value

        for i, face in enumerate(data.get('faces', [])):
            conf = face.get('det_score', face.get('confidence', 0.0))
            if conf < thresh:
                continue

            bbox = face.get('bbox', [])
            if len(bbox) < 4:
                continue
            u = (bbox[0] + bbox[2]) / 2.0
            v = (bbox[1] + bbox[3]) / 2.0

            xyz_cam = self.pixel_to_3d(u, v, depth)
            if xyz_cam is None:
                continue
            xyz_world = self.camera_to_world(xyz_cam)
            if xyz_world is None:
                continue

            # Build a human-readable label
            name = face.get('name', face.get('person_id', f'face_{i}'))
            emotion = face.get('emotion', '')
            label = name
            if emotion:
                label = f'{name} [{emotion}]'

            # Determine source tag for colour-coding
            if emotion:
                source = 'emotion'
            else:
                source = 'insightface'

            obj_id = (f'face_{name}_{int(xyz_world[0]*10)}_'
                      f'{int(xyz_world[1]*10)}')

            extra = {}
            if face.get('age'):
                extra['age'] = face['age']
            if face.get('gender'):
                extra['gender'] = face['gender']
            if emotion:
                extra['emotion'] = emotion
                extra['emotion_confidence'] = face.get(
                    'emotion_confidence', 0.0)

            self._update_world_state(obj_id, label, xyz_world,
                                     conf, source, extra)
            tf_id = f'face_{name}_{i}'.replace(' ', '_')
            self._broadcast_object_tf(tf_id, xyz_world, stamp)
            markers.markers.append(
                self._make_marker(xyz_world, label, source, conf, stamp))

    # ── World-state periodic publisher ───────────────────────────────────────

    def publish_world_state(self):
        """Save world state to JSON and publish on /semantic/world_state."""
        path = self.get_parameter('world_state_path').value
        try:
            with open(path, 'w') as f:
                json.dump(self.world_state, f, indent=2)
            msg = String()
            msg.data = json.dumps(self.world_state)
            self.state_pub.publish(msg)
            if self.world_state:
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
