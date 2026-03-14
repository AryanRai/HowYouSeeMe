#!/usr/bin/env python3
"""
Rerun Bridge Node — HowYouSeeMe live visualizer
Subscribes to all system topics and streams to Rerun viewer.

Topics consumed:
  /kinect2/hd/image_color          → camera/rgb
  /kinect2/hd/image_depth_rect     → camera/depth
  /orb_slam3/pose                  → world/camera_pose + world/trajectory
  /tsdf/pointcloud                 → world/tsdf_map
  /semantic/markers                → world/detections
  /semantic/world_state            → world/objects (labeled 3D points)
  /cv_pipeline/results             → metrics/detections_per_frame
"""

import json
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String
import rerun as rr


def _ros_image_to_numpy(msg: Image) -> np.ndarray:
    """Convert ROS Image msg to numpy array without cv_bridge."""
    dtype_map = {
        'rgb8':   (np.uint8,  3),
        'bgr8':   (np.uint8,  3),
        'rgba8':  (np.uint8,  4),
        'mono8':  (np.uint8,  1),
        '8UC1':   (np.uint8,  1),
        '8UC3':   (np.uint8,  3),
        '16UC1':  (np.uint16, 1),
        '32FC1':  (np.float32, 1),
    }
    dtype, channels = dtype_map.get(msg.encoding, (np.uint8, 1))
    arr = np.frombuffer(bytes(msg.data), dtype=dtype)
    if channels > 1:
        arr = arr.reshape((msg.height, msg.width, channels))
    else:
        arr = arr.reshape((msg.height, msg.width))
    # bgr → rgb
    if msg.encoding == 'bgr8':
        arr = arr[:, :, ::-1].copy()
    return arr


# Colour palette per detection source (matches semantic_projection_node)
_SOURCE_COLORS = {
    'yolo':        (255, 220,  50),
    'detect':      (255, 220,  50),
    'segment':     (  0, 255, 255),
    'pose':        (255, 128,   0),
    'obb':         (128, 255,   0),
    'sam2':        (  0, 200, 255),
    'fastsam':     (  0, 128, 255),
    'insightface': (  0, 255,   0),
    'face':        (  0, 255,   0),
    'emotion':     (255,   0, 255),
}
_DEFAULT_COLOR = (255, 255, 100)


def _color_for(source: str):
    for key, rgb in _SOURCE_COLORS.items():
        if key in source.lower():
            return rgb
    return _DEFAULT_COLOR


class RerunBridgeNode(Node):
    def __init__(self):
        super().__init__('rerun_bridge')

        self.declare_parameter('rgb_downsample', 2)
        self.declare_parameter('pc_max_points', 80000)
        self.declare_parameter('recording_name', 'howyouseeme')

        self._traj_lock = threading.Lock()
        self._trajectory: list[list[float]] = []

        name = self.get_parameter('recording_name').value
        self.rgb_ds = self.get_parameter('rgb_downsample').value
        self.pc_max = self.get_parameter('pc_max_points').value

        # Init rerun — spawn viewer, also save .rrd for replay
        rr.init(name, spawn=True)
        rr.save('/tmp/howyouseeme_live.rrd')

        # Blueprint: log coordinate axes once
        rr.log('world', rr.ViewCoordinates.RIGHT_HAND_Y_UP, static=True)

        qos_be = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(Image,        '/kinect2/hd/image_color',       self._rgb_cb,        qos_be)
        self.create_subscription(Image,        '/kinect2/hd/image_depth_rect',  self._depth_cb,      qos_be)
        self.create_subscription(PoseStamped,  '/orb_slam3/pose',               self._pose_cb,       qos_be)
        self.create_subscription(PointCloud2,  '/tsdf/pointcloud',              self._tsdf_cb,       qos_be)
        self.create_subscription(MarkerArray,  '/semantic/markers',             self._markers_cb,    qos_rel)
        self.create_subscription(String,       '/semantic/world_state',         self._world_state_cb, qos_rel)
        self.create_subscription(String,       '/cv_pipeline/results',          self._cv_results_cb, qos_rel)

        self.get_logger().info(f'Rerun bridge ready → recording "{name}"')
        self.get_logger().info('Viewer spawned. Streaming all topics to Rerun.')

    # ── helpers ──────────────────────────────────────────────────────────────

    def _stamp_to_sec(self, stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9

    def _set_time(self, stamp):
        rr.set_time('ros_time', timestamp=self._stamp_to_sec(stamp))

    # ── RGB image ─────────────────────────────────────────────────────────────

    def _rgb_cb(self, msg: Image):
        try:
            self._set_time(msg.header.stamp)
            img = _ros_image_to_numpy(msg)
            if self.rgb_ds > 1:
                img = img[::self.rgb_ds, ::self.rgb_ds]
            rr.log('camera/rgb', rr.Image(img))
        except Exception as e:
            self.get_logger().warn(f'RGB: {e}', throttle_duration_sec=5.0)

    # ── Depth image ───────────────────────────────────────────────────────────

    def _depth_cb(self, msg: Image):
        try:
            self._set_time(msg.header.stamp)
            depth = _ros_image_to_numpy(msg)
            if self.rgb_ds > 1:
                depth = depth[::self.rgb_ds, ::self.rgb_ds]
            rr.log('camera/depth', rr.DepthImage(depth, meter=1000.0))
        except Exception as e:
            self.get_logger().warn(f'Depth: {e}', throttle_duration_sec=5.0)

    # ── Camera pose + trajectory ──────────────────────────────────────────────

    def _pose_cb(self, msg: PoseStamped):
        try:
            self._set_time(msg.header.stamp)
            p = msg.pose.position
            q = msg.pose.orientation

            rr.log('world/camera_pose', rr.Transform3D(
                translation=[p.x, p.y, p.z],
                quaternion=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w]),
            ))

            # Accumulate trajectory
            with self._traj_lock:
                self._trajectory.append([p.x, p.y, p.z])
                pts = np.array(self._trajectory, dtype=np.float32)

            rr.log('world/trajectory', rr.LineStrips3D(
                [pts],
                colors=[[0, 220, 100]],
                radii=[0.008],
            ))

            # Camera frustum as a small axes gizmo
            rr.log('world/camera_pose/axes', rr.Arrows3D(
                origins=[[p.x, p.y, p.z]] * 3,
                vectors=[[0.15, 0, 0], [0, 0.15, 0], [0, 0, 0.15]],
                colors=[[255, 50, 50], [50, 255, 50], [50, 50, 255]],
            ))
        except Exception as e:
            self.get_logger().warn(f'Pose: {e}', throttle_duration_sec=5.0)

    # ── TSDF point cloud ──────────────────────────────────────────────────────

    def _tsdf_cb(self, msg: PointCloud2):
        try:
            self._set_time(msg.header.stamp)

            # Read XYZ + packed RGB
            pts_raw = list(pc2.read_points(
                msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True))

            if not pts_raw:
                return

            pts_raw = pts_raw[:self.pc_max]
            pts = np.array([[p[0], p[1], p[2]] for p in pts_raw], dtype=np.float32)

            # Unpack RGB from float-encoded uint32
            colors = []
            for p in pts_raw:
                packed = int(p[3]) if not np.isnan(p[3]) else 0
                r = (packed >> 16) & 0xFF
                g = (packed >> 8)  & 0xFF
                b =  packed        & 0xFF
                colors.append([r, g, b])
            colors = np.array(colors, dtype=np.uint8)

            rr.log('world/tsdf_map', rr.Points3D(pts, colors=colors, radii=0.012))
        except Exception as e:
            self.get_logger().warn(f'TSDF: {e}', throttle_duration_sec=5.0)

    # ── Semantic markers (RViz MarkerArray) ───────────────────────────────────

    def _markers_cb(self, msg: MarkerArray):
        try:
            now = self.get_clock().now().to_msg()
            self._set_time(now)

            points, colors, labels = [], [], []
            for m in msg.markers:
                p = m.pose.position
                points.append([p.x, p.y, p.z])
                colors.append([
                    int(m.color.r * 255),
                    int(m.color.g * 255),
                    int(m.color.b * 255),
                ])
                labels.append(m.text)

            if points:
                rr.log('world/detections', rr.Points3D(
                    np.array(points, dtype=np.float32),
                    colors=np.array(colors, dtype=np.uint8),
                    labels=labels,
                    radii=0.06,
                ))
        except Exception as e:
            self.get_logger().warn(f'Markers: {e}', throttle_duration_sec=5.0)

    # ── World state JSON ──────────────────────────────────────────────────────

    def _world_state_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            now = self.get_clock().now().to_msg()
            self._set_time(now)

            # Robot position
            robot = data.get('robot', {})
            rpos = robot.get('position', None)
            if rpos and len(rpos) >= 3:
                rr.log('world/robot', rr.Points3D(
                    [rpos[:3]],
                    colors=[[0, 255, 80]],
                    labels=['robot'],
                    radii=0.1,
                ))

            # Objects
            objects = data.get('objects', {})
            people  = data.get('people', {})
            all_entities = {**objects, **people}

            if all_entities:
                pts, cols, lbls = [], [], []
                for obj_id, obj in all_entities.items():
                    pos = obj.get('position', None)
                    if not pos or len(pos) < 3:
                        continue
                    pts.append(pos[:3])
                    src = obj.get('source', '')
                    cols.append(list(_color_for(src)))
                    label = obj.get('label', obj_id)
                    conf  = obj.get('confidence', 0.0)
                    count = obj.get('count', obj.get('times_seen', 1))
                    lbls.append(f'{label} ({conf:.2f}) ×{count}')

                if pts:
                    rr.log('world/objects', rr.Points3D(
                        np.array(pts, dtype=np.float32),
                        colors=np.array(cols, dtype=np.uint8),
                        labels=lbls,
                        radii=0.07,
                    ))

            # Object count scalar
            rr.log('metrics/object_count', rr.Scalars(float(len(objects))))
            rr.log('metrics/people_count', rr.Scalars(float(len(people))))

        except Exception as e:
            self.get_logger().warn(f'WorldState: {e}', throttle_duration_sec=5.0)

    # ── CV pipeline results (detection count per frame) ───────────────────────

    def _cv_results_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            now = self.get_clock().now().to_msg()
            self._set_time(now)
            n = len(data.get('detections', data.get('faces', data.get('mask_stats', []))))
            rr.log('metrics/detections_per_frame', rr.Scalars(float(n)))
        except Exception as e:
            self.get_logger().warn(f'CV results: {e}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = RerunBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
