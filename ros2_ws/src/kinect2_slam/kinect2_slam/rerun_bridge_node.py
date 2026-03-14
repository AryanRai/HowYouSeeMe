#!/usr/bin/env python3
"""
Rerun Bridge Node — HowYouSeeMe live visualizer
Modelled after the official rerun_example_ros_node pattern.

Usage (run BEFORE starting this node):
    rerun --connect   # opens viewer, listens for SDK connections

Then launch this node:
    python3 rerun_bridge_node.py --connect
    # or: python3 rerun_bridge_node.py --save /tmp/howyouseeme.rrd
"""
from __future__ import annotations

import argparse
import sys
import threading
import numpy as np

import rerun as rr

try:
    import cv_bridge
    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import Image, PointCloud2
    from sensor_msgs_py import point_cloud2 as pc2
    from geometry_msgs.msg import PoseStamped
    from visualization_msgs.msg import MarkerArray
    from std_msgs.msg import String
except ImportError as e:
    print(f"ROS2 import failed: {e}")
    print("Source /opt/ros/jazzy/setup.bash first")
    sys.exit(1)


def _ros_image_to_numpy(msg: Image) -> np.ndarray:
    """Decode ROS Image without cv_bridge (avoids numpy ABI issues)."""
    dtype_map = {
        'rgb8':  (np.uint8,   3),
        'bgr8':  (np.uint8,   3),
        'mono8': (np.uint8,   1),
        '8UC1':  (np.uint8,   1),
        '8UC3':  (np.uint8,   3),
        '16UC1': (np.uint16,  1),
        '32FC1': (np.float32, 1),
    }
    dtype, ch = dtype_map.get(msg.encoding, (np.uint8, 1))
    arr = np.frombuffer(bytes(msg.data), dtype=dtype)
    arr = arr.reshape((msg.height, msg.width, ch) if ch > 1 else (msg.height, msg.width))
    if msg.encoding == 'bgr8':
        arr = arr[:, :, ::-1].copy()
    return arr


# Source → colour mapping
_SOURCE_COLORS: dict[str, tuple[int, int, int]] = {
    'yolo':        (255, 220,  50),
    'detect':      (255, 220,  50),
    'segment':     (  0, 255, 255),
    'pose':        (255, 128,   0),
    'sam2':        (  0, 200, 255),
    'fastsam':     (  0, 128, 255),
    'insightface': (  0, 255,   0),
    'face':        (  0, 255,   0),
    'emotion':     (255,   0, 255),
}

def _color_for(source: str) -> tuple[int, int, int]:
    for k, v in _SOURCE_COLORS.items():
        if k in source.lower():
            return v
    return (255, 255, 100)


class HowYouSeeMeBridge(Node):
    def __init__(self) -> None:
        super().__init__('rerun_bridge')

        self._trajectory: list[list[float]] = []
        self._traj_lock = threading.Lock()
        self._last_rgb_time = 0.0
        self._last_depth_time = 0.0
        self._last_tsdf_time = 0.0
        self._last_pose_time = 0.0
        self._rgb_interval = 0.5    # 2 Hz
        self._depth_interval = 1.0  # 1 Hz
        self._tsdf_interval = 5.0   # every 5s
        self._pose_interval = 0.1   # 10 Hz

        cbg = ReentrantCallbackGroup()

        reliable = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        best_effort = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        def sub(topic, msg_type, cb, qos=reliable):
            self.create_subscription(msg_type, topic, cb, qos, callback_group=cbg)

        sub('/kinect2/hd/image_color',      Image,        self._rgb_cb,         reliable)
        sub('/kinect2/hd/image_depth_rect', Image,        self._depth_cb,       reliable)
        sub('/orb_slam3/pose',              PoseStamped,  self._pose_cb,        best_effort)
        sub('/tsdf/pointcloud',             PointCloud2,  self._tsdf_cb,        best_effort)
        sub('/semantic/markers',            MarkerArray,  self._markers_cb,     reliable)
        sub('/semantic/world_state',        String,       self._world_state_cb, reliable)
        sub('/cv_pipeline/results',         String,       self._cv_results_cb,  reliable)

        self.get_logger().info('Rerun bridge subscribed to all topics')

    # ── time helper ──────────────────────────────────────────────────────────

    @staticmethod
    def _set_time(stamp) -> None:
        t = Time.from_msg(stamp)
        rr.set_time('ros_time', timestamp=np.datetime64(t.nanoseconds, 'ns'))

    # ── RGB ──────────────────────────────────────────────────────────────────

    def _rgb_cb(self, msg: Image) -> None:
        import time
        now = time.monotonic()
        if now - self._last_rgb_time < self._rgb_interval:
            return
        self._last_rgb_time = now
        try:
            self._set_time(msg.header.stamp)
            img = _ros_image_to_numpy(msg)
            img = img[::4, ::4]  # 4x downsample → 480x270
            rr.log('camera/rgb', rr.Image(img))
        except Exception as e:
            self.get_logger().warn(f'RGB: {e}', throttle_duration_sec=5.0)

    # ── Depth ─────────────────────────────────────────────────────────────────

    def _depth_cb(self, msg: Image) -> None:
        import time
        now = time.monotonic()
        if now - self._last_depth_time < self._depth_interval:
            return
        self._last_depth_time = now
        try:
            self._set_time(msg.header.stamp)
            depth = _ros_image_to_numpy(msg)
            depth = depth[::4, ::4]
            rr.log('camera/depth', rr.DepthImage(depth, meter=1000.0))
        except Exception as e:
            self.get_logger().warn(f'Depth: {e}', throttle_duration_sec=5.0)

    # ── Pose + trajectory ─────────────────────────────────────────────────────

    def _pose_cb(self, msg: PoseStamped) -> None:
        import time
        now = time.monotonic()
        if now - self._last_pose_time < self._pose_interval:
            return
        self._last_pose_time = now
        try:
            self._set_time(msg.header.stamp)
            p, q = msg.pose.position, msg.pose.orientation

            rr.log('world/camera_pose', rr.Transform3D(
                translation=[p.x, p.y, p.z],
                quaternion=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w]),
            ))

            with self._traj_lock:
                self._trajectory.append([p.x, p.y, p.z])
                pts = np.array(self._trajectory, dtype=np.float32)

            rr.log('world/trajectory', rr.LineStrips3D([pts],
                colors=[[0, 220, 100]], radii=[0.008]))

            rr.log('world/camera_pose/axes', rr.Arrows3D(
                origins=[[p.x, p.y, p.z]] * 3,
                vectors=[[0.15, 0, 0], [0, 0.15, 0], [0, 0, 0.15]],
                colors=[[255, 50, 50], [50, 255, 50], [50, 50, 255]],
            ))
        except Exception as e:
            self.get_logger().warn(f'Pose: {e}', throttle_duration_sec=5.0)

    # ── TSDF point cloud ──────────────────────────────────────────────────────

    def _tsdf_cb(self, msg: PointCloud2) -> None:
        import time
        now = time.monotonic()
        if now - self._last_tsdf_time < self._tsdf_interval:
            return
        self._last_tsdf_time = now
        try:
            self._set_time(msg.header.stamp)
            raw = list(pc2.read_points(msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True))
            if not raw:
                return
            raw = raw[:80000]
            pts = np.array([[p[0], p[1], p[2]] for p in raw], dtype=np.float32)
            colors = []
            for p in raw:
                pk = int(p[3]) if not np.isnan(p[3]) else 0
                colors.append([(pk >> 16) & 0xFF, (pk >> 8) & 0xFF, pk & 0xFF])
            rr.log('world/tsdf_map', rr.Points3D(pts,
                colors=np.array(colors, dtype=np.uint8), radii=0.012))
        except Exception as e:
            self.get_logger().warn(f'TSDF: {e}', throttle_duration_sec=5.0)

    # ── Semantic markers ──────────────────────────────────────────────────────

    def _markers_cb(self, msg: MarkerArray) -> None:
        try:
            rr.set_time('ros_time', timestamp=np.datetime64(
                self.get_clock().now().nanoseconds, 'ns'))
            pts, cols, lbls = [], [], []
            for m in msg.markers:
                p = m.pose.position
                pts.append([p.x, p.y, p.z])
                cols.append([int(m.color.r*255), int(m.color.g*255), int(m.color.b*255)])
                lbls.append(m.text)
            if pts:
                rr.log('world/detections', rr.Points3D(
                    np.array(pts, dtype=np.float32),
                    colors=np.array(cols, dtype=np.uint8),
                    labels=lbls, radii=0.06))
        except Exception as e:
            self.get_logger().warn(f'Markers: {e}', throttle_duration_sec=5.0)

    # ── World state ───────────────────────────────────────────────────────────

    def _world_state_cb(self, msg: String) -> None:
        try:
            import json
            data = json.loads(msg.data)
            rr.set_time('ros_time', timestamp=np.datetime64(
                self.get_clock().now().nanoseconds, 'ns'))

            robot = data.get('robot', {})
            rpos = robot.get('position')
            if rpos and len(rpos) >= 3:
                rr.log('world/robot', rr.Points3D([rpos[:3]],
                    colors=[[0, 255, 80]], labels=['robot'], radii=0.1))

            all_ents = {**data.get('objects', {}), **data.get('people', {})}
            if all_ents:
                pts, cols, lbls = [], [], []
                for obj_id, obj in all_ents.items():
                    pos = obj.get('position')
                    if not pos or len(pos) < 3:
                        continue
                    pts.append(pos[:3])
                    cols.append(list(_color_for(obj.get('source', ''))))
                    conf = obj.get('confidence', 0.0)
                    count = obj.get('count', obj.get('times_seen', 1))
                    lbls.append(f"{obj.get('label', obj_id)} ({conf:.2f}) ×{count}")
                if pts:
                    rr.log('world/objects', rr.Points3D(
                        np.array(pts, dtype=np.float32),
                        colors=np.array(cols, dtype=np.uint8),
                        labels=lbls, radii=0.07))

            rr.log('metrics/object_count', rr.Scalars(float(len(data.get('objects', {})))))
            rr.log('metrics/people_count', rr.Scalars(float(len(data.get('people', {})))))
        except Exception as e:
            self.get_logger().warn(f'WorldState: {e}', throttle_duration_sec=5.0)

    # ── CV results ────────────────────────────────────────────────────────────

    def _cv_results_cb(self, msg: String) -> None:
        try:
            import json
            data = json.loads(msg.data)
            rr.set_time('ros_time', timestamp=np.datetime64(
                self.get_clock().now().nanoseconds, 'ns'))
            n = len(data.get('detections', data.get('faces', data.get('mask_stats', []))))
            rr.log('metrics/detections_per_frame', rr.Scalars(float(n)))
        except Exception as e:
            self.get_logger().warn(f'CV: {e}', throttle_duration_sec=5.0)


def main() -> None:
    parser = argparse.ArgumentParser(description='HowYouSeeMe Rerun bridge')
    rr.script_add_args(parser)
    args, ros_args = parser.parse_known_args()
    rr.script_setup(args, 'howyouseeme')

    rclpy.init(args=ros_args)
    node = HowYouSeeMeBridge()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    rr.script_teardown(args)


if __name__ == '__main__':
    main()
