#!/usr/bin/env python3
"""
Point Cloud Denoiser for SLAM Accuracy Improvement
====================================================

Subscribes to the raw Kinect v2 point cloud and publishes a cleaned version
that is used for both SLAM input quality improvement and cleaner RViz
visualisation.

Processing pipeline (in order):
  1. Range filter       – keep only points 0.3 m … 4.0 m from the sensor.
                          Very close reflections and far noise are removed.
  2. Voxel-grid filter  – one representative point per 2 cm³ voxel.
                          Reduces cloud density uniformly so ICP has fewer
                          redundant (and noisy) points to match.
  3. Statistical Outlier Removal (SOR) – for each point, compute the mean
                          distance to its k nearest neighbours; remove points
                          whose mean distance exceeds μ + σ_mult · σ.
                          Eliminates isolated sensor noise that causes "ghost"
                          surfaces in the reconstructed map.

The denoiser publishes a filtered cloud at up to 15 Hz (configurable via
`max_process_hz`), dropping frames rather than building a processing queue
so it stays non-blocking.

Topics
------
  Subscribed : /kinect2/qhd/points          (sensor_msgs/PointCloud2)
  Published  : /kinect2/qhd/points_filtered (sensor_msgs/PointCloud2)

Parameters
----------
  ~input_topic       : str   default '/kinect2/qhd/points'
  ~output_topic      : str   default '/kinect2/qhd/points_filtered'
  ~voxel_size        : float default 0.02   [m]  voxel edge length
  ~min_range         : float default 0.3    [m]  near clip
  ~max_range         : float default 4.0    [m]  far clip
  ~sor_k_neighbors   : int   default 20          SOR neighbourhood size
  ~sor_std_multiplier: float default 1.5         SOR outlier threshold
  ~max_process_hz    : float default 15.0        max processing rate
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class PointCloudDenoiser(Node):
    """ROS 2 node that denoises a Kinect v2 point cloud in real-time."""

    def __init__(self) -> None:
        super().__init__("pointcloud_denoiser")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("input_topic", "/kinect2/qhd/points")
        self.declare_parameter("output_topic", "/kinect2/qhd/points_filtered")
        self.declare_parameter("voxel_size", 0.02)
        self.declare_parameter("min_range", 0.3)
        self.declare_parameter("max_range", 4.0)
        self.declare_parameter("sor_k_neighbors", 20)
        self.declare_parameter("sor_std_multiplier", 1.5)
        self.declare_parameter("max_process_hz", 15.0)

        self._input_topic = self.get_parameter("input_topic").value
        self._output_topic = self.get_parameter("output_topic").value
        self._voxel_size = float(self.get_parameter("voxel_size").value)
        self._min_range = float(self.get_parameter("min_range").value)
        self._max_range = float(self.get_parameter("max_range").value)
        self._sor_k = int(self.get_parameter("sor_k_neighbors").value)
        self._sor_std = float(self.get_parameter("sor_std_multiplier").value)
        max_hz = float(self.get_parameter("max_process_hz").value)
        self._min_interval = 1.0 / max_hz if max_hz > 0 else 0.0

        self._last_process_time: float = 0.0

        # Check if scipy is available for SOR
        try:
            from scipy.spatial import cKDTree
            self._scipy_available = True
        except ImportError:
            self._scipy_available = False
            self.get_logger().warn(
                "scipy not found – Statistical Outlier Removal disabled. "
                "Install with: pip install scipy"
            )

        # ── QoS matching Kinect best-effort publisher ────────────────────────
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._sub = self.create_subscription(
            PointCloud2, self._input_topic, self._callback, sensor_qos
        )
        self._pub = self.create_publisher(PointCloud2, self._output_topic, 10)

        self.get_logger().info(
            f"PointCloudDenoiser started\n"
            f"  input  : {self._input_topic}\n"
            f"  output : {self._output_topic}\n"
            f"  voxel  : {self._voxel_size * 100:.0f} cm\n"
            f"  range  : {self._min_range} m – {self._max_range} m\n"
            f"  SOR    : k={self._sor_k}, σ×{self._sor_std} "
            f"({'ON' if self._scipy_available else 'OFF – no scipy'})\n"
            f"  max Hz : {max_hz:.0f}"
        )

    # ────────────────────────────────────────────────────────────────────────
    # Main callback
    # ────────────────────────────────────────────────────────────────────────

    def _callback(self, msg: PointCloud2) -> None:
        # Rate-limit processing to avoid CPU saturation
        now = time.monotonic()
        if now - self._last_process_time < self._min_interval:
            return
        self._last_process_time = now

        try:
            filtered = self._process(msg)
            if filtered is not None:
                self._pub.publish(filtered)
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn(
                f"Denoiser error: {exc}", throttle_duration_sec=5.0
            )

    # ────────────────────────────────────────────────────────────────────────
    # Processing pipeline
    # ────────────────────────────────────────────────────────────────────────

    def _process(self, msg: PointCloud2):
        # Determine available fields
        field_names = {f.name for f in msg.fields}
        read_fields = ["x", "y", "z"]
        has_rgb = "rgb" in field_names
        if has_rgb:
            read_fields.append("rgb")

        # Read into structured numpy array
        pts = pc2.read_points(msg, field_names=read_fields, skip_nans=True)
        if pts is None or len(pts) == 0:
            return None

        pts = np.array(pts)  # structured array
        xyz = np.column_stack([pts["x"], pts["y"], pts["z"]]).astype(np.float32)
        rgb = pts["rgb"] if has_rgb else None

        # ── Step 1: Range filter ─────────────────────────────────────────────
        dist = np.linalg.norm(xyz, axis=1)
        mask = (dist >= self._min_range) & (dist <= self._max_range) & np.isfinite(dist)
        xyz = xyz[mask]
        if rgb is not None:
            rgb = rgb[mask]
        if len(xyz) < 5:
            return None

        # ── Step 2: Voxel-grid downsampling ─────────────────────────────────
        xyz, rgb = self._voxel_filter(xyz, rgb)
        if len(xyz) < 5:
            return None

        # ── Step 3: Statistical Outlier Removal ──────────────────────────────
        if self._scipy_available and len(xyz) > self._sor_k + 1:
            xyz, rgb = self._sor_filter(xyz, rgb)
        if len(xyz) == 0:
            return None

        # ── Reconstruct PointCloud2 ──────────────────────────────────────────
        return self._build_msg(msg, xyz, rgb)

    # ────────────────────────────────────────────────────────────────────────
    # Helpers
    # ────────────────────────────────────────────────────────────────────────

    def _voxel_filter(self, xyz: np.ndarray, rgb):
        """Return one representative point per voxel cell."""
        coords = np.floor(xyz / self._voxel_size).astype(np.int32)
        # Pack (ix, iy, iz) into a single 64-bit integer for fast unique
        # Use bit-shifting: assume coords fit in 20 bits each (±1 048 576 cells)
        packed = (
            coords[:, 0].astype(np.int64) * (1 << 40)
            + coords[:, 1].astype(np.int64) * (1 << 20)
            + coords[:, 2].astype(np.int64)
        )
        _, unique_idx = np.unique(packed, return_index=True)
        xyz_out = xyz[unique_idx]
        rgb_out = rgb[unique_idx] if rgb is not None else None
        return xyz_out, rgb_out

    def _sor_filter(self, xyz: np.ndarray, rgb):
        """Remove statistical outliers using scipy cKDTree."""
        from scipy.spatial import cKDTree

        k = min(self._sor_k + 1, len(xyz))
        tree = cKDTree(xyz)
        dists, _ = tree.query(xyz, k=k, workers=-1)
        mean_dists = dists[:, 1:].mean(axis=1)
        threshold = mean_dists.mean() + self._sor_std * mean_dists.std()
        mask = mean_dists < threshold
        return xyz[mask], (rgb[mask] if rgb is not None else None)

    def _build_msg(self, original: PointCloud2, xyz: np.ndarray, rgb) -> PointCloud2:
        """Assemble a new PointCloud2 from filtered xyz (and optional rgb)."""
        header = original.header
        if rgb is not None:
            # Re-use original fields list so downstream consumers see rgb
            n = len(xyz)
            out_pts = np.zeros(
                n,
                dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32), ("rgb", np.float32)],
            )
            out_pts["x"] = xyz[:, 0]
            out_pts["y"] = xyz[:, 1]
            out_pts["z"] = xyz[:, 2]
            out_pts["rgb"] = rgb  # already float32 from structured array
            return pc2.create_cloud(header, original.fields, out_pts)
        else:
            return pc2.create_cloud_xyz32(header, xyz)


# ────────────────────────────────────────────────────────────────────────────


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointCloudDenoiser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
