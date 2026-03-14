#!/usr/bin/env python3
"""
Live Enrichment Node
Watches /cv_pipeline/results (YOLO detections) and automatically fires
follow-up models on the same frame:

  person detected  → YOLO pose (keypoints)
  any object       → YOLO segmentation
  person detected  → InsightFace analyze (if available)

Publishes enriched results to /cv_pipeline/enriched (String JSON).
The world synthesiser and rerun bridge subscribe to this for richer data.

Rate-limited to avoid GPU contention with the primary YOLO stream.
"""
import sys
import threading
import time
import json
import numpy as np
import cv2

sys.path.insert(0, '/home/aryan/Documents/GitHub/HowYouSeeMe/ros2_ws/src/cv_pipeline/python')

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image

_WS = '/home/aryan/Documents/GitHub/HowYouSeeMe'

# Lazy model cache — loaded once on first use
_models: dict = {}
_model_lock = threading.Lock()

def _get_model(name: str):
    with _model_lock:
        if name not in _models:
            from ultralytics import YOLO
            paths = {
                'pose': f'{_WS}/yolo11n-pose.pt',
                'seg':  f'{_WS}/yolo11n-seg.pt',
            }
            _models[name] = YOLO(paths[name])
        return _models[name]

_insightface_worker = None
_insightface_lock = threading.Lock()

def _get_insightface():
    global _insightface_worker
    with _insightface_lock:
        if _insightface_worker is None:
            from insightface_worker import InsightFaceWorker
            w = InsightFaceWorker(device='cuda')
            w.load_models('buffalo_l')
            w.prepare(det_size=(640, 640), det_thresh=0.3)
            _insightface_worker = w
        return _insightface_worker


def _ros_image_to_numpy(msg: Image) -> np.ndarray:
    dtype_map = {
        'rgb8':  (np.uint8, 3), 'bgr8': (np.uint8, 3),
        'mono8': (np.uint8, 1), '8UC3': (np.uint8, 3),
    }
    dtype, ch = dtype_map.get(msg.encoding, (np.uint8, 3))
    arr = np.frombuffer(bytes(msg.data), dtype=dtype)
    arr = arr.reshape((msg.height, msg.width, ch) if ch > 1 else (msg.height, msg.width))
    if msg.encoding == 'bgr8':
        arr = arr[:, :, ::-1].copy()
    return arr


class LiveEnrichmentNode(Node):
    def __init__(self):
        super().__init__('live_enrichment')

        # Rate limiting — don't run enrichment faster than this
        self.declare_parameter('enrichment_interval', 1.0)  # seconds between enrichments
        self._interval = self.get_parameter('enrichment_interval').value
        self._last_enrichment = 0.0
        self._processing = False

        self._latest_rgb: np.ndarray | None = None
        self._rgb_lock = threading.Lock()

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        be       = QoSProfile(depth=5,  reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(String, '/cv_pipeline/results',         self._on_yolo,  reliable)
        self.create_subscription(Image,  '/kinect2/hd/image_color',      self._on_rgb,   be)
        self.create_subscription(Image,  '/kinect2/qhd/image_color',     self._on_rgb,   be)

        self._pub = self.create_publisher(String, '/cv_pipeline/enriched', 10)

        self.get_logger().info('Live enrichment node started')

    def _on_rgb(self, msg: Image) -> None:
        try:
            img = _ros_image_to_numpy(msg)
            with self._rgb_lock:
                self._latest_rgb = img
        except Exception:
            pass

    def _on_yolo(self, msg: String) -> None:
        now = time.monotonic()
        if self._processing:
            return
        if now - self._last_enrichment < self._interval:
            return

        try:
            data = json.loads(msg.data)
        except Exception:
            return

        if 'error' in data:
            return

        detections = data.get('detections', [])
        if not detections:
            return

        # Normalise label
        for d in detections:
            if 'label' not in d:
                d['label'] = d.get('class_name', 'unknown')

        with self._rgb_lock:
            img = self._latest_rgb.copy() if self._latest_rgb is not None else None

        if img is None:
            return

        self._processing = True
        self._last_enrichment = now
        t = threading.Thread(target=self._enrich, args=(img, detections), daemon=True)
        t.start()

    def _enrich(self, img: np.ndarray, detections: list) -> None:
        try:
            rgb = img[:, :, ::-1].copy() if img.shape[2] == 3 else img

            has_person = any(d['label'] == 'person' for d in detections)
            enriched = {
                'source': 'live_enrichment',
                'timestamp': time.time(),
                'detections': detections,
                'pose': [],
                'segmentation': [],
                'faces': [],
            }

            # ── pose on persons ───────────────────────────────────────────────
            if has_person:
                try:
                    r = _get_model('pose')(rgb, verbose=False)[0]
                    if r.keypoints is not None and r.boxes is not None:
                        for i in range(len(r.boxes)):
                            enriched['pose'].append({
                                'bbox':      r.boxes.xyxy[i].cpu().numpy().tolist(),
                                'conf':      float(r.boxes.conf[i]),
                                'keypoints': r.keypoints.data[i].cpu().numpy().tolist(),
                            })
                    self.get_logger().debug(f'Pose: {len(enriched["pose"])} persons')
                except Exception as e:
                    self.get_logger().warn(f'Pose enrichment failed: {e}', throttle_duration_sec=10.0)

            # ── segmentation on all detections ────────────────────────────────
            try:
                r = _get_model('seg')(rgb, verbose=False)[0]
                if r.masks is not None and r.boxes is not None:
                    for i in range(len(r.boxes)):
                        conf  = float(r.boxes.conf[i])
                        if conf < 0.4:
                            continue
                        label = r.names[int(r.boxes.cls[i])]
                        mask  = r.masks.data[i].cpu().numpy().astype(bool)
                        enriched['segmentation'].append({
                            'label': label,
                            'conf':  conf,
                            'bbox':  r.boxes.xyxy[i].cpu().numpy().tolist(),
                            'area':  int(mask.sum()),
                        })
                self.get_logger().debug(f'Seg: {len(enriched["segmentation"])} objects')
            except Exception as e:
                self.get_logger().warn(f'Seg enrichment failed: {e}', throttle_duration_sec=10.0)

            # ── InsightFace (face detect + recognize + emotion) ───────────────
            if has_person:
                try:
                    worker = _get_insightface()
                    result = worker.process(rgb, {'mode': 'analyze'})
                    enriched['faces'] = result.get('faces', [])
                    self.get_logger().info(f'InsightFace: {len(enriched["faces"])} faces')
                except ImportError:
                    pass  # Not available in this env
                except Exception as e:
                    self.get_logger().warn(f'InsightFace failed: {e}', throttle_duration_sec=30.0)

            out = String()
            out.data = json.dumps(enriched)
            self._pub.publish(out)

            self.get_logger().info(
                f'Enriched: pose={len(enriched["pose"])} '
                f'seg={len(enriched["segmentation"])} '
                f'faces={len(enriched["faces"])}'
            )

        except Exception as e:
            self.get_logger().error(f'Enrichment error: {e}')
        finally:
            self._processing = False


def main(args=None):
    rclpy.init(args=args)
    node = LiveEnrichmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
