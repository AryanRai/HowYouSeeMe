#!/usr/bin/env python3
"""
Tier 1 — Event Checkpointer Node
Watches YOLO detections and saves atomic snapshots on interesting events.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import time
from pathlib import Path

class EventCheckpointerNode(Node):
    def __init__(self):
        super().__init__('event_checkpointer')
        
        # Parameters
        self.declare_parameter('checkpoint_dir', '/tmp/stm')
        self.declare_parameter('max_checkpoints', 200)
        self.declare_parameter('person_always_save', True)
        self.declare_parameter('min_confidence', 0.85)
        self.declare_parameter('cooldown_seconds', 5.0)
        
        self.checkpoint_dir = Path(self.get_parameter('checkpoint_dir').value)
        self.max_checkpoints = self.get_parameter('max_checkpoints').value
        self.person_always_save = self.get_parameter('person_always_save').value
        self.min_confidence = self.get_parameter('min_confidence').value
        self.cooldown_seconds = self.get_parameter('cooldown_seconds').value
        
        # Create checkpoint directory
        self.checkpoint_dir.mkdir(parents=True, exist_ok=True)
        
        # State tracking
        self.seen_classes = {}  # class -> last_seen_time
        self.last_event_time = {}  # class -> last_event_time
        self.bridge = CvBridge()
        self.latest_detections = None
        self.latest_detections_time = None
        
        # QoS for best effort
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Synchronized subscribers for images + pose
        self.rgb_sub = Subscriber(self, Image, '/kinect2/hd/image_color', qos_profile=qos)
        self.depth_sub = Subscriber(self, Image, '/kinect2/hd/image_depth_rect', qos_profile=qos)
        self.pose_sub = Subscriber(self, PoseStamped, '/orb_slam3/pose', qos_profile=qos)
        
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.pose_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.sync_callback)
        
        # Separate subscriber for YOLO detections
        self.detection_sub = self.create_subscription(
            String,
            '/cv_pipeline/results',
            self.detection_callback,
            qos
        )
        
        # Publisher for checkpoint events
        self.checkpoint_pub = self.create_publisher(String, '/memory/checkpoint_saved', 10)
        
        self.get_logger().info(f'Event checkpointer started. Saving to {self.checkpoint_dir}')
    
    def detection_callback(self, msg):
        """Cache latest YOLO detections"""
        try:
            data = json.loads(msg.data)
            self.latest_detections = data.get('detections', [])
            self.latest_detections_time = time.time()
        except json.JSONDecodeError:
            pass
    
    def sync_callback(self, rgb_msg, depth_msg, pose_msg):
        """Process synchronized RGB + depth + pose"""
        if self.latest_detections is None:
            return
        
        # Check if detections are recent (within 0.2s)
        if time.time() - self.latest_detections_time > 0.2:
            return
        
        # Check for events
        events = self.check_for_events(self.latest_detections)
        
        if events:
            self.save_checkpoint(rgb_msg, depth_msg, pose_msg, self.latest_detections, events)
    
    def check_for_events(self, detections):
        """Determine if current detections warrant a checkpoint"""
        events = []
        current_time = time.time()
        current_classes = set()
        
        for det in detections:
            label = det.get('label', '')
            conf = det.get('confidence', 0.0)
            current_classes.add(label)
            
            # Event 1: Person detected (always save if enabled)
            if self.person_always_save and label == 'person':
                if self.should_trigger_event(label, current_time):
                    events.append({'type': 'person_detected', 'class': label, 'conf': conf})
            
            # Event 2: High confidence detection
            if conf > self.min_confidence:
                if self.should_trigger_event(f'high_conf_{label}', current_time):
                    events.append({'type': 'high_confidence', 'class': label, 'conf': conf})
            
            # Event 3: New class not seen in last 30 seconds
            if label not in self.seen_classes or (current_time - self.seen_classes[label]) > 30:
                if self.should_trigger_event(f'new_{label}', current_time):
                    events.append({'type': 'new_class', 'class': label, 'conf': conf})
            
            self.seen_classes[label] = current_time
        
        # Event 4: Object disappeared
        for prev_class in list(self.seen_classes.keys()):
            if prev_class not in current_classes:
                if (current_time - self.seen_classes[prev_class]) < 2.0:  # Just disappeared
                    if self.should_trigger_event(f'disappeared_{prev_class}', current_time):
                        events.append({'type': 'object_disappeared', 'class': prev_class})
        
        return events
    
    def should_trigger_event(self, event_key, current_time):
        """Check cooldown to prevent spam"""
        if event_key not in self.last_event_time:
            self.last_event_time[event_key] = current_time
            return True
        
        if (current_time - self.last_event_time[event_key]) > self.cooldown_seconds:
            self.last_event_time[event_key] = current_time
            return True
        
        return False
    
    def save_checkpoint(self, rgb_msg, depth_msg, pose_msg, detections, events):
        """Save atomic checkpoint to disk"""
        timestamp_ms = int(time.time() * 1000)
        checkpoint_id = f'checkpoint_{timestamp_ms}'
        checkpoint_path = self.checkpoint_dir / checkpoint_id
        checkpoint_path.mkdir(exist_ok=True)
        
        try:
            # Save RGB image
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            cv2.imwrite(str(checkpoint_path / 'rgb.jpg'), rgb_image, [cv2.IMWRITE_JPEG_QUALITY, 90])
            
            # Save depth image
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
            np.save(str(checkpoint_path / 'depth.npy'), depth_image)
            
            # Save pose
            pose_data = {
                'x': pose_msg.pose.position.x,
                'y': pose_msg.pose.position.y,
                'z': pose_msg.pose.position.z,
                'qx': pose_msg.pose.orientation.x,
                'qy': pose_msg.pose.orientation.y,
                'qz': pose_msg.pose.orientation.z,
                'qw': pose_msg.pose.orientation.w,
                'stamp': pose_msg.header.stamp.sec + pose_msg.header.stamp.nanosec * 1e-9
            }
            with open(checkpoint_path / 'pose.json', 'w') as f:
                json.dump(pose_data, f, indent=2)
            
            # Save detections
            with open(checkpoint_path / 'detections.json', 'w') as f:
                json.dump(detections, f, indent=2)
            
            # Save metadata
            meta_data = {
                'checkpoint_id': checkpoint_id,
                'timestamp': time.time(),
                'events': events
            }
            with open(checkpoint_path / 'meta.json', 'w') as f:
                json.dump(meta_data, f, indent=2)
            
            # Write status
            with open(checkpoint_path / 'status.txt', 'w') as f:
                f.write('pending')
            
            # Publish checkpoint event
            msg = String()
            msg.data = checkpoint_id
            self.checkpoint_pub.publish(msg)
            
            event_summary = ', '.join([f"{e['type']}:{e.get('class','')}" for e in events])
            self.get_logger().info(f'Saved checkpoint {checkpoint_id} — {event_summary}')
            
            # Evict old checkpoints
            self.evict_old_checkpoints()
            
        except Exception as e:
            self.get_logger().error(f'Failed to save checkpoint: {e}')
    
    def evict_old_checkpoints(self):
        """Remove oldest checkpoints when limit exceeded"""
        checkpoints = sorted(self.checkpoint_dir.glob('checkpoint_*'), key=lambda p: p.stat().st_mtime)
        
        if len(checkpoints) <= self.max_checkpoints:
            return
        
        # Keep person events 2x longer
        to_delete = []
        for cp in checkpoints[:len(checkpoints) - self.max_checkpoints]:
            meta_file = cp / 'meta.json'
            if meta_file.exists():
                try:
                    with open(meta_file) as f:
                        meta = json.load(f)
                    # Check if any event is person-related
                    is_person = any(e.get('type') == 'person_detected' for e in meta.get('events', []))
                    if is_person and len(checkpoints) < self.max_checkpoints * 2:
                        continue  # Keep person events longer
                except:
                    pass
            to_delete.append(cp)
        
        for cp in to_delete:
            try:
                for file in cp.iterdir():
                    file.unlink()
                cp.rmdir()
            except Exception as e:
                self.get_logger().warn(f'Failed to delete {cp}: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = EventCheckpointerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
