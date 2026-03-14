#!/usr/bin/env python3
"""
Tier 3 — World Synthesiser Node
Produces unified world_state.json from live data + enriched checkpoints + named memories.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from cv_bridge import CvBridge
import json
import time
from pathlib import Path
import numpy as np
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

class WorldSynthesiserNode(Node):
    def __init__(self):
        super().__init__('world_synthesiser')
        
        # Parameters
        self.declare_parameter('world_state_path', '/tmp/world_state.json')
        self.declare_parameter('synthesis_interval', 3.0)
        self.declare_parameter('object_timeout', 60.0)
        self.declare_parameter('recent_events_window', 300.0)
        self.declare_parameter('fx', 1081.37)
        self.declare_parameter('fy', 1081.37)
        self.declare_parameter('cx', 959.5)
        self.declare_parameter('cy', 539.5)
        
        self.world_state_path = Path(self.get_parameter('world_state_path').value)
        self.synthesis_interval = self.get_parameter('synthesis_interval').value
        self.object_timeout = self.get_parameter('object_timeout').value
        self.recent_events_window = self.get_parameter('recent_events_window').value
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value
        
        # State
        self.current_pose = None
        self.current_detections = []
        self.current_depth = None
        self.bridge = CvBridge()
        self.objects_db = {}  # object_id -> object_data
        self.people_db = {}
        
        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Subscribers
        self.pose_sub = self.create_subscription(PoseStamped, '/orb_slam3/pose', self.pose_callback, qos_be)
        self.detection_sub = self.create_subscription(String, '/cv_pipeline/results', self.detection_callback, qos_reliable)
        self.depth_sub = self.create_subscription(Image, '/kinect2/hd/image_depth_rect', self.depth_callback, qos_be)
        
        # Publishers
        self.world_state_pub = self.create_publisher(String, '/semantic/world_state', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/semantic/markers', 10)
        
        # Timer for synthesis
        self.timer = self.create_timer(self.synthesis_interval, self.synthesize_world)
        
        self.get_logger().info(f'World synthesiser started. Output: {self.world_state_path}')
    
    def pose_callback(self, msg):
        self.current_pose = msg
    
    def detection_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if 'error' in data:
                return
            dets = data.get('detections', [])
            # Normalise: YOLO uses 'class_name', spec uses 'label'
            for d in dets:
                if 'label' not in d:
                    d['label'] = d.get('class_name', d.get('label', 'unknown'))
            self.current_detections = dets
        except json.JSONDecodeError:
            pass
    
    def depth_callback(self, msg):
        try:
            self.current_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        except Exception as e:
            self.get_logger().warn(f'Depth conversion failed: {e}')
    
    def synthesize_world(self):
        """Main synthesis loop"""
        if self.current_pose is None or self.current_depth is None:
            return
        
        current_time = time.time()
        world_state = {
            'generated_at': current_time,
            'robot': self.get_robot_state(),
            'objects': {},
            'people': {},
            'recent_events': self.load_recent_events(current_time),
            'named_memories': self.load_named_memories()
        }
        
        # Process live detections
        for det in self.current_detections:
            obj_data = self.process_detection(det, current_time)
            if obj_data:
                if det.get('label') == 'person':
                    self.people_db[obj_data['id']] = obj_data
                else:
                    self.objects_db[obj_data['id']] = obj_data
        
        # Remove stale objects
        self.objects_db = {k: v for k, v in self.objects_db.items() 
                          if current_time - v['last_seen'] < self.object_timeout}
        self.people_db = {k: v for k, v in self.people_db.items()
                         if current_time - v['last_seen'] < self.object_timeout}
        
        world_state['objects'] = self.objects_db
        world_state['people'] = self.people_db
        
        # Write to disk
        try:
            with open(self.world_state_path, 'w') as f:
                json.dump(world_state, f, indent=2)
        except Exception as e:
            self.get_logger().error(f'Failed to write world state: {e}')
        
        # Publish
        msg = String()
        msg.data = json.dumps(world_state)
        self.world_state_pub.publish(msg)
        
        self.publish_markers(world_state)
    
    def get_robot_state(self):
        """Extract robot position and orientation"""
        return {
            'position': [
                self.current_pose.pose.position.x,
                self.current_pose.pose.position.y,
                self.current_pose.pose.position.z
            ],
            'orientation': [
                self.current_pose.pose.orientation.x,
                self.current_pose.pose.orientation.y,
                self.current_pose.pose.orientation.z,
                self.current_pose.pose.orientation.w
            ],
            'room': 'unknown'
        }
    
    def process_detection(self, det, current_time):
        """Back-project detection to 3D and create object record"""
        bbox = det.get('bbox', [])
        if len(bbox) != 4:
            return None
        
        x1, y1, x2, y2 = bbox
        u = int((x1 + x2) / 2)
        v = int((y1 + y2) / 2)
        
        if v >= self.current_depth.shape[0] or u >= self.current_depth.shape[1]:
            return None
        
        Z = self.current_depth[v, u] / 1000.0
        if Z <= 0:
            return None
        
        X = (u - self.cx) * Z / self.fx
        Y = (v - self.cy) * Z / self.fy
        
        # Position in camera frame
        position = [X, Y, Z]
        
        # Generate object ID
        label = det.get('label', 'unknown')
        obj_id = f"{label}_{int(u)}_{int(v)}"
        
        # Check if object exists
        if obj_id in self.objects_db:
            obj = self.objects_db[obj_id]
            obj['last_seen'] = current_time
            obj['times_seen'] += 1
            obj['confidence'] = det.get('confidence', 0.0)
            obj['position'] = position
            return obj
        
        # New object
        return {
            'id': obj_id,
            'label': label,
            'position': position,
            'confidence': det.get('confidence', 0.0),
            'first_seen': current_time,
            'last_seen': current_time,
            'times_seen': 1,
            'depth_m': Z,
            'named_memory': None
        }
    
    def load_recent_events(self, current_time):
        """Load recent events from checkpoints"""
        events = []
        checkpoint_dir = Path('/tmp/stm')
        
        if not checkpoint_dir.exists():
            return events
        
        for cp in checkpoint_dir.glob('checkpoint_*'):
            meta_file = cp / 'meta.json'
            if not meta_file.exists():
                continue
            
            try:
                with open(meta_file) as f:
                    meta = json.load(f)
                
                if current_time - meta['timestamp'] < self.recent_events_window:
                    for event in meta.get('events', []):
                        events.append({
                            'checkpoint_id': meta['checkpoint_id'],
                            'event_type': event['type'],
                            'timestamp': meta['timestamp'],
                            'summary': f"{event['type']}: {event.get('class', '')}"
                        })
            except:
                pass
        
        return sorted(events, key=lambda e: e['timestamp'], reverse=True)[:20]
    
    def load_named_memories(self):
        """Load named memories from persistent store"""
        memory_file = Path('/tmp/named_memories.json')
        if memory_file.exists():
            try:
                with open(memory_file) as f:
                    return json.load(f)
            except:
                pass
        return {}
    
    def publish_markers(self, world_state):
        """Publish RViz markers for objects"""
        marker_array = MarkerArray()
        marker_id = 0
        
        for obj_id, obj in world_state['objects'].items():
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = marker_id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = obj['position'][0]
            marker.pose.position.y = obj['position'][1]
            marker.pose.position.z = obj['position'][2] + 0.1
            marker.scale.z = 0.1
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.text = f"{obj['label']} ({obj['confidence']:.2f})"
            marker_array.markers.append(marker)
            marker_id += 1
        
        self.markers_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = WorldSynthesiserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
