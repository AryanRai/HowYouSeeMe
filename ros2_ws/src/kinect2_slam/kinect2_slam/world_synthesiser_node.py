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
# tf2_ros intentionally not imported — conda libstdc++ conflict causes segfault

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
        self.current_faces = []
        self.current_seg   = []
        self.bridge = CvBridge()
        self.objects_db = {}  # object_id -> object_data
        self.people_db = {}
        
        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Latest RGB frame for Ally's get_camera_frame tool
        self._latest_rgb = None

        # Subscribers
        self.pose_sub = self.create_subscription(PoseStamped, '/orb_slam3/pose', self.pose_callback, qos_be)
        self.detection_sub = self.create_subscription(String, '/cv_pipeline/results', self.detection_callback, qos_reliable)
        self.enriched_sub  = self.create_subscription(String, '/cv_pipeline/enriched', self.enriched_callback, qos_reliable)
        self.depth_sub = self.create_subscription(Image, '/kinect2/hd/image_depth_rect', self.depth_callback, qos_be)
        self.rgb_sub = self.create_subscription(Image, '/kinect2/hd/image_color',
                                                lambda m: setattr(self, '_latest_rgb', m), qos_be)
        
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

    def enriched_callback(self, msg):
        """Cache enriched results (pose, seg, faces) from live enrichment node"""
        try:
            data = json.loads(msg.data)
            if 'error' in data:
                return
            # Merge enriched detections into current_detections with extra fields
            enriched_dets = data.get('detections', [])
            pose_data     = {i: p for i, p in enumerate(data.get('pose', []))}
            seg_data      = data.get('segmentation', [])
            faces         = data.get('faces', [])

            # Attach pose keypoints to person detections
            person_idx = 0
            for d in enriched_dets:
                if d.get('label') == 'person' and person_idx in pose_data:
                    d['keypoints'] = pose_data[person_idx]['keypoints']
                    person_idx += 1

            # Store faces separately for people_db
            self.current_faces = faces
            self.current_seg   = seg_data

            if enriched_dets:
                self.current_detections = enriched_dets

            # Propagate face names into people_db immediately
            if faces:
                best = max(faces, key=lambda f: f.get('similarity', 0.0))
                if best.get('recognized') and best.get('name'):
                    # Apply to all people currently tracked (usually 1 person at a time)
                    for person in self.people_db.values():
                        person['face_name'] = best['name']
                        person['face_similarity'] = best.get('similarity', 0.0)
                        person['age'] = best.get('age')
                        person['gender'] = best.get('gender')
                        emo = best.get('emotion')
                        if emo:
                            person['emotion'] = emo
                            person['emotion_score'] = best.get('emotion_score', 0.0)

            # Store seg areas on matching objects
            for seg in seg_data:
                label = seg.get('label')
                for obj in self.objects_db.values():
                    if obj.get('label') == label:
                        obj['seg_area_px'] = seg.get('area', 0)
                        break
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
        
        # Save latest RGB frame for Ally's get_camera_frame tool
        if self._latest_rgb is not None:
            try:
                import cv2
                rgb_cv = self.bridge.imgmsg_to_cv2(self._latest_rgb, 'bgr8')
                cv2.imwrite('/tmp/latest_frame.jpg', rgb_cv, [cv2.IMWRITE_JPEG_QUALITY, 85])
            except Exception:
                pass

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
        """Publish RViz markers for objects and people"""
        marker_array = MarkerArray()
        marker_id = 0

        # Objects — sphere + white text
        for obj_id, obj in world_state['objects'].items():
            pos = obj.get('position', [])
            if len(pos) < 3:
                continue

            # Small sphere
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.id = marker_id
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = pos[0]
            sphere.pose.position.y = pos[1]
            sphere.pose.position.z = pos[2]
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.12
            sphere.color.r = 1.0
            sphere.color.g = 0.85
            sphere.color.b = 0.2
            sphere.color.a = 0.7
            sphere.lifetime.sec = 5
            marker_array.markers.append(sphere)
            marker_id += 1

            # Text label
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = marker_id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = pos[0]
            marker.pose.position.y = pos[1]
            marker.pose.position.z = pos[2] + 0.15
            marker.scale.z = 0.12
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.text = f"{obj['label']} ({obj['confidence']:.2f})"
            marker.lifetime.sec = 5
            marker_array.markers.append(marker)
            marker_id += 1

        # People — green sphere + name label
        for person_id, person in world_state.get('people', {}).items():
            pos = person.get('position', [])
            if len(pos) < 3:
                continue

            # Sphere at person position
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.id = marker_id
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = pos[0]
            sphere.pose.position.y = pos[1]
            sphere.pose.position.z = pos[2]
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.2
            sphere.color.r = 0.0
            sphere.color.g = 1.0
            sphere.color.b = 0.3
            sphere.color.a = 0.8
            sphere.lifetime.sec = 5
            marker_array.markers.append(sphere)
            marker_id += 1

            # Name label above sphere
            name_marker = Marker()
            name_marker.header.frame_id = 'map'
            name_marker.header.stamp = self.get_clock().now().to_msg()
            name_marker.id = marker_id
            name_marker.type = Marker.TEXT_VIEW_FACING
            name_marker.action = Marker.ADD
            name_marker.pose.position.x = pos[0]
            name_marker.pose.position.y = pos[1]
            name_marker.pose.position.z = pos[2] + 0.35
            name_marker.scale.z = 0.15
            name_marker.color.r = 0.0
            name_marker.color.g = 1.0
            name_marker.color.b = 0.5
            name_marker.color.a = 1.0
            name_marker.lifetime.sec = 5

            # Use face name if available
            face_name = person.get('face_name')
            conf = person.get('confidence', 0.0)
            if face_name and face_name != 'unknown':
                sim = person.get('face_similarity', 0.0)
                name_marker.text = f"👤 {face_name} ({sim:.0%})"
            else:
                name_marker.text = f"person ({conf:.0%})"
            marker_array.markers.append(name_marker)
            marker_id += 1

            # Emotion label if available
            emotion = person.get('emotion')
            if emotion:
                emo_marker = Marker()
                emo_marker.header.frame_id = 'map'
                emo_marker.header.stamp = self.get_clock().now().to_msg()
                emo_marker.id = marker_id
                emo_marker.type = Marker.TEXT_VIEW_FACING
                emo_marker.action = Marker.ADD
                emo_marker.pose.position.x = pos[0]
                emo_marker.pose.position.y = pos[1]
                emo_marker.pose.position.z = pos[2] + 0.55
                emo_marker.scale.z = 0.12
                emo_marker.color.r = 1.0
                emo_marker.color.g = 0.8
                emo_marker.color.b = 0.0
                emo_marker.color.a = 1.0
                emo_marker.lifetime.sec = 5
                score = person.get('emotion_score', 0.0)
                emo_marker.text = f'{emotion} ({score:.0%})' if score else emotion
                marker_array.markers.append(emo_marker)
                marker_id += 1

            # Pose skeleton as LINE_LIST if keypoints available
            kpts_raw = person.get('keypoints')
            if kpts_raw:
                from geometry_msgs.msg import Point
                SKELETON = [
                    (5,6),(5,7),(7,9),(6,8),(8,10),
                    (5,11),(6,12),(11,12),
                    (11,13),(13,15),(12,14),(14,16),
                    (0,1),(0,2),(1,3),(2,4),
                ]
                kpts = np.array(kpts_raw, dtype=np.float32)
                if kpts.shape[0] >= 17:
                    skel = Marker()
                    skel.header.frame_id = 'map'
                    skel.header.stamp = self.get_clock().now().to_msg()
                    skel.id = marker_id
                    skel.type = Marker.LINE_LIST
                    skel.action = Marker.ADD
                    skel.scale.x = 0.02
                    skel.color.r = 1.0
                    skel.color.g = 0.5
                    skel.color.b = 0.0
                    skel.color.a = 0.9
                    skel.lifetime.sec = 5
                    # Project keypoints to 3D using person depth
                    depth_m = person.get('depth_m', pos[2])
                    fx, fy = self.fx, self.fy
                    cx, cy = self.cx, self.cy
                    for a, b in SKELETON:
                        if kpts[a, 2] > 0.3 and kpts[b, 2] > 0.3:
                            for idx in (a, b):
                                px = Point()
                                px.x = (kpts[idx, 0] - cx) * depth_m / fx
                                px.y = (kpts[idx, 1] - cy) * depth_m / fy
                                px.z = depth_m
                                skel.points.append(px)
                    if skel.points:
                        marker_array.markers.append(skel)
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
