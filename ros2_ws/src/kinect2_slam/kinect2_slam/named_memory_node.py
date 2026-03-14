#!/usr/bin/env python3
"""
Tier 3 — Named Memory Store Node
Persistent object pinning and tracking.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import json
import time
from pathlib import Path
import numpy as np

# Service message types (using std_srvs for simplicity)
from std_srvs.srv import Trigger

class NamedMemoryNode(Node):
    def __init__(self):
        super().__init__('named_memory')
        
        self.declare_parameter('persistence_path', '/tmp/named_memories.json')
        
        self.persistence_path = Path(self.get_parameter('persistence_path').value)
        self.memories = self.load_memories()
        
        self.current_pose = None
        self.current_depth = None
        self.bridge = CvBridge()
        
        # Camera intrinsics
        self.fx = 1081.37
        self.fy = 1081.37
        self.cx = 959.5
        self.cy = 539.5
        
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Subscribers
        self.detection_sub = self.create_subscription(String, '/cv_pipeline/results', self.detection_callback, qos)
        self.pose_sub = self.create_subscription(PoseStamped, '/orb_slam3/pose', self.pose_callback, qos)
        self.depth_sub = self.create_subscription(Image, '/kinect2/hd/image_depth_rect', self.depth_callback, qos)
        
        # Publisher
        self.update_pub = self.create_publisher(String, '/memory/updated', 10)
        
        # Services (TODO: implement custom service types)
        # For now, log that services would be created
        self.get_logger().info('Named memory node started')
        self.get_logger().info(f'Persistence: {self.persistence_path}')
    
    def load_memories(self):
        """Load named memories from disk"""
        if self.persistence_path.exists():
            try:
                with open(self.persistence_path) as f:
                    return json.load(f)
            except Exception as e:
                self.get_logger().warn(f'Failed to load memories: {e}')
        return {}
    
    def save_memories(self):
        """Save named memories to disk"""
        try:
            self.persistence_path.parent.mkdir(parents=True, exist_ok=True)
            with open(self.persistence_path, 'w') as f:
                json.dump(self.memories, f, indent=2)
        except Exception as e:
            self.get_logger().error(f'Failed to save memories: {e}')
    
    def pose_callback(self, msg):
        self.current_pose = msg
    
    def depth_callback(self, msg):
        try:
            self.current_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        except:
            pass
    
    def detection_callback(self, msg):
        """Check if any named memory objects are visible"""
        if self.current_pose is None or self.current_depth is None:
            return
        
        try:
            data = json.loads(msg.data)
            detections = data.get('detections', [])
        except:
            return
        
        current_time = time.time()
        
        for name, memory in self.memories.items():
            if memory['status'] != 'watching':
                continue
            
            # Look for matching label
            for det in detections:
                if det.get('label') == memory['label'] and det.get('confidence', 0) > 0.5:
                    # Back-project to 3D
                    position = self.backproject_detection(det)
                    if position:
                        memory['position'] = position
                        memory['last_confirmed'] = current_time
                        memory['confirmed_count'] = memory.get('confirmed_count', 0) + 1
                        
                        if memory['status'] == 'watching':
                            memory['status'] = 'confirmed'
                            memory['pinned_at'] = current_time
                        
                        self.save_memories()
                        
                        # Publish update
                        update_msg = String()
                        update_msg.data = name
                        self.update_pub.publish(update_msg)
                        
                        self.get_logger().info(f'Named memory updated: {name} at {position}')
                        break
        
        # Mark stale memories
        for name, memory in self.memories.items():
            if current_time - memory.get('last_confirmed', 0) > 300:
                if memory['status'] != 'stale':
                    memory['status'] = 'stale'
                    self.save_memories()
    
    def backproject_detection(self, det):
        """Back-project detection bbox to 3D position"""
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
        
        return [float(X), float(Y), float(Z)]

def main(args=None):
    rclpy.init(args=args)
    node = NamedMemoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
