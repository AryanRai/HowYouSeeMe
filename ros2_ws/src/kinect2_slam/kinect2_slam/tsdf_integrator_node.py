#!/usr/bin/env python3
"""
Phase 3: Open3D TSDF Integrator Node
Integrates ORB-SLAM3 poses with Kinect RGB-D into dense colored point cloud
CPU-only, zero VRAM usage
"""

import rclpy
from rclpy.node import Node
import numpy as np
import open3d as o3d
import message_filters
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation


class TSDFIntegrator(Node):
    def __init__(self):
        super().__init__('tsdf_integrator')
        
        # Parameters
        self.declare_parameter('voxel_length', 0.04)
        self.declare_parameter('sdf_trunc', 0.08)
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('export_path', '/tmp/tsdf_mesh.ply')
        self.declare_parameter('fx', 1081.37)
        self.declare_parameter('fy', 1081.37)
        self.declare_parameter('cx', 959.5)
        self.declare_parameter('cy', 539.5)
        
        vl = self.get_parameter('voxel_length').value
        st = self.get_parameter('sdf_trunc').value
        
        # Initialize TSDF volume
        self.volume = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=vl,
            sdf_trunc=st,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
        )
        
        # Camera intrinsics
        fx = self.get_parameter('fx').value
        fy = self.get_parameter('fy').value
        cx = self.get_parameter('cx').value
        cy = self.get_parameter('cy').value
        
        self.intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width=1920,
            height=1080,
            fx=fx, fy=fy,
            cx=cx, cy=cy
        )
        
        self.bridge = CvBridge()
        self.frame_count = 0
        
        # Subscribers with time synchronization
        self.pose_sub = message_filters.Subscriber(
            self, PoseStamped, '/orb_slam3/pose')
        self.rgb_sub = message_filters.Subscriber(
            self, Image, '/kinect2/hd/image_color')
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/kinect2/hd/image_depth_rect')
        
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.pose_sub, self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.05
        )
        self.sync.registerCallback(self.integrate_callback)
        
        # Publisher
        self.pc_pub = self.create_publisher(
            PointCloud2, '/tsdf/pointcloud', 10)
        
        # Timer for publishing
        rate = self.get_parameter('publish_rate').value
        self.create_timer(1.0 / rate, self.publish_cloud)
        
        # Service for mesh export
        self.create_service(Trigger, '/tsdf/export_mesh', self.export_callback)
        
        self.get_logger().info(f'TSDF integrator ready (voxel={vl}, trunc={st})')
    
    def pose_to_matrix(self, pose_msg):
        """Convert ROS PoseStamped to 4x4 transformation matrix"""
        p = pose_msg.pose.position
        q = pose_msg.pose.orientation
        
        # Quaternion to rotation matrix
        R = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        
        # Build 4x4 transformation matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [p.x, p.y, p.z]
        
        return T
    
    def integrate_callback(self, pose_msg, rgb_msg, depth_msg):
        """Integrate RGB-D frame into TSDF volume"""
        try:
            # Convert ROS images to OpenCV
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'rgb8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')
            
            # ORB-SLAM3 outputs poses in OpenCV/optical convention (X right, Y down, Z forward).
            # Open3D TSDF also expects optical convention — no rotation needed.
            T = self.pose_to_matrix(pose_msg)
            
            # Convert to Open3D format
            color_o3d = o3d.geometry.Image(rgb)
            depth_o3d = o3d.geometry.Image(depth)
            
            # Create RGBD image
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color_o3d,
                depth_o3d,
                depth_scale=1000.0,  # Kinect depth is in mm
                depth_trunc=3.0,     # Max depth 3 meters
                convert_rgb_to_intensity=False
            )
            
            # Integrate into TSDF volume
            # ORB-SLAM3 gives camera-to-world, Open3D needs world-to-camera
            self.volume.integrate(rgbd, self.intrinsic, np.linalg.inv(T))
            
            self.frame_count += 1
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'Integrated {self.frame_count} frames')
                
        except Exception as e:
            self.get_logger().warn(f'Integration error: {e}')
    
    def publish_cloud(self):
        """Extract and publish point cloud from TSDF volume"""
        try:
            # Extract point cloud
            pcd = self.volume.extract_point_cloud()
            pts = np.asarray(pcd.points)
            colors = np.asarray(pcd.colors)
            
            if len(pts) == 0:
                return
            
            # Create ROS PointCloud2 message
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'map'
            
            # Create PointField definitions for RGB point cloud
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
            ]
            
            # Pack RGB into uint32
            rgb_packed = np.zeros(len(pts), dtype=np.uint32)
            rgb_packed = ((colors[:, 0] * 255).astype(np.uint32) << 16 |
                         (colors[:, 1] * 255).astype(np.uint32) << 8 |
                         (colors[:, 2] * 255).astype(np.uint32))
            
            # Combine points and colors
            cloud_data = np.zeros(len(pts), dtype=[
                ('x', np.float32),
                ('y', np.float32),
                ('z', np.float32),
                ('rgb', np.uint32)
            ])
            cloud_data['x'] = pts[:, 0]
            cloud_data['y'] = pts[:, 1]
            cloud_data['z'] = pts[:, 2]
            cloud_data['rgb'] = rgb_packed
            
            msg = point_cloud2.create_cloud(header, fields, cloud_data)
            self.pc_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().warn(f'Publish error: {e}')
    
    def export_callback(self, request, response):
        """Export TSDF mesh to PLY file"""
        try:
            path = self.get_parameter('export_path').value
            
            # Extract triangle mesh
            mesh = self.volume.extract_triangle_mesh()
            mesh.compute_vertex_normals()
            
            # Save to file
            o3d.io.write_triangle_mesh(path, mesh)
            
            response.success = True
            response.message = f'Mesh exported to {path} ({len(mesh.vertices)} vertices)'
            self.get_logger().info(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f'Export failed: {e}'
            self.get_logger().error(response.message)
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TSDFIntegrator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
