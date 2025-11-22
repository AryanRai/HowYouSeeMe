#!/usr/bin/env python3
"""
SAM2 Server V2 - Uses ModelManager for extensible architecture
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import json
import time
import sys

# Add local modules
sys.path.insert(0, '/home/aryan/Documents/GitHub/HowYouSeeMe/ros2_ws/src/cv_pipeline/python')
from cv_model_manager import ModelManager


class CVPipelineServer(Node):
    def __init__(self):
        super().__init__('cv_pipeline_server')
        
        self.bridge = CvBridge()
        self.model_manager = ModelManager(device="cuda")
        
        # Latest images
        self.latest_rgb = None
        self.latest_depth = None
        
        # Streaming mode
        self.streaming = False
        self.stream_timer = None
        self.stream_end_time = None
        self.stream_params = {}
        self.stream_model = "sam2"
        
        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image,
            '/kinect2/qhd/image_color',
            self.rgb_callback,
            10)
        
        self.depth_sub = self.create_subscription(
            Image,
            '/kinect2/qhd/image_depth',
            self.depth_callback,
            10)
        
        self.request_sub = self.create_subscription(
            String,
            '/cv_pipeline/model_request',
            self.request_callback,
            10)
        
        # Publishers
        self.result_pub = self.create_publisher(String, '/cv_pipeline/results', 10)
        self.viz_pub = self.create_publisher(Image, '/cv_pipeline/visualization', 10)
        
        # Load default model
        self.get_logger().info('CV Pipeline Server starting...')
        self.get_logger().info(f'Available models: {self.model_manager.list_models()}')
        
        # Load SAM2 by default
        if "sam2" in self.model_manager.list_models():
            self.model_manager.load_model("sam2")
        
        self.get_logger().info('CV Pipeline Server ready!')
    
    def rgb_callback(self, msg):
        """Store latest RGB image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_rgb = cv_image
        except Exception as e:
            self.get_logger().error(f'RGB callback error: {e}')
    
    def depth_callback(self, msg):
        """Store latest depth image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            self.latest_depth = cv_image
        except Exception as e:
            self.get_logger().error(f'Depth callback error: {e}')
    
    def request_callback(self, msg):
        """Process model request"""
        self.get_logger().info(f'Request received: {msg.data}')
        
        # Check if we have images
        if self.latest_rgb is None:
            self.get_logger().warn('No RGB image available yet')
            return
        
        # Parse request
        try:
            parts = msg.data.split(':')
            model_name = parts[0]
            params = {}
            if len(parts) > 1:
                for param in parts[1].split(','):
                    if '=' in param:
                        k, v = param.split('=')
                        params[k.strip()] = v.strip()
            
            # Handle special commands
            if 'list_models' in params:
                self.list_models()
                return
            
            if 'model_info' in params:
                self.get_model_info(model_name)
                return
            
            # Check for streaming mode
            if 'stream' in params:
                duration = float(params.get('duration', 10.0))
                fps = float(params.get('fps', 5.0))
                self.start_streaming(model_name, params, duration, fps)
                return
            
            # Check for stop streaming
            if params.get('stop') == 'true':
                self.stop_streaming()
                return
            
            # Process single frame
            result = self.process_frame(model_name, params)
            
            # Publish result
            result_msg = String()
            result_msg.data = json.dumps(result)
            self.result_pub.publish(result_msg)
            
            self.get_logger().info(f'✅ Processing complete: {result.get("processing_time", 0):.3f}s')
            
        except Exception as e:
            self.get_logger().error(f'Request processing error: {e}')
    
    def process_frame(self, model_name: str, params: dict) -> dict:
        """Process a single frame with specified model"""
        # Convert BGR to RGB
        rgb_image = self.latest_rgb[:, :, ::-1].copy()
        
        # Process with model
        result = self.model_manager.process(model_name, rgb_image, params)
        
        # Create and publish visualization
        if "error" not in result:
            vis_image = self.model_manager.visualize(model_name, rgb_image, result, params)
            
            # Convert back to BGR for ROS
            vis_image_bgr = vis_image[:, :, ::-1].copy()
            
            # Publish visualization
            viz_msg = self.bridge.cv2_to_imgmsg(vis_image_bgr, encoding='bgr8')
            viz_msg.header.stamp = self.get_clock().now().to_msg()
            viz_msg.header.frame_id = 'kinect2_rgb_optical_frame'
            self.viz_pub.publish(viz_msg)
        
        # Remove masks from result (too large for JSON)
        if "masks" in result:
            del result["masks"]
        
        return result
    
    def list_models(self):
        """List available models"""
        models = self.model_manager.list_models()
        result = {
            "command": "list_models",
            "models": models
        }
        
        result_msg = String()
        result_msg.data = json.dumps(result)
        self.result_pub.publish(result_msg)
        
        self.get_logger().info(f'Available models: {models}')
    
    def get_model_info(self, model_name: str):
        """Get model information"""
        info = self.model_manager.get_model_info(model_name)
        
        result_msg = String()
        result_msg.data = json.dumps(info)
        self.result_pub.publish(result_msg)
        
        self.get_logger().info(f'Model info: {info}')
    
    def start_streaming(self, model_name: str, params: dict, duration: float, fps: float):
        """Start streaming mode"""
        if self.streaming:
            self.get_logger().warn('Already streaming! Stop current stream first.')
            return
        
        self.streaming = True
        self.stream_model = model_name
        self.stream_params = params
        self.stream_end_time = time.time() + duration
        
        # Create timer
        interval = 1.0 / fps
        self.stream_timer = self.create_timer(interval, self.stream_callback)
        
        self.get_logger().info(f'🎬 Started streaming: {model_name} for {duration}s @ {fps} FPS')
        
        # Publish status
        status = {
            "status": "streaming_started",
            "model": model_name,
            "duration": duration,
            "fps": fps,
            "end_time": self.stream_end_time
        }
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.result_pub.publish(status_msg)
    
    def stop_streaming(self):
        """Stop streaming mode"""
        if not self.streaming:
            self.get_logger().warn('Not currently streaming')
            return
        
        self.streaming = False
        if self.stream_timer:
            self.stream_timer.cancel()
            self.stream_timer = None
        
        self.get_logger().info('⏹️  Streaming stopped')
        
        # Publish status
        status = {"status": "streaming_stopped"}
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.result_pub.publish(status_msg)
    
    def stream_callback(self):
        """Process one frame in streaming mode"""
        # Check if duration expired
        if time.time() >= self.stream_end_time:
            self.get_logger().info('⏱️  Stream duration completed')
            self.stop_streaming()
            return
        
        # Check if we have images
        if self.latest_rgb is None:
            return
        
        # Process frame
        try:
            result = self.process_frame(self.stream_model, self.stream_params)
            
            # Add streaming info
            result["streaming"] = True
            result["time_remaining"] = self.stream_end_time - time.time()
            
            # Publish result
            result_msg = String()
            result_msg.data = json.dumps(result)
            self.result_pub.publish(result_msg)
            
        except Exception as e:
            self.get_logger().error(f'Stream processing error: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = CVPipelineServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
