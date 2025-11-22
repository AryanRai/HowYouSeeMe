#!/usr/bin/env python3
"""
SAM2 Server - Keeps model loaded and processes requests via ROS2
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import json
import sys
import time
import numpy as np
import cv2

# Add SAM2 to path
sam2_path = "/home/aryan/Documents/GitHub/HowYouSeeMe/sam2"
if sam2_path not in sys.path:
    sys.path.insert(0, sam2_path)

try:
    import torch
    from sam2.sam2_image_predictor import SAM2ImagePredictor
    SAM2_AVAILABLE = True
except ImportError as e:
    print(f"SAM2 not available: {e}")
    SAM2_AVAILABLE = False

class SAM2Server(Node):
    def __init__(self):
        super().__init__('sam2_server')
        
        self.bridge = CvBridge()
        self.predictor = None
        self.device = "cuda" if SAM2_AVAILABLE and torch.cuda.is_available() else "cpu"
        
        # Latest images
        self.latest_rgb = None
        self.latest_depth = None
        
        # Streaming mode
        self.streaming = False
        self.stream_timer = None
        self.stream_end_time = None
        self.stream_params = {}
        
        # Subscribers - NO synchronization, just take latest
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
        
        # Load model at startup
        self.get_logger().info('SAM2 Server starting...')
        self.load_model()
        
        self.get_logger().info('SAM2 Server ready!')
    
    def load_model(self):
        """Load SAM2 model at startup"""
        if not SAM2_AVAILABLE:
            self.get_logger().error('SAM2 not available!')
            return
        
        try:
            if self.device == "cuda":
                torch.cuda.empty_cache()
                mem_before = torch.cuda.memory_allocated() / 1024**3
                self.get_logger().info(f'GPU Memory before: {mem_before:.2f} GB')
            
            self.get_logger().info(f'Loading SAM2 tiny model on {self.device}...')
            
            # Load from HuggingFace
            self.predictor = SAM2ImagePredictor.from_pretrained("facebook/sam2-hiera-tiny")
            
            if self.device == "cuda":
                mem_after = torch.cuda.memory_allocated() / 1024**3
                self.get_logger().info(f'GPU Memory after: {mem_after:.2f} GB')
            
            self.get_logger().info('✅ SAM2 model loaded and ready!')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load SAM2: {e}')
    
    def rgb_callback(self, msg):
        """Store latest RGB image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_rgb = cv_image
            self.get_logger().debug('RGB image received')
        except Exception as e:
            self.get_logger().error(f'RGB callback error: {e}')
    
    def depth_callback(self, msg):
        """Store latest depth image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            self.latest_depth = cv_image
            self.get_logger().debug('Depth image received')
        except Exception as e:
            self.get_logger().error(f'Depth callback error: {e}')
    
    def request_callback(self, msg):
        """Process segmentation request"""
        self.get_logger().info(f'Request received: {msg.data}')
        
        # Check if we have images
        if self.latest_rgb is None:
            self.get_logger().warn('No RGB image available yet')
            return
        
        if self.predictor is None:
            self.get_logger().error('SAM2 model not loaded')
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
            
            if model_name != 'sam2':
                self.get_logger().warn(f'Unknown model: {model_name}')
                return
            
            # Check for streaming mode
            if 'stream' in params:
                duration = float(params.get('duration', 10.0))  # Default 10 seconds
                fps = float(params.get('fps', 5.0))  # Default 5 FPS
                self.start_streaming(params, duration, fps)
                return
            
            # Check for stop streaming
            if params.get('stop') == 'true':
                self.stop_streaming()
                return
            
            # Process single frame with SAM2
            result = self.process_sam2(params)
            
            # Publish result
            result_msg = String()
            result_msg.data = json.dumps(result)
            self.result_pub.publish(result_msg)
            
            self.get_logger().info(f'✅ Processing complete: {result["processing_time"]:.3f}s')
            
        except Exception as e:
            self.get_logger().error(f'Request processing error: {e}')
    
    def process_sam2(self, params):
        """Process image with SAM2"""
        start_time = time.time()
        
        try:
            # Convert BGR to RGB and ensure contiguous array
            rgb_image = self.latest_rgb[:, :, ::-1].copy()
            
            # Clear cache
            if self.device == "cuda":
                torch.cuda.empty_cache()
            
            # Process with SAM2
            with torch.inference_mode(), torch.autocast(self.device, dtype=torch.bfloat16):
                self.predictor.set_image(rgb_image)
                
                # Default: point at center
                h, w = rgb_image.shape[:2]
                point_coords = np.array([[w//2, h//2]])
                point_labels = np.array([1])
                
                masks, scores, logits = self.predictor.predict(
                    point_coords=point_coords,
                    point_labels=point_labels,
                    multimask_output=True
                )
            
            # Build result
            result = {
                "model": "sam2",
                "prompt_type": params.get("prompt_type", "point"),
                "num_masks": len(masks),
                "processing_time": time.time() - start_time,
                "device": self.device,
                "image_size": [w, h],
                "scores": scores.tolist() if hasattr(scores, 'tolist') else list(scores),
            }
            
            # Add mask statistics
            mask_stats = []
            for i, (mask, score) in enumerate(zip(masks, scores)):
                bbox = self.get_bbox(mask)
                stats = {
                    "id": i,
                    "area": int(np.sum(mask)),
                    "bbox": bbox,
                    "score": float(score)
                }
                mask_stats.append(stats)
            
            result["mask_stats"] = mask_stats
            
            # Create and publish visualization
            self.publish_visualization(rgb_image, masks, scores, point_coords)
            
            return result
            
        except Exception as e:
            self.get_logger().error(f'SAM2 processing error: {e}')
            return {
                "error": str(e),
                "processing_time": time.time() - start_time
            }
    
    def get_bbox(self, mask):
        """Get bounding box from mask"""
        if not np.any(mask):
            return [0, 0, 0, 0]
        
        rows = np.any(mask, axis=1)
        cols = np.any(mask, axis=0)
        
        y_min, y_max = np.where(rows)[0][[0, -1]]
        x_min, x_max = np.where(cols)[0][[0, -1]]
        
        return [int(x_min), int(y_min), int(x_max - x_min), int(y_max - y_min)]
    
    def publish_visualization(self, rgb_image, masks, scores, point_coords):
        """Create and publish visualization with masks overlaid"""
        try:
            # Convert RGB back to BGR for OpenCV
            vis_image = rgb_image[:, :, ::-1].copy()
            
            # Use the best mask (highest score)
            best_idx = np.argmax(scores)
            best_mask = masks[best_idx]
            best_score = scores[best_idx]
            
            # Create colored overlay for the mask
            overlay = vis_image.copy()
            color = np.array([0, 255, 0], dtype=np.uint8)  # Green (BGR)
            mask_bool = best_mask.astype(bool)
            overlay[mask_bool] = (overlay[mask_bool] * 0.5 + color * 0.5).astype(np.uint8)
            
            # Blend with original
            vis_image = cv2.addWeighted(vis_image, 0.6, overlay, 0.4, 0)
            
            # Draw mask contours
            contours, _ = cv2.findContours(
                best_mask.astype(np.uint8), 
                cv2.RETR_EXTERNAL, 
                cv2.CHAIN_APPROX_SIMPLE
            )
            cv2.drawContours(vis_image, contours, -1, (0, 255, 0), 2)
            
            # Draw the prompt point
            if point_coords is not None and len(point_coords) > 0:
                for point in point_coords:
                    cv2.circle(vis_image, tuple(point.astype(int)), 8, (255, 0, 0), -1)
                    cv2.circle(vis_image, tuple(point.astype(int)), 10, (255, 255, 255), 2)
            
            # Add text with score
            text = f"SAM2 Score: {best_score:.3f}"
            cv2.putText(vis_image, text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(vis_image, text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)
            
            # Publish visualization
            viz_msg = self.bridge.cv2_to_imgmsg(vis_image, encoding='bgr8')
            viz_msg.header.stamp = self.get_clock().now().to_msg()
            viz_msg.header.frame_id = 'kinect2_rgb_optical_frame'
            self.viz_pub.publish(viz_msg)
            
            self.get_logger().debug('Visualization published')
            
        except Exception as e:
            self.get_logger().error(f'Visualization error: {e}')
    
    def start_streaming(self, params, duration, fps):
        """Start streaming segmentation for a fixed duration"""
        if self.streaming:
            self.get_logger().warn('Already streaming! Stop current stream first.')
            return
        
        self.streaming = True
        self.stream_params = params
        self.stream_end_time = time.time() + duration
        
        # Create timer for periodic processing
        interval = 1.0 / fps
        self.stream_timer = self.create_timer(interval, self.stream_callback)
        
        self.get_logger().info(f'🎬 Started streaming: {duration}s @ {fps} FPS')
        
        # Publish status
        status = {
            "status": "streaming_started",
            "duration": duration,
            "fps": fps,
            "end_time": self.stream_end_time
        }
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.result_pub.publish(status_msg)
    
    def stop_streaming(self):
        """Stop streaming segmentation"""
        if not self.streaming:
            self.get_logger().warn('Not currently streaming')
            return
        
        self.streaming = False
        if self.stream_timer:
            self.stream_timer.cancel()
            self.stream_timer = None
        
        self.get_logger().info('⏹️  Streaming stopped')
        
        # Publish status
        status = {
            "status": "streaming_stopped"
        }
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
            self.get_logger().debug('No RGB image available for streaming')
            return
        
        # Process frame
        try:
            result = self.process_sam2(self.stream_params)
            
            # Add streaming info
            result["streaming"] = True
            result["time_remaining"] = self.stream_end_time - time.time()
            
            # Publish result
            result_msg = String()
            result_msg.data = json.dumps(result)
            self.result_pub.publish(result_msg)
            
            self.get_logger().debug(f'Stream frame: {result["processing_time"]:.3f}s')
            
        except Exception as e:
            self.get_logger().error(f'Stream processing error: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    node = SAM2Server()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
