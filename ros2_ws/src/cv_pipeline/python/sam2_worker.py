#!/usr/bin/env python3
"""
SAM2 Worker - Processes images with Segment Anything Model 2
Optimized for 4GB GPUs with tiny/small models
"""

import argparse
import json
import sys
import time
import os
from pathlib import Path

# Add SAM2 to path
sam2_path = "/home/aryan/Documents/GitHub/HowYouSeeMe/sam2"
if sam2_path not in sys.path:
    sys.path.insert(0, sam2_path)

# Try to import SAM2
try:
    import torch
    import numpy as np
    from PIL import Image
    from sam2.build_sam import build_sam2
    from sam2.sam2_image_predictor import SAM2ImagePredictor
    SAM2_AVAILABLE = True
except ImportError as e:
    print(f"Warning: SAM2 not fully available: {e}", file=sys.stderr)
    SAM2_AVAILABLE = False

class SAM2Worker:
    def __init__(self):
        self.predictor = None
        self.device = "cuda" if SAM2_AVAILABLE and torch.cuda.is_available() else "cpu"
        
    def load_model(self, model_size="tiny"):
        """Load SAM2 model
        
        Args:
            model_size: "tiny" (38.9M, best for 4GB), "small" (46M), "base_plus" (80.8M), or "large" (224.4M)
        """
        if not SAM2_AVAILABLE:
            print("SAM2 not available - running in mock mode", file=sys.stderr)
            return False
            
        try:
            # Clear CUDA cache before loading
            if self.device == "cuda":
                torch.cuda.empty_cache()
                print(f"GPU Memory before loading: {torch.cuda.memory_allocated()/1024**3:.2f} GB", file=sys.stderr)
            
            print(f"Loading SAM2.1 {model_size} model on {self.device}...", file=sys.stderr)
            
            # Model configurations
            model_configs = {
                "tiny": ("sam2.1_hiera_t.yaml", "sam2.1_hiera_tiny.pt"),
                "small": ("sam2.1_hiera_s.yaml", "sam2.1_hiera_small.pt"),
                "base_plus": ("sam2.1_hiera_b+.yaml", "sam2.1_hiera_base_plus.pt"),
                "large": ("sam2.1_hiera_l.yaml", "sam2.1_hiera_large.pt"),
            }
            
            if model_size not in model_configs:
                model_size = "tiny"  # Default to tiny for safety
            
            # Use HuggingFace to load model (simplest approach)
            print(f"Loading from HuggingFace: facebook/sam2-hiera-{model_size}", file=sys.stderr)
            self.predictor = SAM2ImagePredictor.from_pretrained(f"facebook/sam2-hiera-{model_size}")
            
            if self.device == "cuda":
                print(f"GPU Memory after loading: {torch.cuda.memory_allocated()/1024**3:.2f} GB", file=sys.stderr)
            
            print(f"SAM2.1 {model_size} model loaded successfully", file=sys.stderr)
            return True
            
        except RuntimeError as e:
            if "out of memory" in str(e).lower() and self.device == "cuda":
                print(f"❌ CUDA out of memory: {e}", file=sys.stderr)
                print("💡 Try using --model-size tiny or --cpu flag", file=sys.stderr)
                return False
            else:
                print(f"Error loading SAM2: {e}", file=sys.stderr)
                return False
        except Exception as e:
            print(f"Error loading SAM2: {e}", file=sys.stderr)
            return False
    
    def process_image(self, rgb_path, depth_path, params):
        """Process image with SAM2"""
        start_time = time.time()
        
        # Load image
        if not Path(rgb_path).exists():
            return {"error": f"RGB image not found: {rgb_path}"}
        
        image = Image.open(rgb_path)
        image_np = np.array(image)
        
        # Parse parameters
        prompt_type = params.get("prompt_type", "point")
        
        # Process with SAM2 or mock
        if self.predictor is not None:
            try:
                # Clear cache before processing
                if self.device == "cuda":
                    torch.cuda.empty_cache()
                
                # Use inference mode and autocast for efficiency
                with torch.inference_mode(), torch.autocast(self.device, dtype=torch.bfloat16):
                    self.predictor.set_image(image_np)
                    
                    if prompt_type == "point":
                        # Point prompt (center of image by default)
                        points = params.get("points", [[image_np.shape[1]//2, image_np.shape[0]//2]])
                        labels = params.get("labels", [1])
                        
                        masks, scores, logits = self.predictor.predict(
                            point_coords=np.array(points),
                            point_labels=np.array(labels),
                            multimask_output=True
                        )
                    elif prompt_type == "box":
                        # Box prompt
                        box = params.get("box", [0, 0, image_np.shape[1], image_np.shape[0]])
                        
                        masks, scores, logits = self.predictor.predict(
                            box=np.array(box),
                            multimask_output=False
                        )
                    else:
                        # Default: segment center
                        masks, scores, logits = self.predictor.predict(
                            point_coords=np.array([[image_np.shape[1]//2, image_np.shape[0]//2]]),
                            point_labels=np.array([1]),
                            multimask_output=True
                        )
                
                # Process results
                result = {
                    "model": "sam2",
                    "prompt_type": prompt_type,
                    "num_masks": len(masks),
                    "processing_time": time.time() - start_time,
                    "device": self.device,
                    "masks_shape": [list(mask.shape) for mask in masks],
                    "scores": scores.tolist() if hasattr(scores, 'tolist') else list(scores),
                }
                
                # Add mask statistics
                mask_stats = []
                for i, (mask, score) in enumerate(zip(masks, scores)):
                    stats = {
                        "id": i,
                        "area": int(np.sum(mask)),
                        "bbox": self.get_bbox(mask),
                        "score": float(score)
                    }
                    mask_stats.append(stats)
                
                result["mask_stats"] = mask_stats
                
                return result
                
            except Exception as e:
                return {"error": f"SAM2 processing failed: {str(e)}"}
        else:
            # Mock mode
            return self.mock_process(image_np, params, start_time)
    
    def mock_process(self, image_np, params, start_time):
        """Mock processing for testing"""
        h, w = image_np.shape[:2]
        
        return {
            "model": "sam2",
            "mode": "mock",
            "prompt_type": params.get("prompt_type", "point"),
            "num_masks": 3,
            "processing_time": time.time() - start_time,
            "device": "mock",
            "image_size": [w, h],
            "mask_stats": [
                {"id": 0, "area": w*h//4, "bbox": [10, 10, w//2, h//2], "score": 0.95},
                {"id": 1, "area": w*h//6, "bbox": [w//2, 10, w//3, h//3], "score": 0.87},
                {"id": 2, "area": w*h//8, "bbox": [10, h//2, w//4, h//4], "score": 0.72},
            ]
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

def parse_params(params_str):
    """Parse parameter string"""
    params = {}
    if not params_str:
        return params
    
    for pair in params_str.split(','):
        if '=' in pair:
            key, value = pair.split('=', 1)
            params[key.strip()] = value.strip()
    
    return params

def main():
    parser = argparse.ArgumentParser(description='SAM2 Worker')
    parser.add_argument('--rgb', required=True, help='RGB image path')
    parser.add_argument('--depth', required=True, help='Depth image path')
    parser.add_argument('--params', default='', help='Parameters as key=value,key=value')
    parser.add_argument('--model-size', default='tiny', choices=['tiny', 'small', 'base_plus', 'large'],
                       help='Model size (tiny=38.9M for 4GB GPU, small=46M, base_plus=80.8M, large=224.4M)')
    args = parser.parse_args()
    
    # Parse parameters
    params = parse_params(args.params)
    
    # Create worker
    worker = SAM2Worker()
    
    # Load model (lazy loading)
    if SAM2_AVAILABLE:
        worker.load_model(model_size=args.model_size)
    
    # Process image
    result = worker.process_image(args.rgb, args.depth, params)
    
    # Output result as JSON
    print(json.dumps(result, indent=2))
    
    # Cleanup
    if SAM2_AVAILABLE and worker.device == "cuda":
        torch.cuda.empty_cache()
        worker.predictor = None

if __name__ == "__main__":
    main()
