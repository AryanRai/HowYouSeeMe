#!/usr/bin/env python3
"""
CV Model Manager - Handles model loading, activation, and mode management
Extensible architecture for adding new models to the CV pipeline
"""

import sys
import time
import numpy as np
import cv2
from abc import ABC, abstractmethod
from typing import Dict, Any, Optional, Tuple, List

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


class BaseModel(ABC):
    """Base class for all CV models"""
    
    def __init__(self, device: str = "cuda"):
        self.device = device
        self.model = None
        self.loaded = False
        self.model_name = "base"
    
    @abstractmethod
    def load(self) -> bool:
        """Load the model"""
        pass
    
    @abstractmethod
    def process(self, image: np.ndarray, params: Dict[str, Any]) -> Dict[str, Any]:
        """Process image with given parameters"""
        pass
    
    @abstractmethod
    def get_supported_modes(self) -> List[str]:
        """Return list of supported modes"""
        pass
    
    @abstractmethod
    def visualize(self, image: np.ndarray, result: Dict[str, Any], params: Dict[str, Any]) -> np.ndarray:
        """Create visualization of results"""
        pass
    
    def unload(self):
        """Unload model from memory"""
        self.model = None
        self.loaded = False
        if self.device == "cuda" and torch.cuda.is_available():
            torch.cuda.empty_cache()


class SAM2Model(BaseModel):
    """SAM2 Segmentation Model"""
    
    def __init__(self, device: str = "cuda"):
        super().__init__(device)
        self.model_name = "sam2"
        self.predictor = None
    
    def load(self) -> bool:
        """Load SAM2 model"""
        if not SAM2_AVAILABLE:
            print("SAM2 not available!")
            return False
        
        try:
            if self.device == "cuda":
                torch.cuda.empty_cache()
                mem_before = torch.cuda.memory_allocated() / 1024**3
                print(f"GPU Memory before: {mem_before:.2f} GB")
            
            print(f"Loading SAM2 tiny model on {self.device}...")
            
            # Load from HuggingFace
            self.predictor = SAM2ImagePredictor.from_pretrained("facebook/sam2-hiera-tiny")
            
            if self.device == "cuda":
                mem_after = torch.cuda.memory_allocated() / 1024**3
                print(f"GPU Memory after: {mem_after:.2f} GB")
            
            self.loaded = True
            print("✅ SAM2 model loaded and ready!")
            return True
            
        except Exception as e:
            print(f"Failed to load SAM2: {e}")
            return False
    
    def get_supported_modes(self) -> List[str]:
        """Return supported SAM2 modes"""
        return ["point", "box", "points", "everything"]
    
    def process(self, image: np.ndarray, params: Dict[str, Any]) -> Dict[str, Any]:
        """Process image with SAM2"""
        if not self.loaded:
            return {"error": "Model not loaded"}
        
        start_time = time.time()
        
        try:
            # Ensure RGB format
            if len(image.shape) == 3 and image.shape[2] == 3:
                rgb_image = image.copy()
            else:
                return {"error": "Invalid image format"}
            
            h, w = rgb_image.shape[:2]
            
            # Clear cache
            if self.device == "cuda":
                torch.cuda.empty_cache()
            
            # Process with SAM2
            with torch.inference_mode(), torch.autocast(self.device, dtype=torch.bfloat16):
                self.predictor.set_image(rgb_image)
                
                # Parse prompt type
                prompt_type = params.get("prompt_type", "point")
                
                if prompt_type == "point":
                    masks, scores, prompt_data = self._process_point(params, w, h)
                elif prompt_type == "box":
                    masks, scores, prompt_data = self._process_box(params, w, h)
                elif prompt_type == "points":
                    masks, scores, prompt_data = self._process_points(params, w, h)
                elif prompt_type == "everything":
                    masks, scores, prompt_data = self._process_everything(params, w, h)
                else:
                    masks, scores, prompt_data = self._process_point(params, w, h)
            
            # Build result
            result = {
                "model": self.model_name,
                "prompt_type": prompt_type,
                "prompt_data": prompt_data,
                "num_masks": len(masks),
                "processing_time": time.time() - start_time,
                "device": self.device,
                "image_size": [w, h],
                "scores": scores.tolist() if hasattr(scores, 'tolist') else list(scores),
                "masks": masks,  # Keep for visualization
            }
            
            # Add mask statistics
            mask_stats = []
            for i, (mask, score) in enumerate(zip(masks, scores)):
                bbox = self._get_bbox(mask)
                stats = {
                    "id": i,
                    "area": int(np.sum(mask)),
                    "bbox": bbox,
                    "score": float(score)
                }
                mask_stats.append(stats)
            
            result["mask_stats"] = mask_stats
            
            return result
            
        except Exception as e:
            return {
                "error": str(e),
                "processing_time": time.time() - start_time
            }
    
    def _process_point(self, params: Dict, w: int, h: int) -> Tuple:
        """Process single point prompt"""
        x = int(params.get("x", w//2))
        y = int(params.get("y", h//2))
        point_coords = np.array([[x, y]])
        point_labels = np.array([1])
        
        masks, scores, logits = self.predictor.predict(
            point_coords=point_coords,
            point_labels=point_labels,
            multimask_output=True
        )
        
        return masks, scores, {"point": [x, y]}
    
    def _process_box(self, params: Dict, w: int, h: int) -> Tuple:
        """Process box prompt"""
        box_str = params.get("box", f"0,0,{w},{h}")
        box = np.array([int(x) for x in box_str.split(",")])
        
        masks, scores, logits = self.predictor.predict(
            box=box,
            multimask_output=False
        )
        
        return masks, scores, {"box": box.tolist()}
    
    def _process_points(self, params: Dict, w: int, h: int) -> Tuple:
        """Process multiple points prompt"""
        points_str = params.get("points", f"{w//2},{h//2}")
        coords = [int(x) for x in points_str.split(",")]
        point_coords = np.array(coords).reshape(-1, 2)
        
        labels_str = params.get("labels", "1" * len(point_coords))
        point_labels = np.array([int(x) for x in labels_str.split(",")])
        
        masks, scores, logits = self.predictor.predict(
            point_coords=point_coords,
            point_labels=point_labels,
            multimask_output=True
        )
        
        return masks, scores, {"points": point_coords.tolist(), "labels": point_labels.tolist()}
    
    def _process_everything(self, params: Dict, w: int, h: int) -> Tuple:
        """Process everything mode (automatic segmentation)"""
        grid_size = int(params.get("grid_size", 32))
        points = []
        for i in range(grid_size):
            for j in range(grid_size):
                x = int((j + 0.5) * w / grid_size)
                y = int((i + 0.5) * h / grid_size)
                points.append([x, y])
        
        point_coords = np.array(points)
        point_labels = np.ones(len(points), dtype=np.int32)
        
        masks, scores, logits = self.predictor.predict(
            point_coords=point_coords,
            point_labels=point_labels,
            multimask_output=True
        )
        
        return masks, scores, {"mode": "everything", "grid_size": grid_size}
    
    def _get_bbox(self, mask: np.ndarray) -> List[int]:
        """Get bounding box from mask"""
        if not np.any(mask):
            return [0, 0, 0, 0]
        
        rows = np.any(mask, axis=1)
        cols = np.any(mask, axis=0)
        
        y_min, y_max = np.where(rows)[0][[0, -1]]
        x_min, x_max = np.where(cols)[0][[0, -1]]
        
        return [int(x_min), int(y_min), int(x_max - x_min), int(y_max - y_min)]
    
    def visualize(self, image: np.ndarray, result: Dict[str, Any], params: Dict[str, Any]) -> np.ndarray:
        """Create visualization with masks overlaid"""
        try:
            vis_image = image.copy()
            
            masks = result.get("masks", [])
            scores = result.get("scores", [])
            
            if len(masks) == 0:
                return vis_image
            
            # Use the best mask
            best_idx = np.argmax(scores)
            best_mask = masks[best_idx]
            best_score = scores[best_idx]
            
            # Create colored overlay
            overlay = vis_image.copy()
            color = np.array([0, 255, 0], dtype=np.uint8)  # Green
            mask_bool = best_mask.astype(bool)
            overlay[mask_bool] = (overlay[mask_bool] * 0.5 + color * 0.5).astype(np.uint8)
            
            # Blend
            vis_image = cv2.addWeighted(vis_image, 0.6, overlay, 0.4, 0)
            
            # Draw contours
            contours, _ = cv2.findContours(
                best_mask.astype(np.uint8),
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )
            cv2.drawContours(vis_image, contours, -1, (0, 255, 0), 2)
            
            # Draw prompts
            prompt_type = result.get("prompt_type", "point")
            prompt_data = result.get("prompt_data", {})
            
            if prompt_type == "point" and "point" in prompt_data:
                point = prompt_data["point"]
                cv2.circle(vis_image, tuple(point), 8, (255, 0, 0), -1)
                cv2.circle(vis_image, tuple(point), 10, (255, 255, 255), 2)
            
            elif prompt_type == "box" and "box" in prompt_data:
                box = prompt_data["box"]
                cv2.rectangle(vis_image, (box[0], box[1]), (box[2], box[3]), (255, 0, 0), 2)
            
            elif prompt_type == "points" and "points" in prompt_data:
                points = prompt_data["points"]
                labels = prompt_data.get("labels", [1] * len(points))
                for point, label in zip(points, labels):
                    color = (255, 0, 0) if label == 1 else (0, 0, 255)
                    cv2.circle(vis_image, tuple(point), 8, color, -1)
                    cv2.circle(vis_image, tuple(point), 10, (255, 255, 255), 2)
            
            # Add text
            text = f"SAM2 [{prompt_type}] Score: {best_score:.3f}"
            cv2.putText(vis_image, text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(vis_image, text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 1)
            
            return vis_image
            
        except Exception as e:
            print(f"Visualization error: {e}")
            return image


class ModelManager:
    """Manages multiple CV models and their activation"""
    
    def __init__(self, device: str = "cuda"):
        self.device = device
        self.models: Dict[str, BaseModel] = {}
        self.active_model: Optional[str] = None
        
        # Register available models
        self._register_models()
    
    def _register_models(self):
        """Register all available models"""
        # SAM2
        if SAM2_AVAILABLE:
            self.models["sam2"] = SAM2Model(self.device)
        
        # Add more models here in the future:
        # self.models["depth_anything"] = DepthAnythingModel(self.device)
        # self.models["yolo"] = YOLOModel(self.device)
        # self.models["dino"] = DINOModel(self.device)
    
    def list_models(self) -> List[str]:
        """List all registered models"""
        return list(self.models.keys())
    
    def get_model_info(self, model_name: str) -> Dict[str, Any]:
        """Get information about a model"""
        if model_name not in self.models:
            return {"error": f"Model {model_name} not found"}
        
        model = self.models[model_name]
        return {
            "name": model_name,
            "loaded": model.loaded,
            "device": model.device,
            "supported_modes": model.get_supported_modes()
        }
    
    def load_model(self, model_name: str) -> bool:
        """Load a specific model"""
        if model_name not in self.models:
            print(f"Model {model_name} not found")
            return False
        
        model = self.models[model_name]
        if model.loaded:
            print(f"Model {model_name} already loaded")
            return True
        
        success = model.load()
        if success:
            self.active_model = model_name
        
        return success
    
    def unload_model(self, model_name: str):
        """Unload a specific model"""
        if model_name in self.models:
            self.models[model_name].unload()
            if self.active_model == model_name:
                self.active_model = None
    
    def switch_model(self, model_name: str) -> bool:
        """Switch to a different model"""
        if model_name not in self.models:
            print(f"Model {model_name} not found")
            return False
        
        # Unload current model if different
        if self.active_model and self.active_model != model_name:
            self.unload_model(self.active_model)
        
        # Load new model
        return self.load_model(model_name)
    
    def process(self, model_name: str, image: np.ndarray, params: Dict[str, Any]) -> Dict[str, Any]:
        """Process image with specified model"""
        if model_name not in self.models:
            return {"error": f"Model {model_name} not found"}
        
        model = self.models[model_name]
        
        if not model.loaded:
            print(f"Loading model {model_name}...")
            if not model.load():
                return {"error": f"Failed to load model {model_name}"}
        
        return model.process(image, params)
    
    def visualize(self, model_name: str, image: np.ndarray, result: Dict[str, Any], params: Dict[str, Any]) -> np.ndarray:
        """Create visualization for model results"""
        if model_name not in self.models:
            return image
        
        model = self.models[model_name]
        return model.visualize(image, result, params)


# Example usage
if __name__ == "__main__":
    # Create manager
    manager = ModelManager(device="cuda")
    
    # List available models
    print("Available models:", manager.list_models())
    
    # Get model info
    for model_name in manager.list_models():
        info = manager.get_model_info(model_name)
        print(f"\n{model_name}:")
        print(f"  Loaded: {info['loaded']}")
        print(f"  Device: {info['device']}")
        print(f"  Modes: {info['supported_modes']}")
    
    # Load SAM2
    if "sam2" in manager.list_models():
        print("\nLoading SAM2...")
        manager.load_model("sam2")
