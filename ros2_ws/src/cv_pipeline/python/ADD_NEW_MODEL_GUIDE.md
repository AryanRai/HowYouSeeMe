# Adding New Models to CV Pipeline

This guide explains how to add new CV models to the extensible pipeline architecture.

## Architecture Overview

The CV pipeline uses a modular architecture with three main components:

1. **BaseModel** - Abstract base class that all models inherit from
2. **ModelManager** - Manages model loading, switching, and processing
3. **CVPipelineServer** - ROS2 node that handles requests and publishes results

## Adding a New Model

### Step 1: Create Model Class

Create a new class that inherits from `BaseModel` in `cv_model_manager.py`:

```python
class YourModel(BaseModel):
    """Your Model Description"""
    
    def __init__(self, device: str = "cuda"):
        super().__init__(device)
        self.model_name = "your_model"
        # Add model-specific attributes
    
    def load(self) -> bool:
        """Load your model"""
        try:
            # Load model code here
            # Example:
            # self.model = YourModelClass.from_pretrained("model_name")
            
            self.loaded = True
            print(f"✅ {self.model_name} loaded!")
            return True
        except Exception as e:
            print(f"Failed to load {self.model_name}: {e}")
            return False
    
    def get_supported_modes(self) -> List[str]:
        """Return supported modes"""
        return ["mode1", "mode2", "mode3"]
    
    def process(self, image: np.ndarray, params: Dict[str, Any]) -> Dict[str, Any]:
        """Process image with your model"""
        if not self.loaded:
            return {"error": "Model not loaded"}
        
        start_time = time.time()
        
        try:
            # Your processing code here
            mode = params.get("mode", "mode1")
            
            if mode == "mode1":
                result = self._process_mode1(image, params)
            elif mode == "mode2":
                result = self._process_mode2(image, params)
            # ... add more modes
            
            result["processing_time"] = time.time() - start_time
            result["model"] = self.model_name
            
            return result
            
        except Exception as e:
            return {
                "error": str(e),
                "processing_time": time.time() - start_time
            }
    
    def visualize(self, image: np.ndarray, result: Dict[str, Any], 
                  params: Dict[str, Any]) -> np.ndarray:
        """Create visualization of results"""
        try:
            vis_image = image.copy()
            
            # Add your visualization code here
            # Example: draw bounding boxes, masks, keypoints, etc.
            
            return vis_image
            
        except Exception as e:
            print(f"Visualization error: {e}")
            return image
```

### Step 2: Register Model

Add your model to the `ModelManager._register_models()` method:

```python
def _register_models(self):
    """Register all available models"""
    # SAM2
    if SAM2_AVAILABLE:
        self.models["sam2"] = SAM2Model(self.device)
    
    # Your new model
    if YOUR_MODEL_AVAILABLE:
        self.models["your_model"] = YourModel(self.device)
```

### Step 3: Test Your Model

```python
# Test standalone
from cv_model_manager import ModelManager
import cv2

manager = ModelManager(device="cuda")
manager.load_model("your_model")

# Load test image
image = cv2.imread("test.jpg")
image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

# Process
params = {"mode": "mode1", "param1": "value1"}
result = manager.process("your_model", image_rgb, params)

# Visualize
vis = manager.visualize("your_model", image_rgb, result, params)
cv2.imshow("Result", cv2.cvtColor(vis, cv2.COLOR_RGB2BGR))
cv2.waitKey(0)
```

### Step 4: Use via ROS2

```bash
# Single frame
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'your_model:mode=mode1,param1=value1'"

# Streaming
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'your_model:mode=mode1,stream=true,duration=10,fps=5'"

# List models
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'your_model:list_models=true'"

# Get model info
ros2 topic pub --once /cv_pipeline/model_request std_msgs/msg/String \
  "data: 'your_model:model_info=true'"
```

## Example Models to Add

### 1. Depth Estimation (Depth Anything)

```python
class DepthAnythingModel(BaseModel):
    def __init__(self, device: str = "cuda"):
        super().__init__(device)
        self.model_name = "depth_anything"
    
    def get_supported_modes(self) -> List[str]:
        return ["relative", "metric"]
    
    def process(self, image, params):
        # Depth estimation code
        depth_map = self.model.infer(image)
        return {
            "model": "depth_anything",
            "depth_map": depth_map,
            "min_depth": float(depth_map.min()),
            "max_depth": float(depth_map.max())
        }
```

### 2. Object Detection (YOLO)

```python
class YOLOModel(BaseModel):
    def __init__(self, device: str = "cuda"):
        super().__init__(device)
        self.model_name = "yolo"
    
    def get_supported_modes(self) -> List[str]:
        return ["detect", "segment", "pose"]
    
    def process(self, image, params):
        # YOLO detection
        results = self.model(image)
        return {
            "model": "yolo",
            "detections": results.boxes.data.tolist(),
            "num_objects": len(results.boxes)
        }
```

### 3. Feature Extraction (DINO)

```python
class DINOModel(BaseModel):
    def __init__(self, device: str = "cuda"):
        super().__init__(device)
        self.model_name = "dino"
    
    def get_supported_modes(self) -> List[str]:
        return ["features", "attention"]
    
    def process(self, image, params):
        # DINO feature extraction
        features = self.model.extract_features(image)
        return {
            "model": "dino",
            "feature_dim": features.shape,
            "attention_maps": attention
        }
```

## Best Practices

1. **Error Handling**: Always wrap processing in try-except blocks
2. **Memory Management**: Clear GPU cache when switching models
3. **Visualization**: Make visualizations clear and informative
4. **Documentation**: Document all modes and parameters
5. **Testing**: Test each mode thoroughly before deployment
6. **Performance**: Log processing times for optimization

## Model Manager Commands

```python
# List all models
manager.list_models()

# Get model info
manager.get_model_info("model_name")

# Load model
manager.load_model("model_name")

# Switch models
manager.switch_model("new_model")

# Unload model
manager.unload_model("model_name")

# Process image
result = manager.process("model_name", image, params)

# Visualize
vis = manager.visualize("model_name", image, result, params)
```

## Directory Structure

```
ros2_ws/src/cv_pipeline/python/
├── cv_model_manager.py          # Model manager and base classes
├── sam2_server_v2.py            # ROS2 server using model manager
├── ADD_NEW_MODEL_GUIDE.md       # This guide
└── models/                      # Optional: separate model files
    ├── sam2_model.py
    ├── depth_model.py
    ├── yolo_model.py
    └── dino_model.py
```

## Future Enhancements

- Model ensembles (combine multiple models)
- Model chaining (output of one feeds into another)
- Automatic model selection based on task
- Model performance benchmarking
- Model versioning and A/B testing
