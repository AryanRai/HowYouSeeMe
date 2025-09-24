# Getting Started with HowYouSeeMe

## Phase 1: Foundation Setup (Week 1)

This guide walks you through setting up the HowYouSeeMe world perception system from scratch, starting with the essential foundation components.

### Prerequisites Checklist

**Hardware Requirements:**
- [ ] Kinect v2 with USB 3.0 cable
- [ ] USB 3.0 controller (Intel/NEC preferred, avoid ASMedia)
- [ ] NVIDIA GPU (GTX 1060+ recommended) with CUDA support
- [ ] 16GB+ RAM
- [ ] SSD storage (for fast model loading)

**Software Requirements:**
- [ ] Ubuntu 20.04+ or equivalent Linux distribution
- [ ] Python 3.8+
- [ ] NVIDIA CUDA Toolkit (11.x+)
- [ ] Git and development tools

### Step 1: Environment Setup

```bash
# Update system and install basic dependencies
sudo apt update && sudo apt upgrade -y
sudo apt install -y build-essential cmake pkg-config git curl wget
sudo apt install -y libusb-1.0-0-dev libturbojpeg0-dev libglfw3-dev
sudo apt install -y python3-dev python3-pip python3-venv

# Verify NVIDIA setup
nvidia-smi  # Should show your GPU
nvcc --version  # Should show CUDA version

# Clone the repository
git clone https://github.com/AryanRai/HowYouSeeMe.git
cd HowYouSeeMe

# Create Python virtual environment
python3 -m venv venv
source venv/bin/activate

# Install basic Python dependencies
pip install --upgrade pip
pip install numpy opencv-python torch torchvision redis fastapi uvicorn
```

### Step 2: Kinect v2 Setup

Follow the detailed [kinect.md](kinect.md) guide, or use this quickstart:

```bash
# Build libfreenect2
cd libfreenect2/build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2 \
         -DENABLE_CUDA=ON \
         -DENABLE_OPENCL=ON \
         -DENABLE_OPENGL=ON
make -j$(nproc)
make install

# Set up udev rules for device access
sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
# Unplug and replug Kinect after this step

# Test Kinect
export LD_LIBRARY_PATH=$HOME/freenect2/lib:$LD_LIBRARY_PATH
./bin/Protonect

# Test GPU acceleration (if available)
LIBFREENECT2_PIPELINE=cuda ./bin/Protonect
```

### Step 3: Project Structure Setup

```bash
# Create core project structure
cd /home/aryan/Documents/GitHub/HowYouSeeMe
mkdir -p src/{perception,summarizer,mcp_integration}
mkdir -p src/perception/{slam,detection,segmentation,face_analysis,vlm}
mkdir -p src/summarizer/{fusion,nlg,memory}
mkdir -p src/mcp_integration/{server,tools}
mkdir -p tests/{unit,integration,e2e}
mkdir -p config
mkdir -p data/{models,cache,evidence}
mkdir -p logs

# Create basic configuration structure
cat > config/config.yaml << 'EOF'
# HowYouSeeMe Configuration
system:
  log_level: INFO
  data_dir: "./data"
  cache_dir: "./data/cache"
  
perception:
  kinect:
    device_id: 0
    rgb_resolution: [1920, 1080]
    depth_resolution: [512, 424]
    fps: 30
  
  slam:
    backend: "rtabmap"  # or "orb_slam3"
    map_frame: "map"
    
  detection:
    model: "yolov8n"  # Start with nano for testing
    confidence_threshold: 0.5
    
resource_management:
  gpu_memory_limit: 6  # GB
  max_concurrent_models: 2
  model_timeout: 30  # seconds

memory:
  redis_host: "localhost"
  redis_port: 6379
  short_term_ttl: 600  # 10 minutes
  
mcp:
  host: "0.0.0.0"
  port: 8000
  enable_websockets: true
EOF
```

### Step 4: Core Module Implementation

#### 4.1 Basic Sensor Interface

```python
# src/perception/sensor_interface.py
import cv2
import numpy as np
from typing import Tuple, Optional
import threading
import queue
import time

class KinectV2Interface:
    """Basic Kinect v2 interface for RGB-D data acquisition"""
    
    def __init__(self, device_id: int = 0):
        self.device_id = device_id
        self.rgb_queue = queue.Queue(maxsize=5)
        self.depth_queue = queue.Queue(maxsize=5)
        self.running = False
        self.capture_thread = None
        
    def start(self) -> bool:
        """Start Kinect data capture"""
        try:
            # Initialize libfreenect2 (simplified)
            # In practice, use Python bindings for libfreenect2
            self.rgb_cap = cv2.VideoCapture(self.device_id)
            self.rgb_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            self.rgb_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
            
            if not self.rgb_cap.isOpened():
                return False
                
            self.running = True
            self.capture_thread = threading.Thread(target=self._capture_loop)
            self.capture_thread.start()
            return True
            
        except Exception as e:
            print(f"Failed to start Kinect: {e}")
            return False
    
    def _capture_loop(self):
        """Main capture loop"""
        while self.running:
            ret, rgb_frame = self.rgb_cap.read()
            if ret:
                timestamp = time.time()
                
                # Put frame in queue (non-blocking)
                try:
                    self.rgb_queue.put_nowait({
                        'frame': rgb_frame,
                        'timestamp': timestamp,
                        'frame_id': f"rgb_{int(timestamp * 1000)}"
                    })
                except queue.Full:
                    # Drop oldest frame
                    try:
                        self.rgb_queue.get_nowait()
                        self.rgb_queue.put_nowait({
                            'frame': rgb_frame,
                            'timestamp': timestamp,
                            'frame_id': f"rgb_{int(timestamp * 1000)}"
                        })
                    except queue.Empty:
                        pass
            
            time.sleep(1/30)  # 30 FPS
    
    def get_rgb_frame(self) -> Optional[dict]:
        """Get latest RGB frame"""
        try:
            return self.rgb_queue.get_nowait()
        except queue.Empty:
            return None
    
    def stop(self):
        """Stop capture"""
        self.running = False
        if self.capture_thread:
            self.capture_thread.join()
        if hasattr(self, 'rgb_cap'):
            self.rgb_cap.release()

# Test the interface
if __name__ == "__main__":
    kinect = KinectV2Interface()
    if kinect.start():
        print("Kinect started successfully")
        
        for i in range(100):  # Capture 100 frames
            frame_data = kinect.get_rgb_frame()
            if frame_data:
                print(f"Frame {i}: {frame_data['timestamp']}")
                # Optional: Display frame
                # cv2.imshow('RGB', frame_data['frame'])
                # cv2.waitKey(1)
            time.sleep(0.1)
        
        kinect.stop()
    else:
        print("Failed to start Kinect")
```

#### 4.2 Basic SLAM Integration

```python
# src/perception/slam/slam_interface.py
import numpy as np
from typing import Tuple, Optional
import cv2

class BasicSLAM:
    """Basic SLAM implementation using ORB features"""
    
    def __init__(self):
        self.orb = cv2.ORB_create()
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.prev_frame = None
        self.prev_kp = None
        self.prev_desc = None
        self.camera_pose = np.eye(4)  # 4x4 transformation matrix
        self.map_points = []
        
    def process_frame(self, rgb_frame: np.ndarray, depth_frame: Optional[np.ndarray] = None) -> dict:
        """Process RGB frame for SLAM"""
        gray = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2GRAY)
        
        # Extract ORB features
        kp, desc = self.orb.detectAndCompute(gray, None)
        
        if self.prev_desc is not None and desc is not None:
            # Match features
            matches = self.matcher.match(self.prev_desc, desc)
            matches = sorted(matches, key=lambda x: x.distance)
            
            if len(matches) > 10:  # Minimum matches for pose estimation
                # Extract matched points
                prev_pts = np.float32([self.prev_kp[m.queryIdx].pt for m in matches])
                curr_pts = np.float32([kp[m.trainIdx].pt for m in matches])
                
                # Estimate camera motion (simplified)
                # In practice, use proper pose estimation with depth info
                H, mask = cv2.findHomography(prev_pts, curr_pts, cv2.RANSAC)
                
                if H is not None:
                    # Update camera pose (simplified)
                    # This should be replaced with proper 3D pose estimation
                    pass
        
        # Store current frame data
        self.prev_frame = gray.copy()
        self.prev_kp = kp
        self.prev_desc = desc
        
        return {
            'pose': self.camera_pose.copy(),
            'keypoints': kp,
            'descriptors': desc,
            'num_features': len(kp),
            'timestamp': rgb_frame.shape  # Placeholder
        }

# Test SLAM
if __name__ == "__main__":
    slam = BasicSLAM()
    # Test with webcam or Kinect frames
    cap = cv2.VideoCapture(0)
    
    while True:
        ret, frame = cap.read()
        if ret:
            slam_result = slam.process_frame(frame)
            print(f"Detected {slam_result['num_features']} features")
            
            # Visualize keypoints
            img_with_kp = cv2.drawKeypoints(frame, slam_result['keypoints'], None)
            cv2.imshow('SLAM Features', img_with_kp)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    cap.release()
    cv2.destroyAllWindows()
```

#### 4.3 Basic Object Detection

```python
# src/perception/detection/object_detector.py
import torch
import torchvision.transforms as transforms
from typing import List, Dict, Tuple
import cv2
import numpy as np

class YOLODetector:
    """Basic YOLO object detector"""
    
    def __init__(self, model_name: str = "yolov5s", confidence_threshold: float = 0.5):
        self.confidence_threshold = confidence_threshold
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        try:
            # Load YOLOv5 model
            self.model = torch.hub.load('ultralytics/yolov5', model_name, pretrained=True)
            self.model.to(self.device)
            self.model.eval()
            print(f"Loaded {model_name} on {self.device}")
        except Exception as e:
            print(f"Failed to load YOLO model: {e}")
            self.model = None
    
    def detect_objects(self, rgb_frame: np.ndarray) -> List[Dict]:
        """Detect objects in RGB frame"""
        if self.model is None:
            return []
        
        try:
            # Run inference
            results = self.model(rgb_frame)
            
            # Parse results
            detections = []
            for *box, conf, cls_id in results.xyxy[0].cpu().numpy():
                if conf >= self.confidence_threshold:
                    x1, y1, x2, y2 = map(int, box)
                    class_name = self.model.names[int(cls_id)]
                    
                    detections.append({
                        'bbox': [x1, y1, x2 - x1, y2 - y1],  # [x, y, w, h]
                        'confidence': float(conf),
                        'class': class_name,
                        'class_id': int(cls_id),
                        'center': [(x1 + x2) // 2, (y1 + y2) // 2]
                    })
            
            return detections
            
        except Exception as e:
            print(f"Detection error: {e}")
            return []
    
    def visualize_detections(self, frame: np.ndarray, detections: List[Dict]) -> np.ndarray:
        """Draw detection boxes on frame"""
        vis_frame = frame.copy()
        
        for det in detections:
            x, y, w, h = det['bbox']
            conf = det['confidence']
            class_name = det['class']
            
            # Draw bounding box
            cv2.rectangle(vis_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Draw label
            label = f"{class_name}: {conf:.2f}"
            cv2.putText(vis_frame, label, (x, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        return vis_frame

# Test detector
if __name__ == "__main__":
    detector = YOLODetector()
    cap = cv2.VideoCapture(0)
    
    while True:
        ret, frame = cap.read()
        if ret:
            detections = detector.detect_objects(frame)
            vis_frame = detector.visualize_detections(frame, detections)
            
            print(f"Detected {len(detections)} objects")
            for det in detections:
                print(f"  - {det['class']}: {det['confidence']:.2f}")
            
            cv2.imshow('Object Detection', vis_frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    cap.release()
    cv2.destroyAllWindows()
```

### Step 5: Basic Integration Test

Create a simple integration test to verify all components work together:

```python
# test_integration.py
import time
import cv2
from src.perception.sensor_interface import KinectV2Interface
from src.perception.slam.slam_interface import BasicSLAM
from src.perception.detection.object_detector import YOLODetector

def main():
    """Basic integration test"""
    print("Starting HowYouSeeMe integration test...")
    
    # Initialize components
    kinect = KinectV2Interface()
    slam = BasicSLAM()
    detector = YOLODetector()
    
    # Start Kinect
    if not kinect.start():
        print("Failed to start Kinect, using webcam")
        cap = cv2.VideoCapture(0)
        use_kinect = False
    else:
        print("Kinect started successfully")
        use_kinect = True
    
    print("Running perception pipeline...")
    frame_count = 0
    
    try:
        while frame_count < 100:  # Process 100 frames
            if use_kinect:
                frame_data = kinect.get_rgb_frame()
                if frame_data is None:
                    continue
                frame = frame_data['frame']
            else:
                ret, frame = cap.read()
                if not ret:
                    continue
            
            # Process frame through pipeline
            slam_result = slam.process_frame(frame)
            detections = detector.detect_objects(frame)
            
            # Log results
            frame_count += 1
            if frame_count % 10 == 0:
                print(f"Frame {frame_count}:")
                print(f"  SLAM features: {slam_result['num_features']}")
                print(f"  Detections: {len(detections)}")
                for det in detections[:3]:  # Show first 3
                    print(f"    - {det['class']}: {det['confidence']:.2f}")
            
            # Optional: Visualize
            vis_frame = detector.visualize_detections(frame, detections)
            cv2.imshow('HowYouSeeMe Test', vis_frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            time.sleep(0.1)
    
    finally:
        # Cleanup
        if use_kinect:
            kinect.stop()
        else:
            cap.release()
        cv2.destroyAllWindows()
    
    print("Integration test completed!")

if __name__ == "__main__":
    main()
```

### Step 6: Run the Test

```bash
# Activate environment and run integration test
cd /home/aryan/Documents/GitHub/HowYouSeeMe
source venv/bin/activate
python test_integration.py
```

### Next Steps (Week 2)

After completing this foundation setup:

1. **Enhance SLAM**: Integrate proper SLAM solution (RTAB-Map or ORB-SLAM3)
2. **Add Resource Management**: Implement the ResourceManager class
3. **World State Storage**: Set up Redis for entity storage
4. **Human Detection**: Add person detector and face analysis
5. **Memory System**: Begin implementing persistent memory

### Troubleshooting

**Common Issues:**
- **Kinect not detected**: Check USB 3.0 connection and udev rules
- **CUDA errors**: Verify NVIDIA driver and CUDA toolkit installation
- **Model loading fails**: Check internet connection for downloading pretrained models
- **Performance issues**: Start with smaller models (YOLOv5n instead of YOLOv5s)

**Debug Commands:**
```bash
# Check Kinect
lsusb | grep Microsoft
dmesg | grep -i kinect

# Check CUDA
nvidia-smi
python -c "import torch; print(torch.cuda.is_available())"

# Check Python environment
pip list | grep torch
python -c "import cv2; print(cv2.__version__)"
```

This foundation provides the essential building blocks for the HowYouSeeMe system. Each component can be enhanced and optimized as you progress through the implementation phases.