"""
Perception module for HowYouSeeMe
Handles computer vision, SLAM, and sensor processing
"""

# Core imports (always available)
from .sensor_interface import KinectV2Interface
from .slam.slam_interface import BasicSLAM, Pose, MapPoint

# Optional imports (require additional dependencies)
try:
    from .detection.object_detector import YOLODetector, Detection, ResourceManager, MultiObjectDetector
    _DETECTION_AVAILABLE = True
except ImportError:
    _DETECTION_AVAILABLE = False
    # Create dummy classes for graceful degradation
    class YOLODetector:
        def __init__(self, *args, **kwargs):
            raise ImportError("YOLODetector requires PyTorch. Install with: conda install pytorch torchvision")
    
    class Detection:
        pass
    
    class ResourceManager:
        def __init__(self, *args, **kwargs):
            raise ImportError("ResourceManager requires PyTorch. Install with: conda install pytorch torchvision")
    
    class MultiObjectDetector:
        def __init__(self, *args, **kwargs):
            raise ImportError("MultiObjectDetector requires PyTorch. Install with: conda install pytorch torchvision")

__all__ = [
    "KinectV2Interface",
    "BasicSLAM", "Pose", "MapPoint", 
    "YOLODetector", "Detection", "ResourceManager", "MultiObjectDetector"
]

# Module-level flag for checking availability
DETECTION_AVAILABLE = _DETECTION_AVAILABLE