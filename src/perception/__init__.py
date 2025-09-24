"""
Perception module for HowYouSeeMe
Handles computer vision, SLAM, and sensor processing
"""

from .sensor_interface import KinectV2Interface
from .slam.slam_interface import BasicSLAM, Pose, MapPoint
from .detection.object_detector import YOLODetector, Detection, ResourceManager, MultiObjectDetector

__all__ = [
    "KinectV2Interface",
    "BasicSLAM", "Pose", "MapPoint", 
    "YOLODetector", "Detection", "ResourceManager", "MultiObjectDetector"
]