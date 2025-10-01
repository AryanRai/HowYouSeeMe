"""
SLAM (Simultaneous Localization and Mapping) module
"""

from .slam_interface import BasicSLAM, Pose, MapPoint

__all__ = ["BasicSLAM", "Pose", "MapPoint"]