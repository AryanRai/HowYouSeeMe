"""
SLAM Interface for HowYouSeeMe
Provides simultaneous localization and mapping using ORB features
"""

import numpy as np
from typing import Tuple, Optional, Dict, List, Any
import cv2
import logging
import time
from dataclasses import dataclass

logger = logging.getLogger(__name__)

@dataclass
class Pose:
    """Camera pose representation"""
    position: np.ndarray  # [x, y, z]
    orientation: np.ndarray  # Rotation matrix 3x3
    timestamp: float
    confidence: float = 1.0
    
    def to_matrix(self) -> np.ndarray:
        """Convert to 4x4 transformation matrix"""
        T = np.eye(4)
        T[:3, :3] = self.orientation
        T[:3, 3] = self.position
        return T

@dataclass
class MapPoint:
    """3D map point"""
    id: int
    position: np.ndarray  # [x, y, z]
    descriptor: np.ndarray
    observations: int = 0
    last_seen: float = 0.0

class BasicSLAM:
    """Basic SLAM implementation using ORB features and essential matrix"""
    
    def __init__(self, camera_intrinsics: Dict[str, float]):
        # ORB feature detector
        self.orb = cv2.ORB_create(nfeatures=1000)
        
        # Feature matcher
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
        # Camera parameters
        self.K = np.array([
            [camera_intrinsics['fx'], 0, camera_intrinsics['cx']],
            [0, camera_intrinsics['fy'], camera_intrinsics['cy']],
            [0, 0, 1]
        ])
        
        # SLAM state
        self.current_pose = Pose(
            position=np.array([0.0, 0.0, 0.0]),
            orientation=np.eye(3),
            timestamp=time.time()
        )
        
        self.poses = [self.current_pose]
        self.map_points = []
        self.next_point_id = 0
        
        # Previous frame data
        self.prev_frame = None
        self.prev_keypoints = None
        self.prev_descriptors = None
        self.prev_timestamp = None
        
        # Tracking state
        self.is_initialized = False
        self.lost_tracking = False
        self.min_matches = 20
        
    def process_frame(self, rgb_frame: np.ndarray, depth_frame: Optional[np.ndarray] = None, 
                     timestamp: Optional[float] = None) -> Dict[str, Any]:
        """Process RGB(-D) frame for SLAM"""
        if timestamp is None:
            timestamp = time.time()
        
        # Convert to grayscale
        if len(rgb_frame.shape) == 3:
            gray = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = rgb_frame
        
        # Extract ORB features
        keypoints, descriptors = self.orb.detectAndCompute(gray, None)
        
        if descriptors is None:
            logger.warning("No features detected in frame")
            return self._create_result(keypoints, [], timestamp)
        
        # Initialize or track
        if not self.is_initialized:
            result = self._initialize_slam(gray, keypoints, descriptors, timestamp)
        else:
            result = self._track_frame(gray, keypoints, descriptors, depth_frame, timestamp)
        
        # Store current frame data
        self.prev_frame = gray.copy()
        self.prev_keypoints = keypoints
        self.prev_descriptors = descriptors
        self.prev_timestamp = timestamp
        
        return result
    
    def _initialize_slam(self, gray: np.ndarray, keypoints: List, descriptors: np.ndarray, 
                        timestamp: float) -> Dict[str, Any]:
        """Initialize SLAM with first frame"""
        logger.info("Initializing SLAM with first frame")
        
        # Set initial pose
        self.current_pose = Pose(
            position=np.array([0.0, 0.0, 0.0]),
            orientation=np.eye(3),
            timestamp=timestamp
        )
        
        self.is_initialized = True
        return self._create_result(keypoints, [], timestamp)
    
    def _track_frame(self, gray: np.ndarray, keypoints: List, descriptors: np.ndarray,
                    depth_frame: Optional[np.ndarray], timestamp: float) -> Dict[str, Any]:
        """Track camera motion between frames"""
        
        if self.prev_descriptors is None:
            return self._create_result(keypoints, [], timestamp)
        
        # Match features between frames
        matches = self.matcher.match(self.prev_descriptors, descriptors)
        matches = sorted(matches, key=lambda x: x.distance)
        
        if len(matches) < self.min_matches:
            logger.warning(f"Insufficient matches: {len(matches)} < {self.min_matches}")
            self.lost_tracking = True
            return self._create_result(keypoints, matches, timestamp)
        
        # Extract matched points
        prev_pts = np.float32([self.prev_keypoints[m.queryIdx].pt for m in matches])
        curr_pts = np.float32([keypoints[m.trainIdx].pt for m in matches])
        
        # Estimate camera motion
        pose_estimated = False
        
        if depth_frame is not None:
            # Use PnP with depth information (more accurate)
            pose_estimated = self._estimate_pose_pnp(prev_pts, curr_pts, depth_frame, timestamp)
        else:
            # Use essential matrix (monocular)
            pose_estimated = self._estimate_pose_essential(prev_pts, curr_pts, timestamp)
        
        if not pose_estimated:
            logger.warning("Pose estimation failed")
            self.lost_tracking = True
        else:
            self.lost_tracking = False
        
        return self._create_result(keypoints, matches, timestamp)
    
    def _estimate_pose_pnp(self, prev_pts: np.ndarray, curr_pts: np.ndarray,
                          depth_frame: np.ndarray, timestamp: float) -> bool:
        """Estimate pose using PnP with depth information"""
        try:
            # Get 3D points from previous frame using depth
            object_points = []
            image_points = []
            
            for i, (prev_pt, curr_pt) in enumerate(zip(prev_pts, curr_pts)):
                # Get depth at previous point
                x, y = int(prev_pt[0]), int(prev_pt[1])
                if 0 <= x < depth_frame.shape[1] and 0 <= y < depth_frame.shape[0]:
                    depth = depth_frame[y, x]
                    if depth > 0:  # Valid depth
                        # Convert to 3D point
                        z = depth / 1000.0  # Convert mm to meters
                        x_3d = (x - self.K[0, 2]) * z / self.K[0, 0]
                        y_3d = (y - self.K[1, 2]) * z / self.K[1, 1]
                        
                        object_points.append([x_3d, y_3d, z])
                        image_points.append(curr_pt)
            
            if len(object_points) < 6:  # Minimum for PnP
                return False
            
            object_points = np.array(object_points, dtype=np.float32)
            image_points = np.array(image_points, dtype=np.float32)
            
            # Solve PnP
            success, rvec, tvec, inliers = cv2.solvePnPRansac(
                object_points, image_points, self.K, None,
                reprojectionError=5.0, confidence=0.99
            )
            
            if success and len(inliers) > 10:
                # Convert rotation vector to matrix
                R, _ = cv2.Rodrigues(rvec)
                
                # Update pose (relative to previous)
                delta_R = R
                delta_t = tvec.flatten()
                
                # Compose with previous pose
                prev_R = self.current_pose.orientation
                prev_t = self.current_pose.position
                
                new_R = prev_R @ delta_R
                new_t = prev_t + prev_R @ delta_t
                
                self.current_pose = Pose(
                    position=new_t,
                    orientation=new_R,
                    timestamp=timestamp,
                    confidence=len(inliers) / len(object_points)
                )
                
                self.poses.append(self.current_pose)
                return True
            
        except Exception as e:
            logger.error(f"PnP estimation failed: {e}")
        
        return False
    
    def _estimate_pose_essential(self, prev_pts: np.ndarray, curr_pts: np.ndarray,
                               timestamp: float) -> bool:
        """Estimate pose using essential matrix (monocular)"""
        try:
            # Find essential matrix
            E, mask = cv2.findEssentialMat(
                prev_pts, curr_pts, self.K,
                method=cv2.RANSAC, prob=0.999, threshold=1.0
            )
            
            if E is None:
                return False
            
            # Recover pose from essential matrix
            _, R, t, mask = cv2.recoverPose(E, prev_pts, curr_pts, self.K)
            
            # Check if we have enough inliers
            inlier_count = np.sum(mask)
            if inlier_count < self.min_matches:
                return False
            
            # Update pose (relative motion)
            prev_R = self.current_pose.orientation
            prev_t = self.current_pose.position
            
            # Compose transformations
            new_R = prev_R @ R
            new_t = prev_t + prev_R @ (t.flatten() * 0.1)  # Scale factor for monocular
            
            self.current_pose = Pose(
                position=new_t,
                orientation=new_R,
                timestamp=timestamp,
                confidence=inlier_count / len(prev_pts)
            )
            
            self.poses.append(self.current_pose)
            return True
            
        except Exception as e:
            logger.error(f"Essential matrix estimation failed: {e}")
            return False
    
    def _create_result(self, keypoints: List, matches: List, timestamp: float) -> Dict[str, Any]:
        """Create SLAM processing result"""
        return {
            'pose': self.current_pose,
            'pose_matrix': self.current_pose.to_matrix(),
            'keypoints': keypoints,
            'num_features': len(keypoints),
            'num_matches': len(matches),
            'matches': matches,
            'timestamp': timestamp,
            'is_tracking': not self.lost_tracking,
            'is_initialized': self.is_initialized,
            'map_size': len(self.map_points),
            'trajectory_length': len(self.poses)
        }
    
    def get_trajectory(self) -> List[Pose]:
        """Get camera trajectory"""
        return self.poses.copy()
    
    def get_map_points(self) -> List[MapPoint]:
        """Get 3D map points"""
        return self.map_points.copy()
    
    def reset(self):
        """Reset SLAM state"""
        self.current_pose = Pose(
            position=np.array([0.0, 0.0, 0.0]),
            orientation=np.eye(3),
            timestamp=time.time()
        )
        self.poses = [self.current_pose]
        self.map_points = []
        self.is_initialized = False
        self.lost_tracking = False
        self.prev_frame = None
        self.prev_keypoints = None
        self.prev_descriptors = None
    
    def visualize_trajectory(self, img_size: Tuple[int, int] = (800, 600)) -> np.ndarray:
        """Create top-down trajectory visualization"""
        img = np.zeros((img_size[1], img_size[0], 3), dtype=np.uint8)
        
        if len(self.poses) < 2:
            return img
        
        # Get trajectory points
        positions = np.array([pose.position for pose in self.poses])
        
        # Scale and center trajectory
        if len(positions) > 1:
            min_pos = np.min(positions, axis=0)
            max_pos = np.max(positions, axis=0)
            
            # Scale to fit image
            scale = min(img_size[0] * 0.8 / (max_pos[0] - min_pos[0] + 1e-6),
                       img_size[1] * 0.8 / (max_pos[2] - min_pos[2] + 1e-6))
            
            center_x = img_size[0] // 2
            center_y = img_size[1] // 2
            
            # Draw trajectory
            prev_point = None
            for pose in self.poses:
                x = int(center_x + (pose.position[0] - min_pos[0]) * scale)
                y = int(center_y + (pose.position[2] - min_pos[2]) * scale)
                
                if prev_point is not None:
                    cv2.line(img, prev_point, (x, y), (0, 255, 0), 2)
                
                cv2.circle(img, (x, y), 3, (0, 0, 255), -1)
                prev_point = (x, y)
            
            # Draw current position
            if self.poses:
                curr_pose = self.poses[-1]
                x = int(center_x + (curr_pose.position[0] - min_pos[0]) * scale)
                y = int(center_y + (curr_pose.position[2] - min_pos[2]) * scale)
                cv2.circle(img, (x, y), 8, (255, 255, 0), -1)
        
        return img


# Test SLAM
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    
    # Test with webcam
    cap = cv2.VideoCapture(0)
    
    # Camera intrinsics (approximate for webcam)
    intrinsics = {
        'fx': 800, 'fy': 800,
        'cx': 320, 'cy': 240
    }
    
    slam = BasicSLAM(intrinsics)
    
    print("Testing Basic SLAM...")
    print("Press 'q' to quit, 'r' to reset")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Process frame
        result = slam.process_frame(frame)
        
        # Visualize
        vis_frame = frame.copy()
        
        # Draw keypoints
        if result['keypoints']:
            vis_frame = cv2.drawKeypoints(vis_frame, result['keypoints'], None, 
                                        color=(0, 255, 0))
        
        # Draw matches
        if result['num_matches'] > 0:
            cv2.putText(vis_frame, f"Matches: {result['num_matches']}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Draw tracking status
        status = "TRACKING" if result['is_tracking'] else "LOST"
        color = (0, 255, 0) if result['is_tracking'] else (0, 0, 255)
        cv2.putText(vis_frame, status, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        
        # Draw pose info
        pose = result['pose']
        cv2.putText(vis_frame, f"Pos: [{pose.position[0]:.2f}, {pose.position[1]:.2f}, {pose.position[2]:.2f}]",
                   (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        cv2.imshow('SLAM Features', vis_frame)
        
        # Show trajectory
        traj_img = slam.visualize_trajectory()
        cv2.imshow('Trajectory', traj_img)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            slam.reset()
            print("SLAM reset")
    
    cap.release()
    cv2.destroyAllWindows()
    
    print(f"Final trajectory: {len(slam.get_trajectory())} poses")
    print("SLAM test completed!")