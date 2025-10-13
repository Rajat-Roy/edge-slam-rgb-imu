"""
Simple Visual Odometry using ORB features
Part of the SLAM MVP pipeline
"""
import cv2
import numpy as np
from typing import Tuple, List, Optional

class VisualOdometry:
    def __init__(self, camera_matrix: np.ndarray):
        """
        Initialize Visual Odometry

        Args:
            camera_matrix: 3x3 camera intrinsic matrix
        """
        self.K = camera_matrix
        self.orb = cv2.ORB_create(nfeatures=1000)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # Previous frame data
        self.prev_frame = None
        self.prev_keypoints = None
        self.prev_descriptors = None

        # Current pose (rotation and translation)
        self.R = np.eye(3)  # Rotation matrix
        self.t = np.zeros((3, 1))  # Translation vector

        # Trajectory
        self.trajectory = []
        self.poses = []

    def process_frame(self, frame: np.ndarray, timestamp: int) -> Tuple[np.ndarray, np.ndarray]:
        """
        Process a new frame and estimate motion

        Args:
            frame: Input image
            timestamp: Frame timestamp

        Returns:
            Tuple of (rotation_matrix, translation_vector)
        """
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ORB features
        keypoints, descriptors = self.orb.detectAndCompute(gray, None)

        if self.prev_frame is not None and descriptors is not None and self.prev_descriptors is not None:
            # Match features with previous frame
            matches = self.matcher.match(self.prev_descriptors, descriptors)
            matches = sorted(matches, key=lambda x: x.distance)

            if len(matches) > 10:  # Need minimum matches for pose estimation
                # Extract matched points
                prev_pts = np.float32([self.prev_keypoints[m.queryIdx].pt for m in matches])
                curr_pts = np.float32([keypoints[m.trainIdx].pt for m in matches])

                # Estimate essential matrix
                E, mask = cv2.findEssentialMat(
                    prev_pts, curr_pts, self.K,
                    method=cv2.RANSAC,
                    prob=0.999,
                    threshold=1.0
                )

                if E is not None:
                    # Recover pose from essential matrix
                    _, R_rel, t_rel, _ = cv2.recoverPose(E, prev_pts, curr_pts, self.K, mask=mask)

                    # Update global pose
                    self.t = self.t + self.R @ t_rel
                    self.R = R_rel @ self.R

                    # Store trajectory point
                    position = self.t.flatten()
                    self.trajectory.append({
                        'timestamp': timestamp,
                        'position': position.copy(),
                        'matches': len(matches)
                    })

                    # Store full pose
                    self.poses.append({
                        'timestamp': timestamp,
                        'R': self.R.copy(),
                        't': self.t.copy()
                    })
        else:
            # First frame - initialize trajectory
            if len(self.trajectory) == 0:
                self.trajectory.append({
                    'timestamp': timestamp,
                    'position': np.array([0.0, 0.0, 0.0]),
                    'matches': 0
                })
                self.poses.append({
                    'timestamp': timestamp,
                    'R': self.R.copy(),
                    't': self.t.copy()
                })

        # Update previous frame data
        self.prev_frame = gray.copy()
        self.prev_keypoints = keypoints
        self.prev_descriptors = descriptors

        return self.R.copy(), self.t.copy()

    def get_trajectory(self) -> List[dict]:
        """Get the computed trajectory"""
        return self.trajectory

    def get_poses(self) -> List[dict]:
        """Get all poses"""
        return self.poses

    def reset(self):
        """Reset the visual odometry state"""
        self.R = np.eye(3)
        self.t = np.zeros((3, 1))
        self.trajectory = []
        self.poses = []
        self.prev_frame = None
        self.prev_keypoints = None
        self.prev_descriptors = None