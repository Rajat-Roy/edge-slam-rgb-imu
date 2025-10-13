"""
Enhanced Visual SLAM with 3D Mapping
Combines visual odometry with sparse 3D environmental mapping
"""
import cv2
import numpy as np
from typing import Tuple, List, Optional, Dict
from mapping_3d import SparseMapper

class VisualSLAM:
    """Visual SLAM with trajectory tracking and 3D mapping"""

    def __init__(self, camera_matrix: np.ndarray):
        """
        Initialize Visual SLAM system

        Args:
            camera_matrix: 3x3 camera intrinsic matrix
        """
        self.K = camera_matrix
        self.orb = cv2.ORB_create(nfeatures=2000)  # More features for dense mapping
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # Visual odometry state
        self.R = np.eye(3)  # Current rotation
        self.t = np.zeros((3, 1))  # Current translation

        # Frame data storage
        self.frames = {}  # Store frame data for mapping
        self.current_frame_id = 0

        # 3D mapping
        self.mapper = SparseMapper(camera_matrix)

        # Trajectory
        self.trajectory = []
        self.poses = []

        # Previous frame for odometry
        self.prev_frame = None
        self.prev_keypoints = None
        self.prev_descriptors = None

    def process_frame(self, frame: np.ndarray, timestamp: int) -> Tuple[np.ndarray, np.ndarray]:
        """
        Process a new frame for both odometry and mapping

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

        # Perform visual odometry if we have previous frame
        if self.prev_frame is not None and descriptors is not None and self.prev_descriptors is not None:
            # Match features with previous frame
            matches = self.matcher.match(self.prev_descriptors, descriptors)
            matches = sorted(matches, key=lambda x: x.distance)

            if len(matches) > 15:  # Need minimum matches for pose estimation
                # Extract matched points
                prev_pts = np.float32([self.prev_keypoints[m.queryIdx].pt for m in matches[:100]])
                curr_pts = np.float32([keypoints[m.trainIdx].pt for m in matches[:100]])

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
                        'matches': len(matches),
                        'frame_id': self.current_frame_id
                    })

                    # Store full pose
                    self.poses.append({
                        'timestamp': timestamp,
                        'R': self.R.copy(),
                        't': self.t.copy(),
                        'frame_id': self.current_frame_id
                    })

                    # Add frame to mapper
                    if keypoints and descriptors is not None:
                        self.mapper.add_frame(
                            self.current_frame_id,
                            keypoints,
                            descriptors,
                            self.R.copy(),
                            self.t.copy()
                        )

        else:
            # First frame - initialize
            if len(self.trajectory) == 0:
                self.trajectory.append({
                    'timestamp': timestamp,
                    'position': np.array([0.0, 0.0, 0.0]),
                    'matches': 0,
                    'frame_id': self.current_frame_id
                })
                self.poses.append({
                    'timestamp': timestamp,
                    'R': self.R.copy(),
                    't': self.t.copy(),
                    'frame_id': self.current_frame_id
                })

                # Add first frame to mapper
                if keypoints and descriptors is not None:
                    self.mapper.add_frame(
                        self.current_frame_id,
                        keypoints,
                        descriptors,
                        self.R.copy(),
                        self.t.copy()
                    )

        # Store frame data
        self.frames[self.current_frame_id] = {
            'timestamp': timestamp,
            'gray': gray.copy(),
            'keypoints': keypoints,
            'descriptors': descriptors,
            'R': self.R.copy(),
            't': self.t.copy()
        }

        # Update previous frame data
        self.prev_frame = gray.copy()
        self.prev_keypoints = keypoints
        self.prev_descriptors = descriptors
        self.current_frame_id += 1

        return self.R.copy(), self.t.copy()

    def get_trajectory(self) -> List[dict]:
        """Get the computed trajectory"""
        return self.trajectory

    def get_poses(self) -> List[dict]:
        """Get all poses"""
        return self.poses

    def get_map_points(self) -> List[np.ndarray]:
        """Get 3D map points"""
        return self.mapper.get_map_points()

    def get_map_statistics(self) -> Dict:
        """Get mapping statistics"""
        return self.mapper.get_statistics()

    def export_map_ply(self, filename: str):
        """
        Export 3D map to PLY point cloud format

        Args:
            filename: Output PLY file path
        """
        map_points = self.mapper.get_map_points()
        colors = self.mapper.get_map_point_colors()

        if not map_points:
            print("No map points to export")
            return

        # Write PLY header
        with open(filename, 'w') as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {len(map_points)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("property uchar red\n")
            f.write("property uchar green\n")
            f.write("property uchar blue\n")
            f.write("end_header\n")

            # Write point data
            for i, point in enumerate(map_points):
                color = colors[i] if i < len(colors) else [0.5, 0.5, 0.5]
                rgb = (np.array(color) * 255).astype(int)
                f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} "
                       f"{rgb[0]} {rgb[1]} {rgb[2]}\n")

        print(f"Exported {len(map_points)} map points to {filename}")

    def get_frame_count(self) -> int:
        """Get number of processed frames"""
        return len(self.frames)

    def reset(self):
        """Reset the SLAM system"""
        self.R = np.eye(3)
        self.t = np.zeros((3, 1))
        self.trajectory = []
        self.poses = []
        self.frames = {}
        self.current_frame_id = 0
        self.prev_frame = None
        self.prev_keypoints = None
        self.prev_descriptors = None
        self.mapper = SparseMapper(self.K)