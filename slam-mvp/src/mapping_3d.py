"""
3D Mapping Module for Visual SLAM
Triangulates features to build sparse 3D environmental map
"""
import numpy as np
import cv2
from typing import Dict, List, Tuple, Optional
from collections import defaultdict

class MapPoint:
    """Represents a 3D landmark in the environment"""

    def __init__(self, point_id: int, position: np.ndarray):
        self.id = point_id
        self.position = position.copy()  # 3D world coordinates
        self.observations = []  # List of (frame_id, keypoint_idx, descriptor)
        self.creation_frame = None
        self.last_seen_frame = None
        self.observation_count = 0
        self.is_valid = True

    def add_observation(self, frame_id: int, keypoint_idx: int, descriptor: np.ndarray):
        """Add an observation of this map point"""
        self.observations.append({
            'frame_id': frame_id,
            'keypoint_idx': keypoint_idx,
            'descriptor': descriptor.copy()
        })
        self.last_seen_frame = frame_id
        self.observation_count += 1

    def get_observation_frames(self) -> List[int]:
        """Get list of frame IDs where this point was observed"""
        return [obs['frame_id'] for obs in self.observations]

class SparseMapper:
    """Builds sparse 3D map from visual features"""

    def __init__(self, camera_matrix: np.ndarray):
        self.K = camera_matrix
        self.map_points = {}  # Dict[point_id, MapPoint]
        self.next_point_id = 0

        # Feature tracking
        self.frame_features = {}  # Dict[frame_id, (keypoints, descriptors)]
        self.feature_tracks = defaultdict(list)  # Track features across frames

        # Parameters
        self.min_triangulation_angle = 2.0  # degrees (more permissive)
        self.max_reprojection_error = 5.0  # pixels (more permissive)
        self.min_track_length = 2  # minimum observations for triangulation

    def add_frame(self, frame_id: int, keypoints: List[cv2.KeyPoint],
                  descriptors: np.ndarray, pose_R: np.ndarray, pose_t: np.ndarray):
        """
        Add a new frame with features and camera pose

        Args:
            frame_id: Unique frame identifier
            keypoints: Detected keypoints
            descriptors: Feature descriptors
            pose_R: Camera rotation matrix (3x3)
            pose_t: Camera translation vector (3x1)
        """
        # Store frame data
        self.frame_features[frame_id] = {
            'keypoints': keypoints,
            'descriptors': descriptors,
            'R': pose_R.copy(),
            't': pose_t.copy(),
            'camera_center': -pose_R.T @ pose_t
        }

        # Track features if we have previous frames
        if len(self.frame_features) > 1:
            self._track_features(frame_id)
            self._triangulate_new_points(frame_id)

    def _track_features(self, current_frame_id: int):
        """Track features between current and previous frames"""
        current_data = self.frame_features[current_frame_id]
        current_desc = current_data['descriptors']

        # Find previous frame
        frame_ids = sorted(self.frame_features.keys())
        if len(frame_ids) < 2:
            return

        prev_frame_id = frame_ids[-2]  # Previous frame
        prev_data = self.frame_features[prev_frame_id]
        prev_desc = prev_data['descriptors']

        if current_desc is None or prev_desc is None:
            return

        # Match features
        matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = matcher.match(prev_desc, current_desc)
        matches = sorted(matches, key=lambda x: x.distance)

        # Update feature tracks
        for match in matches:
            if match.distance < 80:  # Good match threshold (more permissive)
                prev_kp_idx = match.queryIdx
                curr_kp_idx = match.trainIdx

                # Create track ID based on first occurrence
                track_id = f"{prev_frame_id}_{prev_kp_idx}"

                # Add to track
                self.feature_tracks[track_id].append({
                    'frame_id': current_frame_id,
                    'keypoint_idx': curr_kp_idx,
                    'descriptor': current_desc[curr_kp_idx]
                })

                # Add previous frame if not already in track
                if len(self.feature_tracks[track_id]) == 1:
                    self.feature_tracks[track_id].insert(0, {
                        'frame_id': prev_frame_id,
                        'keypoint_idx': prev_kp_idx,
                        'descriptor': prev_desc[prev_kp_idx]
                    })

    def _triangulate_new_points(self, current_frame_id: int):
        """Triangulate 3D points from feature tracks"""
        for track_id, track in self.feature_tracks.items():
            if len(track) < self.min_track_length:
                continue

            # Check if already triangulated
            if any(self._track_has_map_point(track_id, mp) for mp in self.map_points.values()):
                continue

            # Get the first and last observations for triangulation
            first_obs = track[0]
            last_obs = track[-1]

            # Get camera poses
            frame1_id = first_obs['frame_id']
            frame2_id = last_obs['frame_id']

            if frame1_id not in self.frame_features or frame2_id not in self.frame_features:
                continue

            pose1 = self.frame_features[frame1_id]
            pose2 = self.frame_features[frame2_id]

            # Get 2D points
            kp1 = pose1['keypoints'][first_obs['keypoint_idx']]
            kp2 = pose2['keypoints'][last_obs['keypoint_idx']]

            pt1 = np.array([kp1.pt[0], kp1.pt[1]], dtype=np.float32)
            pt2 = np.array([kp2.pt[0], kp2.pt[1]], dtype=np.float32)

            # Triangulate 3D point
            point_3d = self._triangulate_point(pt1, pt2, pose1, pose2)

            if point_3d is not None:
                # Create new map point
                map_point = MapPoint(self.next_point_id, point_3d)
                map_point.creation_frame = frame1_id

                # Add all observations from track
                for obs in track:
                    map_point.add_observation(
                        obs['frame_id'],
                        obs['keypoint_idx'],
                        obs['descriptor']
                    )

                self.map_points[self.next_point_id] = map_point
                self.next_point_id += 1

    def _track_has_map_point(self, track_id: str, map_point: MapPoint) -> bool:
        """Check if a track already corresponds to a map point"""
        track = self.feature_tracks[track_id]
        if len(track) < 2:
            return False

        # Check if any observations match
        for obs in track:
            for mp_obs in map_point.observations:
                if (mp_obs['frame_id'] == obs['frame_id'] and
                    mp_obs['keypoint_idx'] == obs['keypoint_idx']):
                    return True
        return False

    def _triangulate_point(self, pt1: np.ndarray, pt2: np.ndarray,
                          pose1: Dict, pose2: Dict) -> Optional[np.ndarray]:
        """
        Triangulate 3D point from two camera views

        Args:
            pt1, pt2: 2D points in each camera
            pose1, pose2: Camera poses with R, t, camera_center

        Returns:
            3D point in world coordinates or None if triangulation fails
        """
        # Check triangulation angle
        camera_center1 = pose1['camera_center'].flatten()
        camera_center2 = pose2['camera_center'].flatten()

        baseline = np.linalg.norm(camera_center2 - camera_center1)
        if baseline < 0.001:  # Too close (more permissive)
            return None

        # Convert to normalized coordinates
        pt1_norm = np.linalg.inv(self.K) @ np.array([pt1[0], pt1[1], 1])
        pt2_norm = np.linalg.inv(self.K) @ np.array([pt2[0], pt2[1], 1])

        # Create projection matrices
        P1 = self.K @ np.hstack([pose1['R'], pose1['t']])
        P2 = self.K @ np.hstack([pose2['R'], pose2['t']])

        # Triangulate using OpenCV
        points_4d = cv2.triangulatePoints(P1, P2, pt1.reshape(2, 1), pt2.reshape(2, 1))

        if points_4d[3, 0] == 0:
            return None

        # Convert from homogeneous coordinates
        point_3d = points_4d[:3, 0] / points_4d[3, 0]

        # Check if point is in front of both cameras
        if not self._is_point_in_front(point_3d, pose1) or not self._is_point_in_front(point_3d, pose2):
            return None

        # Check if point is at reasonable distance (filter out points too far)
        if np.linalg.norm(point_3d) > 50:  # More than 50m away (tighter for indoor)
            return None

        # Check reprojection error
        if not self._check_reprojection_error(point_3d, pt1, pose1, self.max_reprojection_error):
            return None
        if not self._check_reprojection_error(point_3d, pt2, pose2, self.max_reprojection_error):
            return None

        return point_3d

    def _is_point_in_front(self, point_3d: np.ndarray, pose: Dict) -> bool:
        """Check if 3D point is in front of camera"""
        # Transform point to camera frame
        point_cam = pose['R'] @ point_3d + pose['t'].flatten()
        return point_cam[2] > 0  # Z > 0 in camera frame

    def _check_reprojection_error(self, point_3d: np.ndarray, pt_2d: np.ndarray,
                                  pose: Dict, threshold: float) -> bool:
        """Check if reprojection error is within threshold"""
        # Project 3D point to image
        point_cam = pose['R'] @ point_3d + pose['t'].flatten()
        if point_cam[2] <= 0:
            return False

        projected = self.K @ point_cam
        projected = projected[:2] / projected[2]

        # Calculate error
        error = np.linalg.norm(projected - pt_2d)
        return error < threshold

    def get_map_points(self) -> List[np.ndarray]:
        """Get all valid 3D map points"""
        return [mp.position for mp in self.map_points.values() if mp.is_valid]

    def get_map_point_colors(self) -> List[np.ndarray]:
        """Get colors for map points (for visualization)"""
        # Simple color coding based on observation count
        colors = []
        for mp in self.map_points.values():
            if mp.is_valid:
                # Color based on how well observed the point is
                obs_ratio = min(mp.observation_count / 10.0, 1.0)
                color = np.array([obs_ratio, 1.0 - obs_ratio, 0.5])
                colors.append(color)
        return colors

    def get_statistics(self) -> Dict:
        """Get mapping statistics"""
        valid_points = [mp for mp in self.map_points.values() if mp.is_valid]

        if not valid_points:
            return {
                'total_map_points': 0,
                'valid_map_points': 0,
                'avg_observations': 0,
                'map_bounds': None
            }

        positions = np.array([mp.position for mp in valid_points])
        observations = [mp.observation_count for mp in valid_points]

        return {
            'total_map_points': len(self.map_points),
            'valid_map_points': len(valid_points),
            'avg_observations': np.mean(observations),
            'map_bounds': {
                'min': positions.min(axis=0),
                'max': positions.max(axis=0),
                'range': positions.max(axis=0) - positions.min(axis=0)
            },
            'feature_tracks': len(self.feature_tracks)
        }