"""
Visual-Inertial SLAM System
Combines visual odometry with IMU for robust tracking and scale estimation
"""
import cv2
import numpy as np
from typing import Tuple, List, Optional, Dict
from mapping_3d import SparseMapper
from imu_processor import IMUProcessor
from vi_initializer import VIInitializer

class VisualInertialSLAM:
    """Visual-Inertial SLAM with trajectory tracking and 3D mapping"""

    def __init__(self, camera_matrix: np.ndarray, imu_processor: IMUProcessor):
        """
        Initialize Visual-Inertial SLAM system

        Args:
            camera_matrix: 3x3 camera intrinsic matrix
            imu_processor: Configured IMU processor
        """
        self.K = camera_matrix
        self.imu_processor = imu_processor
        self.orb = cv2.ORB_create(nfeatures=1500)  # Balanced feature count
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # Visual-Inertial initialization
        self.vi_initializer = VIInitializer(camera_matrix, imu_processor)

        # Visual odometry state
        self.R = np.eye(3)  # Current rotation
        self.t = np.zeros((3, 1))  # Current translation

        # IMU prediction state
        self.predicted_R = np.eye(3)
        self.predicted_t = np.zeros((3, 1))

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
        self.prev_timestamp = None

        # System state
        self.initialized = False
        self.scale_factor = 1.0

        # Tracking parameters
        self.min_matches = 15
        self.use_imu_prediction = True

    def process_frame(self, frame: np.ndarray, timestamp: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Process a new frame for both odometry and mapping

        Args:
            frame: Input image
            timestamp: Frame timestamp (nanoseconds)

        Returns:
            Tuple of (rotation_matrix, translation_vector)
        """
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ORB features
        keypoints, descriptors = self.orb.detectAndCompute(gray, None)

        # Try VI initialization if not initialized
        if not self.initialized:
            if self.vi_initializer.add_frame(self.current_frame_id, keypoints, descriptors, timestamp):
                self.initialized = True
                self.scale_factor = self.vi_initializer.get_scale_factor()
                print(f"VI-SLAM initialized with scale factor: {self.scale_factor:.3f}")

        # Get IMU prediction if available
        if self.initialized and self.prev_timestamp is not None:
            imu_rotation, imu_translation = self.imu_processor.predict_motion(timestamp)
            self.predicted_R = imu_rotation @ self.R
            self.predicted_t = self.t + self.R @ (imu_translation.reshape(3, 1) * self.scale_factor)

        # Perform visual odometry if we have previous frame
        if self.prev_frame is not None and descriptors is not None and self.prev_descriptors is not None:
            # Match features with previous frame
            matches = self.matcher.match(self.prev_descriptors, descriptors)
            matches = sorted(matches, key=lambda x: x.distance)

            if len(matches) >= self.min_matches:
                # Extract matched points
                prev_pts = np.float32([self.prev_keypoints[m.queryIdx].pt for m in matches[:100]])
                curr_pts = np.float32([keypoints[m.trainIdx].pt for m in matches[:100]])

                # Use IMU prediction for RANSAC if available
                if self.initialized and self.use_imu_prediction:
                    visual_R, visual_t = self._estimate_pose_with_imu_prior(
                        prev_pts, curr_pts, self.predicted_R, self.predicted_t)
                else:
                    visual_R, visual_t = self._estimate_pose_visual_only(prev_pts, curr_pts)

                if visual_R is not None and visual_t is not None:
                    # Update global pose
                    if self.initialized:
                        # Use scaled visual translation
                        scaled_translation = visual_t * self.scale_factor
                        self.t = self.t + self.R @ scaled_translation.reshape(3, 1)
                        self.R = visual_R @ self.R
                    else:
                        # Before initialization, use unscaled
                        self.t = self.t + self.R @ visual_t.reshape(3, 1)
                        self.R = visual_R @ self.R

                    # Store trajectory point
                    position = self.t.flatten()
                    self.trajectory.append({
                        'timestamp': timestamp,
                        'position': position.copy(),
                        'matches': len(matches),
                        'frame_id': self.current_frame_id,
                        'initialized': self.initialized,
                        'scale_factor': self.scale_factor
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
                    'frame_id': self.current_frame_id,
                    'initialized': False,
                    'scale_factor': 1.0
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
        self.prev_timestamp = timestamp
        self.current_frame_id += 1

        return self.R.copy(), self.t.copy()

    def _estimate_pose_visual_only(self, prev_pts: np.ndarray, curr_pts: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Estimate pose using visual features only"""
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
            return R_rel, t_rel.flatten()

        return None, None

    def _estimate_pose_with_imu_prior(self, prev_pts: np.ndarray, curr_pts: np.ndarray,
                                     predicted_R: np.ndarray, predicted_t: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Estimate pose using visual features with IMU prior"""
        # First try standard visual estimation
        visual_R, visual_t = self._estimate_pose_visual_only(prev_pts, curr_pts)

        if visual_R is None or visual_t is None:
            # Fall back to IMU prediction
            relative_R = predicted_R @ self.R.T
            relative_t = (predicted_t - self.t).flatten()
            return relative_R, relative_t

        # Blend visual and IMU estimates (simple weighted average)
        imu_weight = 0.3 if self.initialized else 0.0  # Use IMU more after initialization

        if imu_weight > 0:
            # Get relative IMU motion
            relative_R_imu = predicted_R @ self.R.T
            relative_t_imu = (predicted_t - self.t).flatten()

            # Simple blending (could be improved with Kalman filtering)
            # For rotation, use SLERP-like blending
            blended_R = self._blend_rotations(visual_R, relative_R_imu, imu_weight)

            # For translation, use weighted average
            blended_t = (1.0 - imu_weight) * visual_t + imu_weight * relative_t_imu

            return blended_R, blended_t
        else:
            return visual_R, visual_t

    def _blend_rotations(self, R1: np.ndarray, R2: np.ndarray, weight: float) -> np.ndarray:
        """Blend two rotation matrices using quaternion SLERP"""
        # Convert to quaternions for blending
        q1 = self._rotation_matrix_to_quaternion(R1)
        q2 = self._rotation_matrix_to_quaternion(R2)

        # SLERP
        q_blended = self._slerp_quaternions(q1, q2, weight)

        # Convert back to rotation matrix
        return self._quaternion_to_rotation_matrix(q_blended)

    def _rotation_matrix_to_quaternion(self, R: np.ndarray) -> np.ndarray:
        """Convert rotation matrix to quaternion [w, x, y, z]"""
        trace = np.trace(R)
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2
            w = 0.25 * s
            x = (R[2, 1] - R[1, 2]) / s
            y = (R[0, 2] - R[2, 0]) / s
            z = (R[1, 0] - R[0, 1]) / s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
                w = (R[2, 1] - R[1, 2]) / s
                x = 0.25 * s
                y = (R[0, 1] + R[1, 0]) / s
                z = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
                w = (R[0, 2] - R[2, 0]) / s
                x = (R[0, 1] + R[1, 0]) / s
                y = 0.25 * s
                z = (R[1, 2] + R[2, 1]) / s
            else:
                s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
                w = (R[1, 0] - R[0, 1]) / s
                x = (R[0, 2] + R[2, 0]) / s
                y = (R[1, 2] + R[2, 1]) / s
                z = 0.25 * s
        return np.array([w, x, y, z])

    def _quaternion_to_rotation_matrix(self, q: np.ndarray) -> np.ndarray:
        """Convert quaternion [w, x, y, z] to rotation matrix"""
        w, x, y, z = q
        return np.array([
            [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
            [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
        ])

    def _slerp_quaternions(self, q1: np.ndarray, q2: np.ndarray, t: float) -> np.ndarray:
        """Spherical linear interpolation between quaternions"""
        dot = np.dot(q1, q2)

        # If dot product is negative, negate one quaternion to take shorter path
        if dot < 0.0:
            q2 = -q2
            dot = -dot

        # If quaternions are very similar, use linear interpolation
        if dot > 0.9995:
            result = q1 + t * (q2 - q1)
            return result / np.linalg.norm(result)

        # Calculate angle between quaternions
        theta_0 = np.arccos(np.abs(dot))
        sin_theta_0 = np.sin(theta_0)

        theta = theta_0 * t
        sin_theta = np.sin(theta)

        s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0

        return s0 * q1 + s1 * q2

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
        stats = self.mapper.get_statistics()
        stats.update({
            'vi_initialized': self.initialized,
            'scale_factor': self.scale_factor,
            'imu_calibrated': self.imu_processor.is_calibrated
        })
        return stats

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
            f.write("ply\\n")
            f.write("format ascii 1.0\\n")
            f.write(f"element vertex {len(map_points)}\\n")
            f.write("property float x\\n")
            f.write("property float y\\n")
            f.write("property float z\\n")
            f.write("property uchar red\\n")
            f.write("property uchar green\\n")
            f.write("property uchar blue\\n")
            f.write("end_header\\n")

            # Write point data
            for i, point in enumerate(map_points):
                color = colors[i] if i < len(colors) else [0.5, 0.5, 0.5]
                rgb = (np.array(color) * 255).astype(int)
                f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} "
                       f"{rgb[0]} {rgb[1]} {rgb[2]}\\n")

        print(f"Exported {len(map_points)} map points to {filename}")

    def get_frame_count(self) -> int:
        """Get number of processed frames"""
        return len(self.frames)

    def is_initialized(self) -> bool:
        """Check if VI-SLAM is initialized"""
        return self.initialized

    def get_scale_factor(self) -> float:
        """Get current scale factor"""
        return self.scale_factor

    def reset(self):
        """Reset the SLAM system"""
        self.R = np.eye(3)
        self.t = np.zeros((3, 1))
        self.predicted_R = np.eye(3)
        self.predicted_t = np.zeros((3, 1))
        self.trajectory = []
        self.poses = []
        self.frames = {}
        self.current_frame_id = 0
        self.prev_frame = None
        self.prev_keypoints = None
        self.prev_descriptors = None
        self.prev_timestamp = None
        self.initialized = False
        self.scale_factor = 1.0
        self.mapper = SparseMapper(self.K)
        self.vi_initializer.reset()
        self.imu_processor.reset_integration()