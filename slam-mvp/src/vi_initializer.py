"""
Visual-Inertial Initialization Module
Resolves scale ambiguity and initializes VIO system
"""
import numpy as np
import cv2
from typing import List, Dict, Tuple, Optional
from imu_processor import IMUProcessor, IMUData

class VIInitializer:
    """Visual-Inertial system initializer"""

    def __init__(self, camera_matrix: np.ndarray, imu_processor: IMUProcessor):
        self.K = camera_matrix
        self.imu_processor = imu_processor

        # Initialization parameters
        self.min_frames = 10  # Minimum frames for initialization
        self.max_frames = 30  # Maximum frames to consider
        self.min_parallax = 20.0  # Minimum parallax for initialization (pixels)
        self.min_translation = 0.1  # Minimum translation for scale estimation (meters)

        # Stored frame data for initialization
        self.init_frames = []  # List of frame data
        self.initialized = False
        self.scale_factor = 1.0

    def add_frame(self, frame_id: int, keypoints: List[cv2.KeyPoint],
                  descriptors: np.ndarray, timestamp: float) -> bool:
        """
        Add frame for visual-inertial initialization

        Args:
            frame_id: Frame identifier
            keypoints: Detected keypoints
            descriptors: Feature descriptors
            timestamp: Frame timestamp

        Returns:
            True if initialization is complete
        """
        frame_data = {
            'frame_id': frame_id,
            'keypoints': keypoints,
            'descriptors': descriptors,
            'timestamp': timestamp
        }

        self.init_frames.append(frame_data)

        # Remove old frames if we have too many
        if len(self.init_frames) > self.max_frames:
            self.init_frames.pop(0)

        # Try initialization if we have enough frames
        if len(self.init_frames) >= self.min_frames:
            return self._attempt_initialization()

        return False

    def _attempt_initialization(self) -> bool:
        """Attempt visual-inertial initialization"""
        if self.initialized:
            return True

        # Get first and last frames for initialization
        first_frame = self.init_frames[0]
        last_frame = self.init_frames[-1]

        # Match features between first and last frames
        matches = self._match_features(first_frame, last_frame)

        if len(matches) < 50:  # Need sufficient matches
            return False

        # Check parallax
        parallax = self._calculate_parallax(first_frame, last_frame, matches)
        if parallax < self.min_parallax:
            return False

        # Estimate visual odometry between frames
        visual_rotation, visual_translation = self._estimate_visual_motion(
            first_frame, last_frame, matches)

        if visual_rotation is None or visual_translation is None:
            return False

        # Get IMU prediction for the same time period
        imu_rotation, imu_translation = self._get_imu_motion(
            first_frame['timestamp'], last_frame['timestamp'])

        # Estimate scale factor
        scale = self._estimate_scale(visual_translation, imu_translation)

        if scale is None or scale <= 0:
            return False

        # Validate scale estimate
        if self._validate_scale(scale, visual_rotation, imu_rotation):
            self.scale_factor = scale
            self.initialized = True
            print(f"Visual-Inertial initialization successful!")
            print(f"  Scale factor: {self.scale_factor:.3f}")
            print(f"  Parallax: {parallax:.1f} pixels")
            print(f"  Matches: {len(matches)}")
            return True

        return False

    def _match_features(self, frame1: Dict, frame2: Dict) -> List[cv2.DMatch]:
        """Match features between two frames"""
        if frame1['descriptors'] is None or frame2['descriptors'] is None:
            return []

        matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = matcher.match(frame1['descriptors'], frame2['descriptors'])
        matches = sorted(matches, key=lambda x: x.distance)

        # Filter good matches
        good_matches = [m for m in matches if m.distance < 50]
        return good_matches[:100]  # Limit to best 100 matches

    def _calculate_parallax(self, frame1: Dict, frame2: Dict,
                           matches: List[cv2.DMatch]) -> float:
        """Calculate median parallax between matched features"""
        if len(matches) == 0:
            return 0.0

        parallaxes = []
        for match in matches:
            pt1 = frame1['keypoints'][match.queryIdx].pt
            pt2 = frame2['keypoints'][match.trainIdx].pt

            parallax = np.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)
            parallaxes.append(parallax)

        return np.median(parallaxes)

    def _estimate_visual_motion(self, frame1: Dict, frame2: Dict,
                               matches: List[cv2.DMatch]) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Estimate visual motion between two frames"""
        if len(matches) < 8:  # Need minimum matches for essential matrix
            return None, None

        # Extract matched points
        pts1 = np.float32([frame1['keypoints'][m.queryIdx].pt for m in matches])
        pts2 = np.float32([frame2['keypoints'][m.trainIdx].pt for m in matches])

        # Estimate essential matrix
        E, mask = cv2.findEssentialMat(
            pts1, pts2, self.K,
            method=cv2.RANSAC,
            prob=0.999,
            threshold=1.0
        )

        if E is None:
            return None, None

        # Recover pose
        _, R, t, _ = cv2.recoverPose(E, pts1, pts2, self.K, mask=mask)

        return R, t.flatten()

    def _get_imu_motion(self, start_timestamp: float, end_timestamp: float) -> Tuple[np.ndarray, np.ndarray]:
        """Get IMU-predicted motion between timestamps"""
        if not self.imu_processor.is_calibrated:
            return np.eye(3), np.zeros(3)

        # Reset integration state
        self.imu_processor.reset_integration()

        # Get IMU measurements in range
        imu_measurements = self.imu_processor.get_imu_measurements_in_range(
            start_timestamp, end_timestamp)

        if len(imu_measurements) == 0:
            return np.eye(3), np.zeros(3)

        # Integrate IMU measurements
        total_rotation = np.eye(3)
        total_translation = np.zeros(3)

        prev_timestamp = start_timestamp

        for imu_data in imu_measurements:
            dt = (imu_data.timestamp - prev_timestamp) / 1e9  # Convert to seconds

            if dt <= 0:
                continue

            # Get corrected measurements
            acc_corrected, gyro_corrected = self.imu_processor.get_corrected_measurements(imu_data)

            # Integrate rotation
            angular_velocity_magnitude = np.linalg.norm(gyro_corrected)
            if angular_velocity_magnitude > 1e-6:
                axis = gyro_corrected / angular_velocity_magnitude
                angle = angular_velocity_magnitude * dt

                # Rodrigues' rotation formula
                K = np.array([[0, -axis[2], axis[1]],
                             [axis[2], 0, -axis[0]],
                             [-axis[1], axis[0], 0]])

                rotation_step = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K
                total_rotation = rotation_step @ total_rotation

            # Integrate translation (double integration of acceleration)
            acc_world = total_rotation @ acc_corrected
            # Note: Not removing gravity here as we're looking at relative motion

            translation_step = acc_world * dt * dt  # Simple double integration
            total_translation += translation_step

            prev_timestamp = imu_data.timestamp

        return total_rotation, total_translation

    def _estimate_scale(self, visual_translation: np.ndarray,
                       imu_translation: np.ndarray) -> Optional[float]:
        """Estimate scale factor from visual and IMU translations"""
        visual_magnitude = np.linalg.norm(visual_translation)
        imu_magnitude = np.linalg.norm(imu_translation)

        if visual_magnitude < 1e-6 or imu_magnitude < 1e-6:
            return None

        # Scale factor: how much to scale visual translation to match IMU
        scale = imu_magnitude / visual_magnitude

        # Sanity check: scale should be reasonable for mobile device
        if scale < 0.001 or scale > 1000.0:
            return None

        return scale

    def _validate_scale(self, scale: float, visual_rotation: np.ndarray,
                       imu_rotation: np.ndarray) -> bool:
        """Validate scale estimate using rotation consistency"""
        # Check if rotations are consistent (should be similar)
        rotation_diff = visual_rotation @ imu_rotation.T
        angle_diff = np.arccos(np.clip((np.trace(rotation_diff) - 1) / 2, -1, 1))

        # Allow up to 30 degrees difference (IMU and visual may have different reference frames)
        max_angle_diff = np.radians(30)

        if angle_diff > max_angle_diff:
            print(f"Rotation inconsistency: {np.degrees(angle_diff):.1f} degrees")
            return False

        # Check if scale is reasonable for typical motion
        if scale < 0.01 or scale > 100.0:
            print(f"Unreasonable scale: {scale:.3f}")
            return False

        return True

    def get_scale_factor(self) -> float:
        """Get the estimated scale factor"""
        return self.scale_factor if self.initialized else 1.0

    def is_initialized(self) -> bool:
        """Check if initialization is complete"""
        return self.initialized

    def reset(self):
        """Reset initialization state"""
        self.init_frames = []
        self.initialized = False
        self.scale_factor = 1.0

    def get_initialization_info(self) -> Dict:
        """Get initialization information"""
        return {
            'initialized': self.initialized,
            'scale_factor': self.scale_factor,
            'frames_used': len(self.init_frames),
            'min_frames_required': self.min_frames
        }