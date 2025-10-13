"""
IMU Integration for Visual-Inertial SLAM
Simple dead-reckoning with accelerometer and gyroscope
"""
import numpy as np
from typing import List, Dict, Tuple
from scipy.spatial.transform import Rotation as R

class IMUIntegration:
    def __init__(self, gravity: float = 9.81):
        """
        Initialize IMU integration

        Args:
            gravity: Gravitational acceleration magnitude
        """
        self.gravity = gravity
        self.gravity_vector = np.array([0, 0, -gravity])

        # State variables
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = R.from_quat([0, 0, 0, 1])  # Identity quaternion

        # IMU biases (simple constant bias model)
        self.accel_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)

        # Integration results
        self.trajectory = []
        self.orientations = []

        # Previous timestamp for integration
        self.prev_timestamp = None

    def calibrate_biases(self, accel_data: List[Dict], gyro_data: List[Dict], static_duration: float = 2.0):
        """
        Calibrate IMU biases assuming initial static period

        Args:
            accel_data: List of accelerometer measurements
            gyro_data: List of gyroscope measurements
            static_duration: Duration in seconds to use for calibration
        """
        if not accel_data or not gyro_data:
            return

        # Find measurements within static duration from start
        start_time = min(accel_data[0]['timestamp'], gyro_data[0]['timestamp'])
        static_end = start_time + static_duration * 1e9  # Convert to nanoseconds

        static_accel = [sample for sample in accel_data if sample['timestamp'] <= static_end]
        static_gyro = [sample for sample in gyro_data if sample['timestamp'] <= static_end]

        if len(static_accel) > 10 and len(static_gyro) > 10:
            # Calculate mean accelerometer reading (should be gravity vector)
            accel_mean = np.mean([sample['values'] for sample in static_accel], axis=0)

            # Gyroscope bias is simply the mean (should be zero when static)
            self.gyro_bias = np.mean([sample['values'] for sample in static_gyro], axis=0)

            # Accelerometer bias is mean minus gravity vector
            # Assuming initial orientation has Z-axis aligned with gravity
            gravity_measured = np.linalg.norm(accel_mean)
            gravity_direction = accel_mean / gravity_measured

            # Simple bias estimation (assuming phone held upright initially)
            expected_gravity = np.array([0, 0, gravity_measured])
            self.accel_bias = accel_mean - expected_gravity

            print(f"IMU Calibration:")
            print(f"  Accel bias: {self.accel_bias}")
            print(f"  Gyro bias: {self.gyro_bias}")
            print(f"  Gravity magnitude: {gravity_measured:.2f} m/s²")

    def integrate_step(self, accel: np.ndarray, gyro: np.ndarray, timestamp: int) -> Tuple[np.ndarray, np.ndarray]:
        """
        Integrate one IMU measurement step

        Args:
            accel: Accelerometer measurement [x, y, z] in m/s²
            gyro: Gyroscope measurement [x, y, z] in rad/s
            timestamp: Measurement timestamp in nanoseconds

        Returns:
            Tuple of (position, velocity)
        """
        if self.prev_timestamp is None:
            self.prev_timestamp = timestamp
            return self.position.copy(), self.velocity.copy()

        # Calculate time step
        dt = (timestamp - self.prev_timestamp) / 1e9  # Convert to seconds
        if dt <= 0 or dt > 1.0:  # Skip invalid time steps
            self.prev_timestamp = timestamp
            return self.position.copy(), self.velocity.copy()

        # Remove biases
        accel_corrected = accel - self.accel_bias
        gyro_corrected = gyro - self.gyro_bias

        # Integrate angular velocity to get orientation
        rotation_vec = gyro_corrected * dt
        rotation_delta = R.from_rotvec(rotation_vec)
        self.orientation = self.orientation * rotation_delta

        # Transform accelerometer to world frame and remove gravity
        accel_world = self.orientation.apply(accel_corrected) + self.gravity_vector

        # Integrate acceleration to get velocity and position
        self.velocity += accel_world * dt
        self.position += self.velocity * dt

        # Store results
        self.trajectory.append({
            'timestamp': timestamp,
            'position': self.position.copy(),
            'velocity': self.velocity.copy(),
            'accel_world': accel_world.copy()
        })

        self.orientations.append({
            'timestamp': timestamp,
            'orientation': self.orientation,
            'gyro': gyro_corrected.copy()
        })

        self.prev_timestamp = timestamp
        return self.position.copy(), self.velocity.copy()

    def integrate_sequence(self, accel_data: List[Dict], gyro_data: List[Dict]):
        """
        Integrate a sequence of IMU measurements

        Args:
            accel_data: List of accelerometer measurements
            gyro_data: List of gyroscope measurements
        """
        # First calibrate biases
        self.calibrate_biases(accel_data, gyro_data)

        # Merge and sort IMU data by timestamp
        all_imu = []

        for sample in accel_data:
            all_imu.append({
                'timestamp': sample['timestamp'],
                'type': 'accel',
                'values': np.array(sample['values'])
            })

        for sample in gyro_data:
            all_imu.append({
                'timestamp': sample['timestamp'],
                'type': 'gyro',
                'values': np.array(sample['values'])
            })

        all_imu.sort(key=lambda x: x['timestamp'])

        # Integrate step by step
        current_accel = None
        current_gyro = None

        for sample in all_imu:
            if sample['type'] == 'accel':
                current_accel = sample['values']
            elif sample['type'] == 'gyro':
                current_gyro = sample['values']

            # Integrate when we have both measurements
            if current_accel is not None and current_gyro is not None:
                self.integrate_step(current_accel, current_gyro, sample['timestamp'])

    def get_trajectory(self) -> List[Dict]:
        """Get the integrated trajectory"""
        return self.trajectory

    def get_orientations(self) -> List[Dict]:
        """Get the orientation history"""
        return self.orientations

    def reset(self):
        """Reset the integration state"""
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = R.from_quat([0, 0, 0, 1])
        self.trajectory = []
        self.orientations = []
        self.prev_timestamp = None