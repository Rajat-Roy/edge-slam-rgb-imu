"""
IMU Data Processor for Visual-Inertial SLAM
Handles IMU preprocessing, calibration, and integration
"""
import numpy as np
from typing import List, Dict, Tuple, Optional
from collections import deque
import json

class IMUData:
    """Container for IMU measurements"""

    def __init__(self, timestamp: float, acc: np.ndarray, gyro: np.ndarray):
        self.timestamp = timestamp
        self.acceleration = acc.copy()  # m/s²
        self.gyroscope = gyro.copy()    # rad/s

class IMUProcessor:
    """Processes and integrates IMU data for SLAM"""

    def __init__(self, gravity_magnitude: float = 9.81):
        self.gravity = gravity_magnitude

        # IMU data storage
        self.imu_data = []  # List of IMUData

        # Calibration parameters
        self.acc_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)
        self.is_calibrated = False

        # Integration state
        self.velocity = np.zeros(3)  # Current velocity
        self.position = np.zeros(3)  # Current position
        self.orientation = np.eye(3)  # Current rotation matrix

        # Previous timestamp for integration
        self.prev_timestamp = None

        # Noise parameters (typical for mobile IMU)
        self.acc_noise_std = 0.1    # m/s²
        self.gyro_noise_std = 0.01  # rad/s
        self.acc_bias_std = 0.01    # m/s²
        self.gyro_bias_std = 0.001  # rad/s

    def load_imu_data(self, data_loader) -> int:
        """Load IMU data from data loader"""
        imu_samples = data_loader.get_imu_data()

        acc_data = imu_samples.get('accelerometer', [])
        gyro_data = imu_samples.get('gyroscope', [])

        # Combine and sort by timestamp
        combined_data = []

        for acc_sample in acc_data:
            combined_data.append({
                'timestamp': acc_sample['timestamp'],
                'type': 'acc',
                'data': acc_sample
            })

        for gyro_sample in gyro_data:
            combined_data.append({
                'timestamp': gyro_sample['timestamp'],
                'type': 'gyro',
                'data': gyro_sample
            })

        # Sort by timestamp
        combined_data.sort(key=lambda x: x['timestamp'])

        # Process into IMU data pairs
        latest_acc = None
        latest_gyro = None

        for sample in combined_data:
            if sample['type'] == 'acc':
                latest_acc = sample['data']
            elif sample['type'] == 'gyro':
                latest_gyro = sample['data']

            # Create IMU data when we have both acc and gyro
            if latest_acc and latest_gyro:
                # Use the newer timestamp
                timestamp = max(latest_acc['timestamp'], latest_gyro['timestamp'])

                acc = np.array(latest_acc['values'])  # Android format uses 'values' array
                gyro = np.array(latest_gyro['values'])  # Android format uses 'values' array

                self.imu_data.append(IMUData(timestamp, acc, gyro))

        print(f"Loaded {len(self.imu_data)} IMU measurement pairs")
        return len(self.imu_data)

    def calibrate_static_bias(self, duration_seconds: float = 2.0) -> bool:
        """
        Calibrate IMU biases assuming device is stationary at start

        Args:
            duration_seconds: How long to use for calibration

        Returns:
            True if calibration successful
        """
        if len(self.imu_data) == 0:
            return False

        # Find samples within calibration duration
        start_time = self.imu_data[0].timestamp
        calib_samples = []

        for imu in self.imu_data:
            if (imu.timestamp - start_time) / 1e9 <= duration_seconds:  # Convert ns to s
                calib_samples.append(imu)
            else:
                break

        if len(calib_samples) < 10:
            print("Not enough samples for calibration")
            return False

        # Calculate bias as mean of stationary period
        acc_measurements = np.array([imu.acceleration for imu in calib_samples])
        gyro_measurements = np.array([imu.gyroscope for imu in calib_samples])

        # Gyro bias is mean (should be zero when stationary)
        self.gyro_bias = np.mean(gyro_measurements, axis=0)

        # Accelerometer bias: remove gravity component
        acc_mean = np.mean(acc_measurements, axis=0)
        gravity_vector = acc_mean / np.linalg.norm(acc_mean) * self.gravity
        self.acc_bias = acc_mean - gravity_vector

        self.is_calibrated = True

        print(f"IMU calibration complete:")
        print(f"  Accelerometer bias: {self.acc_bias}")
        print(f"  Gyroscope bias: {self.gyro_bias}")
        print(f"  Calibration samples: {len(calib_samples)}")

        return True

    def get_corrected_measurements(self, imu_data: IMUData) -> Tuple[np.ndarray, np.ndarray]:
        """Get bias-corrected IMU measurements"""
        acc_corrected = imu_data.acceleration - self.acc_bias
        gyro_corrected = imu_data.gyroscope - self.gyro_bias
        return acc_corrected, gyro_corrected

    def predict_motion(self, timestamp: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Predict camera motion using IMU integration

        Args:
            timestamp: Current timestamp (nanoseconds)

        Returns:
            Tuple of (predicted_rotation, predicted_translation)
        """
        if not self.is_calibrated:
            return np.eye(3), np.zeros(3)

        if self.prev_timestamp is None:
            self.prev_timestamp = timestamp
            return np.eye(3), np.zeros(3)

        # Find IMU measurements between prev_timestamp and timestamp
        dt_total = (timestamp - self.prev_timestamp) / 1e9  # Convert ns to seconds

        if dt_total <= 0:
            return np.eye(3), np.zeros(3)

        # Get relevant IMU measurements
        relevant_imus = []
        for imu in self.imu_data:
            if self.prev_timestamp <= imu.timestamp <= timestamp:
                relevant_imus.append(imu)

        if len(relevant_imus) == 0:
            # No IMU data, return identity
            self.prev_timestamp = timestamp
            return np.eye(3), np.zeros(3)

        # Integrate IMU measurements
        total_rotation = np.eye(3)
        total_velocity_change = np.zeros(3)

        for i, imu in enumerate(relevant_imus):
            # Calculate time step
            if i == 0:
                dt = (imu.timestamp - self.prev_timestamp) / 1e9
            else:
                dt = (imu.timestamp - relevant_imus[i-1].timestamp) / 1e9

            if dt <= 0:
                continue

            # Get corrected measurements
            acc_corrected, gyro_corrected = self.get_corrected_measurements(imu)

            # Integrate rotation (simple integration)
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

            # Integrate acceleration (remove gravity, integrate twice)
            acc_world = total_rotation @ acc_corrected
            acc_world[2] -= self.gravity  # Remove gravity (assuming Z is up)

            # Simple velocity integration
            velocity_change = acc_world * dt
            total_velocity_change += velocity_change

        # Update state
        self.orientation = total_rotation @ self.orientation
        self.velocity += total_velocity_change

        # Simple position integration (assumes small time steps)
        position_change = self.velocity * dt_total + 0.5 * total_velocity_change * dt_total
        self.position += position_change

        self.prev_timestamp = timestamp

        return total_rotation, position_change

    def get_imu_measurements_in_range(self, start_time: float, end_time: float) -> List[IMUData]:
        """Get IMU measurements within time range"""
        measurements = []
        for imu in self.imu_data:
            if start_time <= imu.timestamp <= end_time:
                measurements.append(imu)
        return measurements

    def reset_integration(self):
        """Reset integration state"""
        self.velocity = np.zeros(3)
        self.position = np.zeros(3)
        self.orientation = np.eye(3)
        self.prev_timestamp = None

    def get_statistics(self) -> Dict:
        """Get IMU processing statistics"""
        if len(self.imu_data) == 0:
            return {'total_measurements': 0}

        timestamps = [imu.timestamp for imu in self.imu_data]
        accelerations = np.array([imu.acceleration for imu in self.imu_data])
        gyroscopes = np.array([imu.gyroscope for imu in self.imu_data])

        return {
            'total_measurements': len(self.imu_data),
            'time_range_seconds': (timestamps[-1] - timestamps[0]) / 1e9,
            'frequency_hz': len(self.imu_data) / ((timestamps[-1] - timestamps[0]) / 1e9),
            'accelerometer': {
                'mean': accelerations.mean(axis=0),
                'std': accelerations.std(axis=0),
                'magnitude_mean': np.linalg.norm(accelerations, axis=1).mean()
            },
            'gyroscope': {
                'mean': gyroscopes.mean(axis=0),
                'std': gyroscopes.std(axis=0),
                'magnitude_mean': np.linalg.norm(gyroscopes, axis=1).mean()
            },
            'calibrated': self.is_calibrated,
            'biases': {
                'accelerometer': self.acc_bias,
                'gyroscope': self.gyro_bias
            }
        }