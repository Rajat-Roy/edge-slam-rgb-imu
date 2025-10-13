#!/usr/bin/env python3
"""
IMU-only tracking for comparison with VI-SLAM
"""
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent / "src"))

from data_loader import SLAMDataLoader
from imu_processor import IMUProcessor
import numpy as np
import json

class IMUOnlyTracker:
    """Pure inertial navigation using only IMU data"""

    def __init__(self, gravity_magnitude=9.81):
        self.gravity = gravity_magnitude

        # State variables
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = np.eye(3)  # Rotation matrix

        # Trajectory storage
        self.trajectory = []
        self.timestamps = []

        # Noise parameters (realistic for mobile IMU)
        self.process_noise_acc = 0.1  # m/s²
        self.process_noise_gyro = 0.01  # rad/s

    def process_imu_data(self, imu_data_list, start_timestamp=None):
        """Process IMU data for dead reckoning"""
        print(f"Processing {len(imu_data_list)} IMU samples...")

        # Reset state
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = np.eye(3)
        self.trajectory = []
        self.timestamps = []

        prev_timestamp = None

        for i, imu_data in enumerate(imu_data_list):
            timestamp = imu_data.timestamp

            # Skip until start timestamp if specified
            if start_timestamp and timestamp < start_timestamp:
                continue

            # Calculate time step
            if prev_timestamp is None:
                dt = 0.0
            else:
                dt = (timestamp - prev_timestamp) / 1e9  # Convert ns to seconds

            if dt > 0 and dt < 0.1:  # Sanity check for reasonable time steps
                # Get accelerometer and gyroscope readings
                acceleration = imu_data.acceleration
                angular_velocity = imu_data.gyroscope

                # Update orientation using gyroscope
                self._update_orientation(angular_velocity, dt)

                # Update velocity and position using accelerometer
                self._update_position(acceleration, dt)

                # Store trajectory point every 10 samples for visualization
                if i % 10 == 0:
                    self.trajectory.append(self.position.copy())
                    self.timestamps.append(timestamp)

            prev_timestamp = timestamp

        print(f"Generated {len(self.trajectory)} trajectory points")
        return self.trajectory

    def _update_orientation(self, angular_velocity, dt):
        """Update orientation using gyroscope data"""
        # Calculate rotation angle
        angular_velocity_magnitude = np.linalg.norm(angular_velocity)

        if angular_velocity_magnitude > 1e-6:
            # Axis of rotation
            axis = angular_velocity / angular_velocity_magnitude
            angle = angular_velocity_magnitude * dt

            # Rodrigues' rotation formula
            K = np.array([[0, -axis[2], axis[1]],
                         [axis[2], 0, -axis[0]],
                         [-axis[1], axis[0], 0]])

            rotation_matrix = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K
            self.orientation = rotation_matrix @ self.orientation

    def _update_position(self, acceleration, dt):
        """Update position using accelerometer data"""
        # Transform acceleration to world frame
        acceleration_world = self.orientation @ acceleration

        # Remove gravity (assuming it's in the Z direction after orientation correction)
        # This is a simplification - real IMU processing would estimate gravity direction
        acceleration_world[2] -= self.gravity

        # Double integration: acceleration -> velocity -> position
        self.velocity += acceleration_world * dt
        self.position += self.velocity * dt

def main():
    print("=== IMU-Only Tracking Analysis ===")

    # Load data
    data_loader = SLAMDataLoader("data/collected_data")

    # Setup IMU processor
    imu_processor = IMUProcessor()
    imu_processor.load_imu_data(data_loader)
    imu_processor.calibrate_static_bias(duration_seconds=2.0)

    print("\\nIMU Statistics:")
    imu_stats = imu_processor.get_statistics()
    print(f"  Total measurements: {imu_stats['total_measurements']}")
    print(f"  Frequency: {imu_stats['frequency_hz']:.1f} Hz")
    print(f"  Accelerometer magnitude: {imu_stats['accelerometer']['magnitude_mean']:.3f} m/s²")

    # Get gravity direction for orientation correction
    gravity_vector = imu_stats['accelerometer']['mean']
    gravity_magnitude = np.linalg.norm(gravity_vector)
    print(f"  Gravity magnitude: {gravity_magnitude:.3f} m/s²")
    print(f"  Gravity direction: [{gravity_vector[0]:.3f}, {gravity_vector[1]:.3f}, {gravity_vector[2]:.3f}]")

    # Create IMU-only tracker
    imu_tracker = IMUOnlyTracker(gravity_magnitude)

    # Get first frame timestamp to align with VI-SLAM
    first_frame, first_frame_data = data_loader.get_frame(0)
    start_timestamp = first_frame_data['timestamp'] if first_frame_data else None

    print(f"\\nStarting IMU tracking from timestamp: {start_timestamp}")

    # Process IMU data
    imu_trajectory = imu_tracker.process_imu_data(imu_processor.imu_data, start_timestamp)

    if len(imu_trajectory) == 0:
        print("No trajectory generated!")
        return

    # Analyze IMU trajectory
    imu_trajectory = np.array(imu_trajectory)
    print(f"\\nIMU-Only Trajectory Analysis:")
    print(f"  Total points: {len(imu_trajectory)}")

    if len(imu_trajectory) > 1:
        start_pos = imu_trajectory[0]
        end_pos = imu_trajectory[-1]
        total_displacement = end_pos - start_pos

        print(f"  Start position: [{start_pos[0]:.3f}, {start_pos[1]:.3f}, {start_pos[2]:.3f}]")
        print(f"  End position: [{end_pos[0]:.3f}, {end_pos[1]:.3f}, {end_pos[2]:.3f}]")
        print(f"  Net displacement: [{total_displacement[0]:.3f}, {total_displacement[1]:.3f}, {total_displacement[2]:.3f}]")

        # Calculate total path length
        total_distance = 0
        for i in range(1, len(imu_trajectory)):
            distance = np.linalg.norm(imu_trajectory[i] - imu_trajectory[i-1])
            total_distance += distance

        print(f"  Total path length: {total_distance:.3f} meters")

        # Movement range analysis
        movement_range = imu_trajectory.max(axis=0) - imu_trajectory.min(axis=0)
        print(f"  Movement range: X={movement_range[0]:.3f}m, Y={movement_range[1]:.3f}m, Z={movement_range[2]:.3f}m")

    # Load VI-SLAM trajectory for comparison
    print(f"\\n=== Loading VI-SLAM Trajectory for Comparison ===")

    try:
        with open("output/camera_trajectory.json", 'r') as f:
            vi_slam_data = json.load(f)
            vi_slam_trajectory = np.array([point['position'] for point in vi_slam_data['trajectory']])

        print(f"VI-SLAM trajectory points: {len(vi_slam_trajectory)}")

        if len(vi_slam_trajectory) > 1:
            vi_start = vi_slam_trajectory[0]
            vi_end = vi_slam_trajectory[-1]
            vi_displacement = vi_end - vi_start

            print(f"VI-SLAM displacement: [{vi_displacement[0]:.3f}, {vi_displacement[1]:.3f}, {vi_displacement[2]:.3f}]")

            # Compare trajectories
            print(f"\\n=== IMU vs VI-SLAM Comparison ===")
            print(f"IMU-only displacement: [{total_displacement[0]:.3f}, {total_displacement[1]:.3f}, {total_displacement[2]:.3f}]")
            print(f"VI-SLAM displacement:  [{vi_displacement[0]:.3f}, {vi_displacement[1]:.3f}, {vi_displacement[2]:.3f}]")

            # Calculate difference
            displacement_diff = total_displacement - vi_displacement
            print(f"Difference:             [{displacement_diff[0]:.3f}, {displacement_diff[1]:.3f}, {displacement_diff[2]:.3f}]")

    except FileNotFoundError:
        print("VI-SLAM trajectory not found - run camera path extraction first")
        vi_slam_trajectory = None

    # Save IMU-only results
    output_data = {
        'imu_statistics': {
            'total_measurements': imu_stats['total_measurements'],
            'frequency_hz': imu_stats['frequency_hz'],
            'gravity_magnitude': gravity_magnitude,
            'gravity_vector': gravity_vector.tolist()
        },
        'imu_trajectory': imu_trajectory.tolist(),
        'analysis': {
            'total_points': len(imu_trajectory),
            'net_displacement': total_displacement.tolist() if len(imu_trajectory) > 1 else [0, 0, 0],
            'total_distance': total_distance if len(imu_trajectory) > 1 else 0,
            'movement_range': movement_range.tolist() if len(imu_trajectory) > 1 else [0, 0, 0]
        }
    }

    if vi_slam_trajectory is not None:
        output_data['comparison'] = {
            'vi_slam_displacement': vi_displacement.tolist(),
            'displacement_difference': displacement_diff.tolist()
        }

    # Save results
    output_file = Path(__file__).parent / "output" / "imu_only_tracking.json"
    with open(output_file, 'w') as f:
        json.dump(output_data, f, indent=2)

    print(f"\\nIMU-only tracking results saved to: {output_file}")

    # Analysis summary
    print(f"\\n=== Summary ===")
    print(f"IMU-only tracking shows the challenges of pure inertial navigation:")
    print(f"• Double integration of accelerometer data amplifies noise")
    print(f"• Gyroscope drift affects orientation estimation")
    print(f"• No external reference to correct accumulated errors")
    print(f"• Demonstrates why visual-inertial fusion is superior")

if __name__ == "__main__":
    main()