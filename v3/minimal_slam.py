#!/usr/bin/env python3
"""
Minimal Sensor Fusion SLAM for 3D Path Tracking
Fuses RGB visual odometry with IMU measurements for 3D trajectory
No mapping - only path tracking
"""

import json
import numpy as np
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pathlib import Path
from scipy.signal import butter, filtfilt
from scipy.spatial.transform import Rotation

class MinimalSensorFusionSLAM:
    """Minimal SLAM using Visual Odometry + IMU fusion"""

    def __init__(self, camera_matrix):
        """
        Initialize minimal SLAM system

        Args:
            camera_matrix: 3x3 camera intrinsic matrix
        """
        self.K = camera_matrix

        # ORB feature detector
        self.orb = cv2.ORB_create(nfeatures=500)  # Minimal features for speed
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # State variables
        self.position = np.zeros(3)  # Current position
        self.velocity = np.zeros(3)  # Current velocity
        self.orientation = np.eye(3)  # Current rotation matrix

        # Previous frame data
        self.prev_frame = None
        self.prev_keypoints = None
        self.prev_descriptors = None
        self.prev_timestamp = None

        # Trajectory storage
        self.trajectory = []
        self.timestamps = []

        # IMU integration
        self.imu_accel_buffer = []
        self.imu_gyro_buffer = []

    def process_frame(self, frame, timestamp, accel, gyro):
        """
        Process single frame with IMU data

        Args:
            frame: RGB image
            timestamp: Frame timestamp (seconds)
            accel: Accelerometer reading [ax, ay, az]
            gyro: Gyroscope reading [gx, gy, gz]

        Returns:
            Current position [x, y, z]
        """

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect features
        keypoints, descriptors = self.orb.detectAndCompute(gray, None)

        # First frame initialization
        if self.prev_frame is None:
            self.prev_frame = gray
            self.prev_keypoints = keypoints
            self.prev_descriptors = descriptors
            self.prev_timestamp = timestamp
            self.trajectory.append(self.position.copy())
            self.timestamps.append(timestamp)
            return self.position

        # Skip if no features
        if descriptors is None or self.prev_descriptors is None:
            return self.position

        # Match features
        matches = self.matcher.match(self.prev_descriptors, descriptors)

        # Need at least 8 matches for essential matrix
        if len(matches) < 8:
            return self.position

        # Extract matched points
        pts1 = np.float32([self.prev_keypoints[m.queryIdx].pt for m in matches])
        pts2 = np.float32([keypoints[m.trainIdx].pt for m in matches])

        # Estimate essential matrix
        E, mask = cv2.findEssentialMat(pts1, pts2, self.K, method=cv2.RANSAC, prob=0.999, threshold=1.0)

        if E is None:
            return self.position

        # Recover pose
        _, R, t, mask = cv2.recoverPose(E, pts1, pts2, self.K, mask=mask)

        # Time delta
        dt = timestamp - self.prev_timestamp

        # IMU integration for orientation
        if gyro is not None:
            # Integrate gyroscope for rotation
            angular_velocity = gyro
            rotation_vector = angular_velocity * dt
            rotation_delta = Rotation.from_rotvec(rotation_vector)
            self.orientation = rotation_delta.as_matrix() @ self.orientation

        # Visual odometry translation (in camera frame)
        t_camera = t.flatten()

        # Transform to world frame
        t_world = self.orientation @ t_camera

        # IMU acceleration integration
        if accel is not None:
            # Remove gravity (should be handled in preprocessing)
            # Integrate acceleration
            self.velocity += accel * dt

            # Combine with visual odometry
            # Weight visual more heavily as it provides absolute scale
            alpha = 0.7  # Visual weight
            combined_delta = alpha * t_world + (1 - alpha) * self.velocity * dt

            # Update position
            self.position += combined_delta

            # Damping
            self.velocity *= 0.95
        else:
            # Visual-only
            self.position += t_world

        # Store trajectory
        self.trajectory.append(self.position.copy())
        self.timestamps.append(timestamp)

        # Update for next iteration
        self.prev_frame = gray
        self.prev_keypoints = keypoints
        self.prev_descriptors = descriptors
        self.prev_timestamp = timestamp
        self.orientation = self.orientation @ R  # Update orientation

        return self.position

    def get_trajectory(self):
        """Get complete trajectory"""
        return np.array(self.trajectory), np.array(self.timestamps)


def load_sensor_data(data_dir):
    """Load RGB frames and IMU data"""

    data_dir = Path(data_dir)

    print("📂 Loading sensor data...")

    # Load RGB timestamps
    rgb_file = data_dir / "rgb_corrected.txt"
    rgb_data = []
    with open(rgb_file, 'r') as f:
        for line in f:
            parts = line.split()
            if len(parts) >= 3:
                rgb_data.append({
                    'timestamp': float(parts[1]),
                    'filename': parts[2]
                })

    # Load IMU data
    imu_file = data_dir / "imu_corrected.txt"
    imu_data = []
    with open(imu_file, 'r') as f:
        for line in f:
            parts = line.split()
            if len(parts) >= 7:
                imu_data.append({
                    'timestamp': float(parts[0]),
                    'gyro': np.array([float(parts[1]), float(parts[2]), float(parts[3])]),
                    'accel': np.array([float(parts[4]), float(parts[5]), float(parts[6])])
                })

    # Load images directory - try multiple paths
    images_dir = data_dir / "data" / "images"
    if not images_dir.exists():
        images_dir = data_dir.parent / "android-slam-logger" / "slam_datasets" / "slam_session_20251014_154732" / "images"

    print(f"✅ Loaded {len(rgb_data)} RGB frames")
    print(f"✅ Loaded {len(imu_data)} IMU samples")

    return rgb_data, imu_data, images_dir


def get_imu_at_timestamp(imu_data, timestamp):
    """Get interpolated IMU data at specific timestamp"""

    # Find closest IMU samples
    idx = np.searchsorted([imu['timestamp'] for imu in imu_data], timestamp)

    if idx == 0:
        return imu_data[0]['accel'], imu_data[0]['gyro']
    elif idx >= len(imu_data):
        return imu_data[-1]['accel'], imu_data[-1]['gyro']

    # Linear interpolation
    imu1 = imu_data[idx - 1]
    imu2 = imu_data[idx]

    t1, t2 = imu1['timestamp'], imu2['timestamp']
    if t2 - t1 == 0:
        return imu1['accel'], imu1['gyro']

    alpha = (timestamp - t1) / (t2 - t1)

    accel = imu1['accel'] * (1 - alpha) + imu2['accel'] * alpha
    gyro = imu1['gyro'] * (1 - alpha) + imu2['gyro'] * alpha

    return accel, gyro


def run_minimal_slam(data_dir, output_dir="output", downsample=5):
    """
    Run minimal sensor fusion SLAM

    Args:
        data_dir: Directory with sensor data
        output_dir: Output directory for results
        downsample: Process every Nth frame (for speed)
    """

    output_dir = Path(output_dir)
    output_dir.mkdir(exist_ok=True)

    print("🚀 Minimal Sensor Fusion SLAM")
    print("=" * 50)

    # Load data
    rgb_data, imu_data, images_dir = load_sensor_data(data_dir)

    # Camera matrix (estimated for Android phone)
    K = np.array([
        [1344.0, 0, 960.0],
        [0, 756.0, 540.0],
        [0, 0, 1.0]
    ])

    # Initialize SLAM
    slam = MinimalSensorFusionSLAM(K)

    print(f"\n🎥 Processing frames (every {downsample}th frame)...")

    processed_count = 0
    for i in range(0, len(rgb_data), downsample):
        frame_data = rgb_data[i]

        # Load image
        image_path = images_dir / frame_data['filename']
        if not image_path.exists():
            continue

        frame = cv2.imread(str(image_path))
        if frame is None:
            continue

        # Get IMU data
        accel, gyro = get_imu_at_timestamp(imu_data, frame_data['timestamp'])

        # Process frame
        position = slam.process_frame(frame, frame_data['timestamp'], accel, gyro)

        processed_count += 1
        if processed_count % 10 == 0:
            print(f"   Processed {processed_count} frames, position: [{position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}]")

    print(f"\n✅ Processed {processed_count} frames")

    # Get trajectory
    trajectory, timestamps = slam.get_trajectory()

    # Plot results
    plot_trajectory_3d(trajectory, timestamps, output_dir / "minimal_slam_trajectory.png")

    # Save trajectory
    np.savetxt(output_dir / "trajectory.txt",
               np.column_stack([timestamps, trajectory]),
               header="timestamp x y z")

    print(f"\n✅ Minimal SLAM complete!")
    print(f"   Trajectory saved to: {output_dir}")
    print(f"   Total frames processed: {processed_count}")

    return trajectory, timestamps


def plot_trajectory_3d(trajectory, timestamps, save_path):
    """Plot 3D trajectory"""

    # Handle empty or single-point trajectory
    if len(trajectory) < 2:
        print(f"   ⚠️  Not enough trajectory points ({len(trajectory)}) to plot")
        return

    fig = plt.figure(figsize=(15, 12))

    # 3D trajectory
    ax1 = fig.add_subplot(221, projection='3d')

    colors = plt.cm.viridis(np.linspace(0, 1, len(trajectory)))
    for i in range(len(trajectory) - 1):
        ax1.plot(trajectory[i:i+2, 0], trajectory[i:i+2, 1], trajectory[i:i+2, 2],
                color=colors[i], linewidth=2)

    ax1.scatter(trajectory[0, 0], trajectory[0, 1], trajectory[0, 2],
               color='green', s=100, marker='o', label='Start')
    ax1.scatter(trajectory[-1, 0], trajectory[-1, 1], trajectory[-1, 2],
               color='red', s=100, marker='s', label='End')

    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('Minimal Sensor Fusion SLAM - 3D Trajectory')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Top view (XY)
    ax2 = fig.add_subplot(222)
    scatter = ax2.scatter(trajectory[:, 0], trajectory[:, 1], c=timestamps, cmap='viridis', s=5)
    ax2.scatter(trajectory[0, 0], trajectory[0, 1], color='green', s=50, marker='o')
    ax2.scatter(trajectory[-1, 0], trajectory[-1, 1], color='red', s=50, marker='s')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('Top View')
    ax2.axis('equal')
    ax2.grid(True, alpha=0.3)
    plt.colorbar(scatter, ax=ax2, label='Time (s)')

    # Elevation profile
    ax3 = fig.add_subplot(223)
    ax3.plot(timestamps, trajectory[:, 2], 'b-', linewidth=2)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Z (m) - Elevation')
    ax3.set_title('Elevation Profile')
    ax3.grid(True, alpha=0.3)

    # Distance traveled
    ax4 = fig.add_subplot(224)
    distances = np.cumsum(np.sqrt(np.sum(np.diff(trajectory, axis=0)**2, axis=1)))
    distances = np.insert(distances, 0, 0)
    ax4.plot(timestamps, distances, 'g-', linewidth=2)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Distance (m)')
    ax4.set_title('Cumulative Distance Traveled')
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"   Trajectory plot saved: {save_path}")

    # Statistics
    total_distance = distances[-1]
    duration = timestamps[-1] - timestamps[0]
    elevation_change = trajectory[-1, 2] - trajectory[0, 2]

    print(f"\n📊 Trajectory Statistics:")
    print(f"   Duration: {duration:.2f} seconds")
    print(f"   Total distance: {total_distance:.2f} meters")
    print(f"   Average speed: {total_distance/duration:.2f} m/s")
    print(f"   Elevation change: {elevation_change:.2f} meters")
    print(f"   Final position: [{trajectory[-1, 0]:.2f}, {trajectory[-1, 1]:.2f}, {trajectory[-1, 2]:.2f}] m")


if __name__ == "__main__":
    import sys

    # Use meeting data as input
    data_dir = "../meeting"

    print("🎯 Starting Minimal Sensor Fusion SLAM")
    print(f"   Data directory: {data_dir}")
    print(f"   Processing strategy: Downsample frames for speed")

    run_minimal_slam(data_dir, output_dir="output", downsample=5)

    print("\n🎉 SLAM Complete!")
