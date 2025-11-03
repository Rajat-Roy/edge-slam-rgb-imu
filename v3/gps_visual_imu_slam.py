#!/usr/bin/env python3
"""
GPS + Visual + IMU Sensor Fusion SLAM
Combines GPS absolute positioning with Visual-Inertial odometry
"""

import json
import numpy as np
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pathlib import Path
from scipy.spatial.transform import Rotation

class GPSVisualIMUSLAM:
    """SLAM with GPS + Visual + IMU fusion"""

    def __init__(self, camera_matrix, gps_weight=0.3):
        """
        Initialize GPS-Visual-IMU SLAM

        Args:
            camera_matrix: 3x3 camera intrinsic matrix
            gps_weight: Weight for GPS corrections (0-1)
        """
        self.K = camera_matrix
        self.gps_weight = gps_weight

        # ORB feature detector
        self.orb = cv2.ORB_create(nfeatures=500)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # State variables
        self.position = np.zeros(3)  # Current position [x, y, z]
        self.velocity = np.zeros(3)
        self.orientation = np.eye(3)

        # Previous frame
        self.prev_frame = None
        self.prev_keypoints = None
        self.prev_descriptors = None
        self.prev_timestamp = None

        # Trajectory
        self.trajectory = []
        self.timestamps = []
        self.gps_positions = []

        # GPS reference (first GPS point)
        self.gps_ref_lat = None
        self.gps_ref_lon = None
        self.gps_ref_alt = None

    def set_gps_reference(self, lat, lon, alt):
        """Set GPS reference point for local coordinates"""
        self.gps_ref_lat = lat
        self.gps_ref_lon = lon
        self.gps_ref_alt = alt

    def gps_to_local(self, lat, lon, alt):
        """Convert GPS to local XYZ coordinates"""
        if self.gps_ref_lat is None:
            return np.zeros(3)

        # Earth radius
        R = 6371000  # meters

        # Convert to radians
        lat_rad = np.radians(lat)
        lon_rad = np.radians(lon)
        ref_lat_rad = np.radians(self.gps_ref_lat)
        ref_lon_rad = np.radians(self.gps_ref_lon)

        # Local coordinates
        x = R * (lon_rad - ref_lon_rad) * np.cos(ref_lat_rad)
        y = R * (lat_rad - ref_lat_rad)
        z = alt - self.gps_ref_alt

        return np.array([x, y, z])

    def process_frame(self, frame, timestamp, accel, gyro, gps_data=None):
        """
        Process frame with GPS + Visual + IMU fusion

        Args:
            frame: RGB image
            timestamp: Frame timestamp
            accel: Accelerometer [ax, ay, az]
            gyro: Gyroscope [gx, gy, gz]
            gps_data: Optional GPS data [lat, lon, alt]

        Returns:
            Current position [x, y, z]
        """

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect features
        keypoints, descriptors = self.orb.detectAndCompute(gray, None)

        # Initialize on first frame
        if self.prev_frame is None:
            self.prev_frame = gray
            self.prev_keypoints = keypoints
            self.prev_descriptors = descriptors
            self.prev_timestamp = timestamp

            # Set GPS reference if available
            if gps_data is not None:
                self.set_gps_reference(gps_data[0], gps_data[1], gps_data[2])
                gps_local = self.gps_to_local(gps_data[0], gps_data[1], gps_data[2])
                self.position = gps_local
                self.gps_positions.append(gps_local)

            self.trajectory.append(self.position.copy())
            self.timestamps.append(timestamp)
            return self.position

        # Time delta
        dt = timestamp - self.prev_timestamp

        # Visual odometry
        visual_delta = np.zeros(3)

        if descriptors is not None and self.prev_descriptors is not None:
            matches = self.matcher.match(self.prev_descriptors, descriptors)

            if len(matches) >= 8:
                pts1 = np.float32([self.prev_keypoints[m.queryIdx].pt for m in matches])
                pts2 = np.float32([keypoints[m.trainIdx].pt for m in matches])

                E, mask = cv2.findEssentialMat(pts1, pts2, self.K, method=cv2.RANSAC, prob=0.999, threshold=1.0)

                if E is not None:
                    _, R, t, mask = cv2.recoverPose(E, pts1, pts2, self.K, mask=mask)

                    # Transform to world frame
                    t_camera = t.flatten()
                    visual_delta = self.orientation @ t_camera

                    # Update orientation
                    self.orientation = self.orientation @ R

        # IMU integration
        imu_delta = np.zeros(3)

        if gyro is not None:
            # Gyroscope for orientation
            angular_velocity = gyro
            rotation_vector = angular_velocity * dt
            rotation_delta = Rotation.from_rotvec(rotation_vector)
            self.orientation = rotation_delta.as_matrix() @ self.orientation

        if accel is not None:
            # Accelerometer for velocity
            self.velocity += accel * dt
            self.velocity *= 0.95  # Damping
            imu_delta = self.velocity * dt

        # GPS correction
        gps_delta = np.zeros(3)
        gps_local = None

        if gps_data is not None:
            gps_local = self.gps_to_local(gps_data[0], gps_data[1], gps_data[2])
            self.gps_positions.append(gps_local)

            # GPS correction vector
            gps_delta = gps_local - self.position

        # Sensor fusion
        if gps_data is not None:
            # GPS available: use weighted combination
            # GPS provides absolute correction, Visual+IMU provide relative motion
            visual_imu_weight = 1.0 - self.gps_weight

            # Combine visual and IMU
            visual_imu_delta = 0.7 * visual_delta + 0.3 * imu_delta

            # Apply GPS correction with weight
            self.position += visual_imu_delta * visual_imu_weight + gps_delta * self.gps_weight
        else:
            # No GPS: use visual + IMU only
            self.position += 0.7 * visual_delta + 0.3 * imu_delta

        # Store trajectory
        self.trajectory.append(self.position.copy())
        self.timestamps.append(timestamp)

        # Update for next iteration
        self.prev_frame = gray
        self.prev_keypoints = keypoints
        self.prev_descriptors = descriptors
        self.prev_timestamp = timestamp

        return self.position

    def get_trajectory(self):
        """Get trajectory and GPS positions"""
        return (np.array(self.trajectory),
                np.array(self.timestamps),
                np.array(self.gps_positions) if self.gps_positions else None)


def load_all_sensor_data(data_dir):
    """Load RGB, IMU, and GPS data"""

    data_dir = Path(data_dir)

    print("📂 Loading all sensor data...")

    # RGB frames
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

    # IMU data
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

    # GPS data
    gps_file = data_dir / "data" / "gps_data.jsonl"
    gps_data = []

    with open(gps_file, 'r') as f:
        content = f.read()
        current_obj = ""
        brace_count = 0

        for char in content:
            current_obj += char
            if char == '{':
                brace_count += 1
            elif char == '}':
                brace_count -= 1
                if brace_count == 0:
                    try:
                        sample = json.loads(current_obj.strip())
                        # Convert nanosecond timestamp to seconds from session start
                        session_start = 1760437052214 / 1e9
                        timestamp = (sample['timestamp'] / 1e9) - session_start

                        gps_data.append({
                            'timestamp': timestamp,
                            'latitude': sample['latitude'],
                            'longitude': sample['longitude'],
                            'altitude': sample.get('altitude', 0.0),
                            'accuracy': sample.get('accuracy', 0.0)
                        })
                    except:
                        pass
                    current_obj = ""

    # Images directory
    images_dir = data_dir / "data" / "images"
    if not images_dir.exists():
        images_dir = data_dir.parent / "android-slam-logger" / "slam_datasets" / "slam_session_20251014_154732" / "images"

    print(f"✅ Loaded {len(rgb_data)} RGB frames")
    print(f"✅ Loaded {len(imu_data)} IMU samples")
    print(f"✅ Loaded {len(gps_data)} GPS samples")

    return rgb_data, imu_data, gps_data, images_dir


def get_imu_at_timestamp(imu_data, timestamp):
    """Interpolate IMU data at timestamp"""
    idx = np.searchsorted([imu['timestamp'] for imu in imu_data], timestamp)

    if idx == 0:
        return imu_data[0]['accel'], imu_data[0]['gyro']
    elif idx >= len(imu_data):
        return imu_data[-1]['accel'], imu_data[-1]['gyro']

    imu1 = imu_data[idx - 1]
    imu2 = imu_data[idx]

    t1, t2 = imu1['timestamp'], imu2['timestamp']
    if t2 - t1 == 0:
        return imu1['accel'], imu1['gyro']

    alpha = (timestamp - t1) / (t2 - t1)
    accel = imu1['accel'] * (1 - alpha) + imu2['accel'] * alpha
    gyro = imu1['gyro'] * (1 - alpha) + imu2['gyro'] * alpha

    return accel, gyro


def get_gps_at_timestamp(gps_data, timestamp):
    """Get nearest GPS data at timestamp"""
    if not gps_data:
        return None

    # Find closest GPS sample
    timestamps = [gps['timestamp'] for gps in gps_data]
    idx = np.searchsorted(timestamps, timestamp)

    if idx == 0:
        gps = gps_data[0]
    elif idx >= len(gps_data):
        gps = gps_data[-1]
    else:
        # Choose closest
        if abs(timestamps[idx] - timestamp) < abs(timestamps[idx-1] - timestamp):
            gps = gps_data[idx]
        else:
            gps = gps_data[idx-1]

    # Only return GPS if within 5 seconds
    if abs(gps['timestamp'] - timestamp) < 5.0:
        return [gps['latitude'], gps['longitude'], gps['altitude']]

    return None


def run_gps_visual_imu_slam(data_dir, output_dir="output_gps", downsample=5, gps_weight=0.3):
    """Run GPS + Visual + IMU fusion SLAM"""

    output_dir = Path(output_dir)
    output_dir.mkdir(exist_ok=True)

    print("🌍 GPS + Visual + IMU Sensor Fusion SLAM")
    print("=" * 50)

    # Load all data
    rgb_data, imu_data, gps_data, images_dir = load_all_sensor_data(data_dir)

    # Camera matrix
    K = np.array([
        [1344.0, 0, 960.0],
        [0, 756.0, 540.0],
        [0, 0, 1.0]
    ])

    # Initialize SLAM
    slam = GPSVisualIMUSLAM(K, gps_weight=gps_weight)

    print(f"\n🎥 Processing frames (every {downsample}th frame, GPS weight={gps_weight})...")

    processed_count = 0
    gps_used_count = 0

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

        # Get GPS data
        gps_point = get_gps_at_timestamp(gps_data, frame_data['timestamp'])
        if gps_point is not None:
            gps_used_count += 1

        # Process frame
        position = slam.process_frame(frame, frame_data['timestamp'], accel, gyro, gps_point)

        processed_count += 1
        if processed_count % 10 == 0:
            gps_str = f" [GPS]" if gps_point is not None else ""
            print(f"   Processed {processed_count} frames, position: [{position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}]{gps_str}")

    print(f"\n✅ Processed {processed_count} frames")
    print(f"   GPS corrections applied: {gps_used_count} times")

    # Get trajectory
    trajectory, timestamps, gps_positions = slam.get_trajectory()

    # Plot
    plot_gps_visual_imu_trajectory(trajectory, timestamps, gps_positions,
                                   output_dir / "gps_visual_imu_trajectory.png")

    # Save trajectory
    np.savetxt(output_dir / "trajectory.txt",
               np.column_stack([timestamps, trajectory]),
               header="timestamp x y z")

    print(f"\n✅ GPS+Visual+IMU SLAM complete!")
    print(f"   Output directory: {output_dir}")

    return trajectory, timestamps, gps_positions


def plot_gps_visual_imu_trajectory(trajectory, timestamps, gps_positions, save_path):
    """Plot trajectory with GPS points"""

    if len(trajectory) < 2:
        print("   ⚠️  Not enough trajectory points to plot")
        return

    fig = plt.figure(figsize=(16, 12))

    # 3D trajectory
    ax1 = fig.add_subplot(221, projection='3d')

    colors = plt.cm.viridis(np.linspace(0, 1, len(trajectory)))
    for i in range(len(trajectory) - 1):
        ax1.plot(trajectory[i:i+2, 0], trajectory[i:i+2, 1], trajectory[i:i+2, 2],
                color=colors[i], linewidth=2, label='Fused Trajectory' if i == 0 else '')

    # GPS points
    if gps_positions is not None and len(gps_positions) > 0:
        ax1.scatter(gps_positions[:, 0], gps_positions[:, 1], gps_positions[:, 2],
                   c='red', s=80, marker='o', alpha=0.6, label='GPS Points')

    ax1.scatter(trajectory[0, 0], trajectory[0, 1], trajectory[0, 2],
               color='green', s=150, marker='o', label='Start')
    ax1.scatter(trajectory[-1, 0], trajectory[-1, 1], trajectory[-1, 2],
               color='blue', s=150, marker='s', label='End')

    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('GPS + Visual + IMU Fusion - 3D Trajectory')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Top view
    ax2 = fig.add_subplot(222)
    ax2.plot(trajectory[:, 0], trajectory[:, 1], 'b-', linewidth=2, alpha=0.7, label='Fused Path')

    if gps_positions is not None and len(gps_positions) > 0:
        ax2.scatter(gps_positions[:, 0], gps_positions[:, 1], c='red', s=50, alpha=0.6, label='GPS')

    ax2.scatter(trajectory[0, 0], trajectory[0, 1], color='green', s=80, marker='o')
    ax2.scatter(trajectory[-1, 0], trajectory[-1, 1], color='blue', s=80, marker='s')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('Top View - GPS Corrections')
    ax2.legend()
    ax2.axis('equal')
    ax2.grid(True, alpha=0.3)

    # Elevation
    ax3 = fig.add_subplot(223)
    ax3.plot(timestamps, trajectory[:, 2], 'b-', linewidth=2, label='Fused Z')

    if gps_positions is not None and len(gps_positions) > 0:
        gps_times = timestamps[:len(gps_positions)]
        ax3.scatter(gps_times, gps_positions[:, 2], c='red', s=30, alpha=0.6, label='GPS Z')

    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Z (m) - Elevation')
    ax3.set_title('Elevation Profile with GPS')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # Distance
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
    print(f"   Plot saved: {save_path}")

    # Statistics
    total_distance = distances[-1]
    duration = timestamps[-1] - timestamps[0]
    elevation_change = trajectory[-1, 2] - trajectory[0, 2]

    print(f"\n📊 GPS+Visual+IMU Statistics:")
    print(f"   Duration: {duration:.2f} seconds")
    print(f"   Total distance: {total_distance:.2f} meters")
    print(f"   Average speed: {total_distance/duration:.2f} m/s")
    print(f"   Elevation change: {elevation_change:.2f} meters")
    print(f"   GPS samples used: {len(gps_positions) if gps_positions is not None else 0}")
    print(f"   Final position: [{trajectory[-1, 0]:.2f}, {trajectory[-1, 1]:.2f}, {trajectory[-1, 2]:.2f}] m")


if __name__ == "__main__":
    data_dir = "../meeting"

    print("🎯 Starting GPS + Visual + IMU Fusion SLAM")
    print(f"   Data directory: {data_dir}")
    print(f"   Multi-sensor fusion: GPS + Camera + IMU")

    run_gps_visual_imu_slam(data_dir, output_dir="output_gps", downsample=5, gps_weight=0.3)

    print("\n🎉 GPS Fusion SLAM Complete!")
