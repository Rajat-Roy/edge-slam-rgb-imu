#!/usr/bin/env python3
"""
Master Analysis Script for study_v2
Dataset: slam_session_20251119_123341

This script runs all sensor analyses and SLAM pipeline for the new dataset.
"""

import json
import numpy as np
import cv2
from pathlib import Path
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.ndimage import gaussian_filter1d
from scipy import signal
import os

# Dataset paths
DATASET_ROOT = Path("android-slam-logger/slam_datasets/slam_session_20251119_123341")
DATASET_DIR = DATASET_ROOT / "data"
IMAGES_DIR = DATASET_ROOT / "images"
OUTPUT_ROOT = Path("study_v2")

# Phone orientation flag
# "portrait" = phone held vertically (previous dataset) - needs +90° rotation
# "landscape" = phone held horizontally (this dataset) - correct axis already
PHONE_ORIENTATION = "landscape"

print("="*70)
print("SLAM ANALYSIS - Dataset: slam_session_20251119_123341")
print("="*70)
print()

# ============================================================================
# Data Loading Functions
# ============================================================================

def load_jsonl(filepath):
    """Load JSONL file with robust parsing"""
    data = []
    with open(filepath, 'r') as f:
        content = f.read().strip()
        parts = content.split('}\n{')
        json_objects = []
        for i, part in enumerate(parts):
            if i == 0:
                json_objects.append(part + '}')
            elif i == len(parts) - 1:
                json_objects.append('{' + part)
            else:
                json_objects.append('{' + part + '}')

        for json_str in json_objects:
            try:
                obj = json.loads(json_str)
                data.append(obj)
            except:
                continue
    return data

def load_imu_data():
    """Load and synchronize IMU data"""
    raw_data = {1: [], 4: [], 2: []}  # 1=accel, 4=gyro, 2=mag
    imu_file = DATASET_DIR / "imu_data.jsonl"

    with open(imu_file, 'r') as f:
        content = f.read().strip()
        parts = content.split('}\n{')
        json_objects = []
        for i, part in enumerate(parts):
            if i == 0:
                json_objects.append(part + '}')
            elif i == len(parts) - 1:
                json_objects.append('{' + part)
            else:
                json_objects.append('{' + part + '}')

        for json_str in json_objects:
            try:
                data = json.loads(json_str)
                sensor_type = data.get('sensorType')
                if sensor_type in raw_data:
                    raw_data[sensor_type].append({
                        'timestamp': data['timestamp'],
                        'values': np.array(data['values'])
                    })
            except:
                continue

    # Sort by timestamp
    for key in raw_data:
        raw_data[key] = sorted(raw_data[key], key=lambda x: x['timestamp'])

    return raw_data

# ============================================================================
# 1. ACCELEROMETER ANALYSIS
# ============================================================================

def analyze_accelerometer():
    print("="*70)
    print("1. ACCELEROMETER ANALYSIS")
    print("="*70)

    output_dir = OUTPUT_ROOT / "output" / "accel"
    output_dir.mkdir(parents=True, exist_ok=True)

    imu_data = load_imu_data()
    accel_data = imu_data[1]

    times = np.array([s['timestamp']/1e9 for s in accel_data])
    times = times - times[0]
    values = np.array([s['values'] for s in accel_data])

    print(f"  Samples: {len(accel_data)}")
    print(f"  Duration: {times[-1]:.1f}s")
    print(f"  Sampling rate: {len(accel_data)/times[-1]:.1f} Hz")

    # Main analysis plot
    fig, axes = plt.subplots(4, 1, figsize=(14, 12))
    fig.suptitle('Accelerometer Analysis - slam_session_20251119_123341', fontsize=14, fontweight='bold')

    labels = ['X', 'Y', 'Z']
    colors = ['r', 'g', 'b']

    for i in range(3):
        axes[i].plot(times, values[:, i], colors[i], linewidth=0.5, alpha=0.8)
        axes[i].set_ylabel(f'{labels[i]} (m/s²)')
        axes[i].grid(True, alpha=0.3)
        axes[i].set_title(f'{labels[i]}-axis: mean={np.mean(values[:, i]):.2f}, std={np.std(values[:, i]):.2f}')

    # Magnitude
    magnitude = np.linalg.norm(values, axis=1)
    axes[3].plot(times, magnitude, 'purple', linewidth=0.5, alpha=0.8)
    axes[3].axhline(y=9.81, color='k', linestyle='--', alpha=0.5, label='Gravity (9.81)')
    axes[3].set_ylabel('Magnitude (m/s²)')
    axes[3].set_xlabel('Time (s)')
    axes[3].grid(True, alpha=0.3)
    axes[3].legend()
    axes[3].set_title(f'Magnitude: mean={np.mean(magnitude):.2f}')

    plt.tight_layout()
    plt.savefig(output_dir / 'raw_accelerometer_analysis.png', dpi=150, bbox_inches='tight')
    plt.close()

    # Sampling rate analysis
    dt = np.diff(times) * 1000  # ms
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.hist(dt, bins=100, edgecolor='black', alpha=0.7)
    ax.axvline(x=np.median(dt), color='r', linestyle='--', label=f'Median: {np.median(dt):.2f}ms')
    ax.set_xlabel('Sample Interval (ms)')
    ax.set_ylabel('Count')
    ax.set_title('Accelerometer Sampling Rate Distribution')
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(output_dir / 'raw_accelerometer_sampling_rate.png', dpi=150, bbox_inches='tight')
    plt.close()

    print(f"  ✓ Saved to {output_dir}")
    print()

# ============================================================================
# 2. GYROSCOPE ANALYSIS
# ============================================================================

def analyze_gyroscope():
    print("="*70)
    print("2. GYROSCOPE ANALYSIS")
    print("="*70)

    output_dir = OUTPUT_ROOT / "output" / "gyroscope"
    output_dir.mkdir(parents=True, exist_ok=True)

    imu_data = load_imu_data()
    gyro_data = imu_data[4]

    times = np.array([s['timestamp']/1e9 for s in gyro_data])
    times = times - times[0]
    values = np.array([s['values'] for s in gyro_data])

    print(f"  Samples: {len(gyro_data)}")
    print(f"  Duration: {times[-1]:.1f}s")
    print(f"  Sampling rate: {len(gyro_data)/times[-1]:.1f} Hz")

    # Estimate bias from first second
    first_sec_mask = times < 1.0
    bias = np.mean(values[first_sec_mask], axis=0)
    print(f"  Estimated bias: [{bias[0]:.6f}, {bias[1]:.6f}, {bias[2]:.6f}] rad/s")

    # Main analysis plot
    fig, axes = plt.subplots(4, 1, figsize=(14, 12))
    fig.suptitle('Gyroscope Analysis - slam_session_20251119_123341', fontsize=14, fontweight='bold')

    labels = ['X (Roll)', 'Y (Pitch)', 'Z (Yaw)']
    colors = ['r', 'g', 'b']

    for i in range(3):
        axes[i].plot(times, np.degrees(values[:, i]), colors[i], linewidth=0.5, alpha=0.8)
        axes[i].axhline(y=np.degrees(bias[i]), color='k', linestyle='--', alpha=0.5)
        axes[i].set_ylabel(f'{labels[i]} (°/s)')
        axes[i].grid(True, alpha=0.3)
        axes[i].set_title(f'{labels[i]}: mean={np.degrees(np.mean(values[:, i])):.2f}°/s')

    # Integrated heading (Z-axis)
    values_corrected = values - bias
    heading = np.cumsum(values_corrected[:, 2] * np.gradient(times))
    axes[3].plot(times, np.degrees(heading), 'purple', linewidth=1)
    axes[3].set_ylabel('Heading (°)')
    axes[3].set_xlabel('Time (s)')
    axes[3].grid(True, alpha=0.3)
    axes[3].set_title(f'Integrated Heading: total rotation = {np.degrees(heading[-1]):.1f}°')

    plt.tight_layout()
    plt.savefig(output_dir / 'raw_gyroscope_analysis.png', dpi=150, bbox_inches='tight')
    plt.close()

    # Frequency spectrum
    fs = len(gyro_data) / times[-1]
    f, psd = signal.welch(values[:, 2], fs, nperseg=1024)

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.semilogy(f, psd)
    ax.set_xlabel('Frequency (Hz)')
    ax.set_ylabel('PSD')
    ax.set_title('Gyroscope Z-axis Power Spectral Density')
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(output_dir / 'raw_gyroscope_spectrum.png', dpi=150, bbox_inches='tight')
    plt.close()

    print(f"  ✓ Saved to {output_dir}")
    print()

    return bias

# ============================================================================
# 3. MAGNETOMETER ANALYSIS
# ============================================================================

def analyze_magnetometer():
    print("="*70)
    print("3. MAGNETOMETER ANALYSIS")
    print("="*70)

    output_dir = OUTPUT_ROOT / "output" / "magnetometer"
    output_dir.mkdir(parents=True, exist_ok=True)

    imu_data = load_imu_data()
    mag_data = imu_data[2]

    if len(mag_data) == 0:
        print("  No magnetometer data found")
        return

    times = np.array([s['timestamp']/1e9 for s in mag_data])
    times = times - times[0]
    values = np.array([s['values'] for s in mag_data])

    print(f"  Samples: {len(mag_data)}")
    print(f"  Duration: {times[-1]:.1f}s")

    # Main analysis plot
    fig, axes = plt.subplots(4, 1, figsize=(14, 12))
    fig.suptitle('Magnetometer Analysis - slam_session_20251119_123341', fontsize=14, fontweight='bold')

    labels = ['X', 'Y', 'Z']
    colors = ['r', 'g', 'b']

    for i in range(3):
        axes[i].plot(times, values[:, i], colors[i], linewidth=0.5, alpha=0.8)
        axes[i].set_ylabel(f'{labels[i]} (µT)')
        axes[i].grid(True, alpha=0.3)
        axes[i].set_title(f'{labels[i]}-axis: mean={np.mean(values[:, i]):.2f}, std={np.std(values[:, i]):.2f}')

    # Magnitude
    magnitude = np.linalg.norm(values, axis=1)
    axes[3].plot(times, magnitude, 'purple', linewidth=0.5, alpha=0.8)
    axes[3].set_ylabel('Magnitude (µT)')
    axes[3].set_xlabel('Time (s)')
    axes[3].grid(True, alpha=0.3)
    axes[3].set_title(f'Magnitude: mean={np.mean(magnitude):.2f}')

    plt.tight_layout()
    plt.savefig(output_dir / 'raw_magnetometer_analysis.png', dpi=150, bbox_inches='tight')
    plt.close()

    # Heading polar plot
    heading = np.arctan2(values[:, 1], values[:, 0])

    fig, ax = plt.subplots(figsize=(8, 8), subplot_kw=dict(projection='polar'))
    ax.scatter(heading, magnitude, c=times, cmap='viridis', s=1, alpha=0.5)
    ax.set_title('Magnetometer Heading Polar Plot')
    plt.savefig(output_dir / 'magnetometer_heading_polar.png', dpi=150, bbox_inches='tight')
    plt.close()

    print(f"  ✓ Saved to {output_dir}")
    print()

# ============================================================================
# 4. GPS ANALYSIS
# ============================================================================

def analyze_gps():
    print("="*70)
    print("4. GPS ANALYSIS")
    print("="*70)

    output_dir = OUTPUT_ROOT / "output" / "gps"
    output_dir.mkdir(parents=True, exist_ok=True)

    gps_data = load_jsonl(DATASET_DIR / 'gps_data.jsonl')
    gps_data = [g for g in gps_data if g.get('latitude', 0) != 0 and g.get('longitude', 0) != 0]

    print(f"  Samples: {len(gps_data)}")

    if len(gps_data) == 0:
        print("  No GPS data found")
        return None, None

    times = np.array([s['timestamp']/1e9 for s in gps_data])
    times = times - times[0]

    # Convert to local meters
    lat0 = gps_data[0]['latitude']
    lon0 = gps_data[0]['longitude']
    alt0 = gps_data[0].get('altitude', 0)

    meters_per_deg_lat = 111320
    meters_per_deg_lon = 111320 * np.cos(np.radians(lat0))

    positions = []
    for sample in gps_data:
        x = (sample['longitude'] - lon0) * meters_per_deg_lon
        y = (sample['latitude'] - lat0) * meters_per_deg_lat
        z = sample.get('altitude', alt0) - alt0
        positions.append([x, y, z])

    positions = np.array(positions)

    # Smooth GPS
    sigma = 2
    positions_smooth = np.zeros_like(positions)
    for i in range(3):
        positions_smooth[:, i] = gaussian_filter1d(positions[:, i], sigma)

    # Calculate path length
    path_length_raw = np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1))
    path_length_smooth = np.sum(np.linalg.norm(np.diff(positions_smooth, axis=0), axis=1))

    print(f"  Duration: {times[-1]:.1f}s")
    print(f"  Path length (raw): {path_length_raw:.1f}m")
    print(f"  Path length (smoothed): {path_length_smooth:.1f}m")

    # Main analysis plot
    fig = plt.figure(figsize=(16, 10))
    fig.suptitle('GPS Analysis - slam_session_20251119_123341', fontsize=14, fontweight='bold')

    # 2D trajectory
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.plot(positions[:, 0], positions[:, 1], 'b-', alpha=0.3, label='Raw')
    ax1.plot(positions_smooth[:, 0], positions_smooth[:, 1], 'orange', linewidth=2, label='Smoothed')
    ax1.scatter(positions_smooth[0, 0], positions_smooth[0, 1], c='green', s=100, marker='o', label='Start', zorder=5)
    ax1.scatter(positions_smooth[-1, 0], positions_smooth[-1, 1], c='red', s=100, marker='X', label='End', zorder=5)
    ax1.set_xlabel('X - East (m)')
    ax1.set_ylabel('Y - North (m)')
    ax1.set_title('GPS Trajectory')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect('equal')

    # Position over time
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(times, positions[:, 0], 'r-', alpha=0.5, label='X (East)')
    ax2.plot(times, positions[:, 1], 'g-', alpha=0.5, label='Y (North)')
    ax2.plot(times, positions[:, 2], 'b-', alpha=0.5, label='Z (Alt)')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Position (m)')
    ax2.set_title('GPS Position vs Time')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # Accuracy distribution
    accuracies = [s.get('accuracy', 0) for s in gps_data if s.get('accuracy', 0) > 0]
    ax3 = fig.add_subplot(2, 2, 3)
    if len(accuracies) > 0:
        ax3.hist(accuracies, bins=30, edgecolor='black', alpha=0.7)
        ax3.axvline(x=np.mean(accuracies), color='r', linestyle='--', label=f'Mean: {np.mean(accuracies):.1f}m')
        ax3.set_xlabel('Accuracy (m)')
        ax3.set_ylabel('Count')
        ax3.set_title('GPS Accuracy Distribution')
        ax3.legend()
        ax3.grid(True, alpha=0.3)

    # 3D trajectory
    ax4 = fig.add_subplot(2, 2, 4, projection='3d')
    ax4.plot(positions_smooth[:, 0], positions_smooth[:, 1], positions_smooth[:, 2], 'b-', linewidth=2)
    ax4.set_xlabel('X (m)')
    ax4.set_ylabel('Y (m)')
    ax4.set_zlabel('Z (m)')
    ax4.set_title('3D GPS Trajectory')

    plt.tight_layout()
    plt.savefig(output_dir / 'raw_gps_analysis.png', dpi=150, bbox_inches='tight')
    plt.close()

    print(f"  ✓ Saved to {output_dir}")
    print()

    return times, positions_smooth

# ============================================================================
# 5. CAMERA ANALYSIS
# ============================================================================

def analyze_camera():
    print("="*70)
    print("5. CAMERA ANALYSIS")
    print("="*70)

    output_dir = OUTPUT_ROOT / "output" / "camera"
    output_dir.mkdir(parents=True, exist_ok=True)

    camera_data = load_jsonl(DATASET_DIR / 'camera_frames.jsonl')
    camera_data.sort(key=lambda x: x['timestamp'])

    print(f"  Frames: {len(camera_data)}")

    times = np.array([c['timestamp']/1e9 for c in camera_data])
    times = times - times[0]

    print(f"  Duration: {times[-1]:.1f}s")
    print(f"  Frame rate: {len(camera_data)/times[-1]:.1f} FPS")

    # Analyze sample frames
    sample_indices = np.linspace(0, len(camera_data)-1, 6, dtype=int)

    brightness = []
    sharpness = []

    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    fig.suptitle('Camera Sample Frames - slam_session_20251119_123341', fontsize=14, fontweight='bold')

    for idx, sample_idx in enumerate(sample_indices):
        frame = camera_data[sample_idx]
        filename = frame.get('fileName', frame.get('filename', ''))
        img_path = IMAGES_DIR / filename

        if img_path.exists():
            img = cv2.imread(str(img_path))
            if img is not None:
                # Rotate based on phone orientation
                if PHONE_ORIENTATION == "portrait":
                    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
                # landscape = no rotation needed
                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

                # Metrics
                bright = np.mean(gray)
                sharp = cv2.Laplacian(gray, cv2.CV_64F).var()
                brightness.append(bright)
                sharpness.append(sharp)

                # Plot
                ax = axes[idx // 3, idx % 3]
                ax.imshow(img_rgb)
                ax.set_title(f'Frame {sample_idx}\nt={times[sample_idx]:.1f}s\nB={bright:.0f}, S={sharp:.0f}')
                ax.axis('off')

    plt.tight_layout()
    plt.savefig(output_dir / 'camera_quality_analysis.png', dpi=150, bbox_inches='tight')
    plt.close()

    # Timing analysis
    dt = np.diff(times) * 1000  # ms

    fig, axes = plt.subplots(2, 1, figsize=(14, 8))
    fig.suptitle('Camera Timing Analysis', fontsize=14, fontweight='bold')

    axes[0].plot(times[1:], dt, 'b-', linewidth=0.5, alpha=0.8)
    axes[0].axhline(y=np.median(dt), color='r', linestyle='--', label=f'Median: {np.median(dt):.2f}ms')
    axes[0].set_ylabel('Frame Interval (ms)')
    axes[0].set_xlabel('Time (s)')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].hist(dt, bins=100, edgecolor='black', alpha=0.7)
    axes[1].axvline(x=np.median(dt), color='r', linestyle='--')
    axes[1].set_xlabel('Frame Interval (ms)')
    axes[1].set_ylabel('Count')
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_dir / 'camera_timing_analysis.png', dpi=150, bbox_inches='tight')
    plt.close()

    print(f"  ✓ Saved to {output_dir}")
    print()

# ============================================================================
# 6. OPTICAL FLOW + VIO + SLAM
# ============================================================================

def run_slam_pipeline(gyro_bias):
    print("="*70)
    print("6. SLAM PIPELINE (Optical Flow + Gyro + GPS Scale)")
    print("="*70)

    output_dir = OUTPUT_ROOT / "optical-flow-gyro"
    output_dir.mkdir(parents=True, exist_ok=True)

    # Load data
    camera_data = load_jsonl(DATASET_DIR / 'camera_frames.jsonl')
    camera_data.sort(key=lambda x: x['timestamp'])

    imu_data = load_imu_data()
    gyro_data = imu_data[4]

    gps_data = load_jsonl(DATASET_DIR / 'gps_data.jsonl')
    gps_data = [g for g in gps_data if g.get('latitude', 0) != 0 and g.get('longitude', 0) != 0]

    # Time synchronization
    camera_times = np.array([c['timestamp']/1e9 for c in camera_data])
    gyro_times = np.array([s['timestamp']/1e9 for s in gyro_data])
    gps_times = np.array([s['timestamp']/1e9 for s in gps_data])

    time_offset = min(camera_times[0], gyro_times[0], gps_times[0])
    camera_times -= time_offset
    gyro_times -= time_offset
    gps_times -= time_offset

    # Gyroscope heading
    print("  Computing gyroscope heading...")
    gyro_values = np.array([s['values'] for s in gyro_data])
    gyro_corrected = gyro_values - gyro_bias

    heading = np.zeros(len(gyro_data))
    for i in range(1, len(gyro_data)):
        dt = gyro_times[i] - gyro_times[i-1]
        heading[i] = heading[i-1] + gyro_corrected[i, 2] * dt

    print(f"    Heading range: {np.degrees(heading.min()):.1f}° to {np.degrees(heading.max()):.1f}°")

    # GPS positions
    lat0 = gps_data[0]['latitude']
    lon0 = gps_data[0]['longitude']
    meters_per_deg_lat = 111320
    meters_per_deg_lon = 111320 * np.cos(np.radians(lat0))

    gps_positions = []
    for sample in gps_data:
        x = (sample['longitude'] - lon0) * meters_per_deg_lon
        y = (sample['latitude'] - lat0) * meters_per_deg_lat
        z = sample.get('altitude', gps_data[0].get('altitude', 0)) - gps_data[0].get('altitude', 0)
        gps_positions.append([x, y, z])
    gps_positions = np.array(gps_positions)

    # Smooth GPS
    sigma = 2
    gps_smooth = np.zeros_like(gps_positions)
    for i in range(3):
        gps_smooth[:, i] = gaussian_filter1d(gps_positions[:, i], sigma)

    gps_path_length = np.sum(np.linalg.norm(np.diff(gps_smooth, axis=0), axis=1))
    print(f"    GPS path length (smoothed): {gps_path_length:.1f}m")

    # Optical flow
    print("  Computing optical flow...")
    SAMPLE_INTERVAL = 10
    sample_indices = list(range(0, len(camera_data) - 1, SAMPLE_INTERVAL))

    flow_data = []

    for idx in sample_indices:
        if idx >= len(camera_data) - 1:
            break

        img1_filename = camera_data[idx].get('fileName', camera_data[idx].get('filename', ''))
        img2_filename = camera_data[idx+1].get('fileName', camera_data[idx+1].get('filename', ''))

        img1_path = IMAGES_DIR / img1_filename
        img2_path = IMAGES_DIR / img2_filename

        if not img1_path.exists() or not img2_path.exists():
            continue

        img1 = cv2.imread(str(img1_path))
        img2 = cv2.imread(str(img2_path))

        if img1 is None or img2 is None:
            continue

        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

        flow = cv2.calcOpticalFlowFarneback(
            gray1, gray2, None,
            pyr_scale=0.5, levels=3, winsize=15,
            iterations=3, poly_n=5, poly_sigma=1.2, flags=0
        )

        flow_data.append({
            'time': camera_times[idx],
            'flow_x': np.mean(flow[:, :, 0]),
            'flow_y': np.mean(flow[:, :, 1])
        })

    print(f"    Flow samples: {len(flow_data)}")

    flow_times = np.array([f['time'] for f in flow_data])
    flow_x = np.array([f['flow_x'] for f in flow_data])
    flow_y = np.array([f['flow_y'] for f in flow_data])

    # Integrate VIO trajectory
    print("  Integrating VIO trajectory...")

    # Compute initial heading from GPS (first few points)
    if len(gps_smooth) >= 5:
        gps_dx = gps_smooth[4, 0] - gps_smooth[0, 0]
        gps_dy = gps_smooth[4, 1] - gps_smooth[0, 1]
        initial_gps_heading = np.arctan2(gps_dy, gps_dx)
    else:
        initial_gps_heading = 0

    # Offset gyro heading to match GPS initial direction
    heading_offset = initial_gps_heading - heading[0]
    heading_aligned = heading + heading_offset

    heading_at_flow = np.interp(flow_times, gyro_times, heading_aligned)

    position = np.zeros((len(flow_data), 3))

    for i in range(1, len(flow_data)):
        dt = flow_times[i] - flow_times[i-1]
        h = heading_at_flow[i]

        # Coordinate transform depends on phone orientation
        if PHONE_ORIENTATION == "portrait":
            # Portrait: phone held vertically
            flow_forward = -flow_y[i]
            flow_right = flow_x[i]
        else:
            # Landscape: camera forward, volume buttons on top
            # Forward motion causes pixels to move up (negative Y)
            # Right motion causes pixels to move left (negative X)
            flow_forward = -flow_y[i]
            flow_right = -flow_x[i]

        vx = flow_forward * np.cos(h) - flow_right * np.sin(h)
        vy = flow_forward * np.sin(h) + flow_right * np.cos(h)

        position[i, 0] = position[i-1, 0] + vx * dt
        position[i, 1] = position[i-1, 1] + vy * dt

    vio_path_length = np.sum(np.linalg.norm(np.diff(position, axis=0), axis=1))

    # Scale calibration
    scale = gps_path_length / vio_path_length
    position_scaled = position * scale

    print(f"    Scale factor: {scale:.6f} m/unit")
    print(f"    VIO path (scaled): {vio_path_length * scale:.1f}m")

    # Visualization
    fig = plt.figure(figsize=(16, 12))
    fig.suptitle('SLAM Analysis - slam_session_20251119_123341', fontsize=14, fontweight='bold')

    # Trajectory comparison
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.plot(gps_smooth[:, 0], gps_smooth[:, 1], 'orange', linewidth=2, label='GPS (smoothed)')
    ax1.plot(position_scaled[:, 0], position_scaled[:, 1], 'b-', linewidth=2, label='VIO (scaled)')
    ax1.scatter(position_scaled[0, 0], position_scaled[0, 1], c='green', s=100, marker='o', label='Start', zorder=5)
    ax1.scatter(position_scaled[-1, 0], position_scaled[-1, 1], c='red', s=100, marker='X', label='End', zorder=5)
    ax1.set_xlabel('X - East (m)')
    ax1.set_ylabel('Y - North (m)')
    ax1.set_title('Trajectory Comparison')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect('equal')

    # Heading over time
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(gyro_times, np.degrees(heading), 'b-', linewidth=1)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Heading (°)')
    ax2.set_title(f'Gyroscope Heading (total: {np.degrees(heading[-1] - heading[0]):.1f}°)')
    ax2.grid(True, alpha=0.3)

    # Optical flow magnitude
    ax3 = fig.add_subplot(2, 2, 3)
    flow_mag = np.sqrt(flow_x**2 + flow_y**2)
    ax3.plot(flow_times, flow_mag, 'b-', linewidth=0.5, alpha=0.8)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Flow Magnitude (pixels)')
    ax3.set_title('Optical Flow Magnitude')
    ax3.grid(True, alpha=0.3)

    # Statistics
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.axis('off')

    stats_text = f"""SLAM Statistics

Dataset: slam_session_20251119_123341

Motion Estimation:
  • Frames: {len(camera_data)}
  • Flow samples: {len(flow_data)}
  • Heading range: {np.degrees(heading.max() - heading.min()):.1f}°

Scale Calibration:
  • GPS path: {gps_path_length:.1f}m
  • VIO path (scaled): {vio_path_length * scale:.1f}m
  • Scale factor: {scale:.6f} m/unit
  • Match: {100 * vio_path_length * scale / gps_path_length:.1f}%

Displacement:
  • GPS: {np.linalg.norm(gps_smooth[-1] - gps_smooth[0]):.1f}m
  • VIO: {np.linalg.norm(position_scaled[-1] - position_scaled[0]):.1f}m
"""

    ax4.text(0.05, 0.95, stats_text, transform=ax4.transAxes,
            fontsize=10, verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    plt.tight_layout()
    plt.savefig(output_dir / 'slam_analysis.png', dpi=150, bbox_inches='tight')
    plt.close()

    print(f"  ✓ Saved to {output_dir}")
    print()

# ============================================================================
# MAIN
# ============================================================================

if __name__ == "__main__":
    # Run all analyses
    analyze_accelerometer()
    gyro_bias = analyze_gyroscope()
    analyze_magnetometer()
    analyze_gps()
    analyze_camera()

    if gyro_bias is not None:
        run_slam_pipeline(gyro_bias)

    print("="*70)
    print("ALL ANALYSES COMPLETE")
    print(f"Results saved to: {OUTPUT_ROOT}")
    print("="*70)
