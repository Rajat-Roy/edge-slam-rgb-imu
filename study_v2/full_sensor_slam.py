#!/usr/bin/env python3
"""
Full Sensor Fusion SLAM
- Camera: Optical flow for translation
- Gyroscope: Rotation rate
- Magnetometer: Absolute heading reference
- Accelerometer: Motion detection / gravity reference
- GPS: Scale calibration
"""

import json
import numpy as np
import cv2
from pathlib import Path
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.ndimage import gaussian_filter1d
from scipy import signal

# Dataset paths
DATASET_ROOT = Path("android-slam-logger/slam_datasets/slam_session_20251119_123341")
DATASET_DIR = DATASET_ROOT / "data"
IMAGES_DIR = DATASET_ROOT / "images"
OUTPUT_DIR = Path("study_v2/full-sensor-slam")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

# Phone orientation
PHONE_ORIENTATION = "landscape"

print("="*70)
print("FULL SENSOR FUSION SLAM")
print("Camera + Gyro + Magnetometer + Accelerometer + GPS")
print("="*70)
print()

# ============================================================================
# Data Loading
# ============================================================================

def load_jsonl(filepath):
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
                data.append(json.loads(json_str))
            except:
                continue
    return data

def load_imu_data():
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

    for key in raw_data:
        raw_data[key] = sorted(raw_data[key], key=lambda x: x['timestamp'])

    return raw_data

# Load all data
print("Loading data...")
camera_data = load_jsonl(DATASET_DIR / 'camera_frames.jsonl')
camera_data.sort(key=lambda x: x['timestamp'])

imu_data = load_imu_data()
accel_data = imu_data[1]
gyro_data = imu_data[4]
mag_data = imu_data[2]

gps_data = load_jsonl(DATASET_DIR / 'gps_data.jsonl')
gps_data = [g for g in gps_data if g.get('latitude', 0) != 0]

print(f"  Camera: {len(camera_data)} frames")
print(f"  Accelerometer: {len(accel_data)} samples")
print(f"  Gyroscope: {len(gyro_data)} samples")
print(f"  Magnetometer: {len(mag_data)} samples")
print(f"  GPS: {len(gps_data)} samples")
print()

# ============================================================================
# Time Synchronization
# ============================================================================

camera_times = np.array([c['timestamp']/1e9 for c in camera_data])
accel_times = np.array([s['timestamp']/1e9 for s in accel_data])
gyro_times = np.array([s['timestamp']/1e9 for s in gyro_data])
mag_times = np.array([s['timestamp']/1e9 for s in mag_data])
gps_times = np.array([s['timestamp']/1e9 for s in gps_data])

time_offset = min(camera_times[0], accel_times[0], gyro_times[0], mag_times[0], gps_times[0])
camera_times -= time_offset
accel_times -= time_offset
gyro_times -= time_offset
mag_times -= time_offset
gps_times -= time_offset

# ============================================================================
# 1. GPS Processing
# ============================================================================

print("="*70)
print("1. GPS PROCESSING")
print("="*70)

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
print(f"  GPS path length: {gps_path_length:.1f}m")

# ============================================================================
# 2. ACCELEROMETER PROCESSING
# ============================================================================

print("="*70)
print("2. ACCELEROMETER PROCESSING")
print("="*70)

accel_values = np.array([s['values'] for s in accel_data])
accel_magnitude = np.linalg.norm(accel_values, axis=1)

# Detect motion (deviation from gravity)
gravity_mean = np.mean(accel_magnitude)
motion_threshold = 1.0  # m/s² deviation from gravity
is_moving = np.abs(accel_magnitude - gravity_mean) > motion_threshold

# Smooth motion detection
motion_smooth = gaussian_filter1d(is_moving.astype(float), sigma=50)
motion_detected = motion_smooth > 0.3

print(f"  Gravity magnitude: {gravity_mean:.2f} m/s²")
print(f"  Motion detected: {100*np.mean(motion_detected):.1f}% of time")

# ============================================================================
# 3. GYROSCOPE PROCESSING
# ============================================================================

print("="*70)
print("3. GYROSCOPE PROCESSING")
print("="*70)

gyro_values = np.array([s['values'] for s in gyro_data])

# Estimate bias from first second
first_sec_mask = gyro_times < 1.0
gyro_bias = np.mean(gyro_values[first_sec_mask], axis=0)
gyro_corrected = gyro_values - gyro_bias

print(f"  Gyro bias: [{gyro_bias[0]:.4f}, {gyro_bias[1]:.4f}, {gyro_bias[2]:.4f}] rad/s")

# Integrate heading
heading_gyro = np.zeros(len(gyro_data))
for i in range(1, len(gyro_data)):
    dt = gyro_times[i] - gyro_times[i-1]
    heading_gyro[i] = heading_gyro[i-1] + gyro_corrected[i, 2] * dt

print(f"  Gyro heading range: {np.degrees(heading_gyro.min()):.1f}° to {np.degrees(heading_gyro.max()):.1f}°")

# ============================================================================
# 4. MAGNETOMETER PROCESSING
# ============================================================================

print("="*70)
print("4. MAGNETOMETER PROCESSING")
print("="*70)

mag_values = np.array([s['values'] for s in mag_data])

# Compute magnetic heading (X-Y plane)
# For landscape mode with camera forward
heading_mag_raw = np.arctan2(mag_values[:, 0], mag_values[:, 1])

# Unwrap to avoid discontinuities
heading_mag_unwrapped = np.unwrap(heading_mag_raw)

# Smooth magnetometer heading
heading_mag_smooth = gaussian_filter1d(heading_mag_unwrapped, sigma=20)

print(f"  Mag heading range: {np.degrees(heading_mag_smooth.min()):.1f}° to {np.degrees(heading_mag_smooth.max()):.1f}°")

# ============================================================================
# 5. SENSOR FUSION: Complementary Filter for Heading
# ============================================================================

print("="*70)
print("5. HEADING FUSION (Gyro + Magnetometer)")
print("="*70)

# Interpolate magnetometer to gyroscope timestamps
heading_mag_interp = np.interp(gyro_times, mag_times, heading_mag_smooth)

# Complementary filter
# High-pass gyro (short-term accuracy) + Low-pass magnetometer (long-term reference)
alpha = 0.98  # Weight for gyroscope (high-frequency)

heading_fused = np.zeros(len(gyro_data))
heading_fused[0] = heading_mag_interp[0]  # Initialize with magnetometer

for i in range(1, len(gyro_data)):
    dt = gyro_times[i] - gyro_times[i-1]

    # Gyroscope prediction
    heading_pred = heading_fused[i-1] + gyro_corrected[i, 2] * dt

    # Magnetometer measurement
    heading_meas = heading_mag_interp[i]

    # Complementary filter fusion
    heading_fused[i] = alpha * heading_pred + (1 - alpha) * heading_meas

# Align to GPS initial direction
if len(gps_smooth) >= 5:
    gps_dx = gps_smooth[4, 0] - gps_smooth[0, 0]
    gps_dy = gps_smooth[4, 1] - gps_smooth[0, 1]
    initial_gps_heading = np.arctan2(gps_dy, gps_dx)
else:
    initial_gps_heading = 0

heading_offset = initial_gps_heading - heading_fused[0]
heading_fused_aligned = heading_fused + heading_offset

print(f"  Fused heading range: {np.degrees(heading_fused_aligned.min()):.1f}° to {np.degrees(heading_fused_aligned.max()):.1f}°")
print(f"  Complementary filter alpha: {alpha}")

# ============================================================================
# 6. OPTICAL FLOW PROCESSING
# ============================================================================

print("="*70)
print("6. OPTICAL FLOW PROCESSING")
print("="*70)

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

flow_times = np.array([f['time'] for f in flow_data])
flow_x = np.array([f['flow_x'] for f in flow_data])
flow_y = np.array([f['flow_y'] for f in flow_data])

print(f"  Flow samples: {len(flow_data)}")

# ============================================================================
# 7. TRAJECTORY INTEGRATION
# ============================================================================

print("="*70)
print("7. TRAJECTORY INTEGRATION")
print("="*70)

# Get heading at flow times
heading_at_flow = np.interp(flow_times, gyro_times, heading_fused_aligned)

# Get motion detection at flow times
motion_at_flow = np.interp(flow_times, accel_times, motion_detected.astype(float)) > 0.5

position = np.zeros((len(flow_data), 3))

for i in range(1, len(flow_data)):
    dt = flow_times[i] - flow_times[i-1]
    h = heading_at_flow[i]

    # Coordinate transform for landscape mode
    if PHONE_ORIENTATION == "portrait":
        flow_forward = -flow_y[i]
        flow_right = flow_x[i]
    else:
        flow_forward = -flow_y[i]
        flow_right = -flow_x[i]

    # Apply motion detection gate (reduce drift when stationary)
    if motion_at_flow[i]:
        motion_scale = 1.0
    else:
        motion_scale = 0.1  # Reduce motion when accelerometer says stationary

    vx = flow_forward * np.cos(h) - flow_right * np.sin(h)
    vy = flow_forward * np.sin(h) + flow_right * np.cos(h)

    position[i, 0] = position[i-1, 0] + vx * dt * motion_scale
    position[i, 1] = position[i-1, 1] + vy * dt * motion_scale

# ============================================================================
# 8. GPS SCALE CALIBRATION
# ============================================================================

print("="*70)
print("8. GPS SCALE CALIBRATION")
print("="*70)

vio_path_length = np.sum(np.linalg.norm(np.diff(position, axis=0), axis=1))
scale = gps_path_length / vio_path_length
position_scaled = position * scale

print(f"  Scale factor: {scale:.4f} m/unit")
print(f"  VIO path (scaled): {vio_path_length * scale:.1f}m")

# ============================================================================
# 9. VISUALIZATION
# ============================================================================

print("="*70)
print("9. CREATING VISUALIZATION")
print("="*70)

fig = plt.figure(figsize=(20, 16))
fig.suptitle('Full Sensor Fusion SLAM - slam_session_20251119_123341\nCamera + Gyro + Magnetometer + Accelerometer + GPS',
             fontsize=14, fontweight='bold')

# 1. Trajectory comparison
ax1 = fig.add_subplot(3, 3, 1)
ax1.plot(gps_smooth[:, 0], gps_smooth[:, 1], 'orange', linewidth=2, label='GPS')
ax1.plot(position_scaled[:, 0], position_scaled[:, 1], 'b-', linewidth=1.5, label='Fused VIO')
ax1.scatter(position_scaled[0, 0], position_scaled[0, 1], c='green', s=100, marker='o', label='Start', zorder=5)
ax1.scatter(position_scaled[-1, 0], position_scaled[-1, 1], c='red', s=100, marker='X', label='End', zorder=5)
ax1.set_xlabel('X - East (m)')
ax1.set_ylabel('Y - North (m)')
ax1.set_title('Trajectory Comparison')
ax1.legend(loc='upper left')
ax1.grid(True, alpha=0.3)
ax1.set_aspect('equal')

# 2. Heading comparison
ax2 = fig.add_subplot(3, 3, 2)
ax2.plot(gyro_times, np.degrees(heading_gyro + heading_offset), 'b-', alpha=0.5, linewidth=0.5, label='Gyro only')
ax2.plot(mag_times, np.degrees(heading_mag_smooth + heading_offset), 'g-', alpha=0.5, linewidth=0.5, label='Mag only')
ax2.plot(gyro_times, np.degrees(heading_fused_aligned), 'r-', linewidth=1, label='Fused')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Heading (°)')
ax2.set_title('Heading: Gyro vs Mag vs Fused')
ax2.legend()
ax2.grid(True, alpha=0.3)

# 3. Accelerometer magnitude
ax3 = fig.add_subplot(3, 3, 3)
ax3.plot(accel_times, accel_magnitude, 'b-', linewidth=0.3, alpha=0.5)
ax3.axhline(y=gravity_mean, color='r', linestyle='--', label=f'Gravity: {gravity_mean:.1f}')
ax3.fill_between(accel_times, 0, 15, where=motion_detected, alpha=0.3, color='green', label='Motion detected')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Accel Magnitude (m/s²)')
ax3.set_title('Accelerometer Motion Detection')
ax3.legend()
ax3.grid(True, alpha=0.3)

# 4. Optical flow magnitude
ax4 = fig.add_subplot(3, 3, 4)
flow_mag = np.sqrt(flow_x**2 + flow_y**2)
ax4.plot(flow_times, flow_mag, 'b-', linewidth=0.5, alpha=0.8)
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Flow Magnitude (pixels)')
ax4.set_title('Optical Flow Magnitude')
ax4.grid(True, alpha=0.3)

# 5. Magnetometer raw
ax5 = fig.add_subplot(3, 3, 5)
ax5.plot(mag_times, mag_values[:, 0], 'r-', linewidth=0.5, alpha=0.7, label='X')
ax5.plot(mag_times, mag_values[:, 1], 'g-', linewidth=0.5, alpha=0.7, label='Y')
ax5.plot(mag_times, mag_values[:, 2], 'b-', linewidth=0.5, alpha=0.7, label='Z')
ax5.set_xlabel('Time (s)')
ax5.set_ylabel('Magnetic Field (µT)')
ax5.set_title('Magnetometer Raw Data')
ax5.legend()
ax5.grid(True, alpha=0.3)

# 6. Gyroscope Z-axis
ax6 = fig.add_subplot(3, 3, 6)
ax6.plot(gyro_times, np.degrees(gyro_corrected[:, 2]), 'b-', linewidth=0.3, alpha=0.7)
ax6.set_xlabel('Time (s)')
ax6.set_ylabel('Angular Velocity (°/s)')
ax6.set_title('Gyroscope Z-axis (Yaw Rate)')
ax6.grid(True, alpha=0.3)

# 7. Position over time
ax7 = fig.add_subplot(3, 3, 7)
ax7.plot(flow_times, position_scaled[:, 0], 'r-', linewidth=1, label='X (East)')
ax7.plot(flow_times, position_scaled[:, 1], 'g-', linewidth=1, label='Y (North)')
gps_x_interp = np.interp(flow_times, gps_times, gps_smooth[:, 0])
gps_y_interp = np.interp(flow_times, gps_times, gps_smooth[:, 1])
ax7.plot(flow_times, gps_x_interp, 'r--', linewidth=1, alpha=0.5, label='GPS X')
ax7.plot(flow_times, gps_y_interp, 'g--', linewidth=1, alpha=0.5, label='GPS Y')
ax7.set_xlabel('Time (s)')
ax7.set_ylabel('Position (m)')
ax7.set_title('Position vs Time')
ax7.legend()
ax7.grid(True, alpha=0.3)

# 8. Error analysis
ax8 = fig.add_subplot(3, 3, 8)
vio_x_at_gps = np.interp(gps_times, flow_times, position_scaled[:, 0])
vio_y_at_gps = np.interp(gps_times, flow_times, position_scaled[:, 1])
position_error = np.sqrt((vio_x_at_gps - gps_smooth[:, 0])**2 + (vio_y_at_gps - gps_smooth[:, 1])**2)
ax8.plot(gps_times, position_error, 'b-', linewidth=1)
ax8.set_xlabel('Time (s)')
ax8.set_ylabel('Position Error (m)')
ax8.set_title(f'Position Error vs GPS (mean: {np.mean(position_error):.1f}m)')
ax8.grid(True, alpha=0.3)

# 9. Statistics
ax9 = fig.add_subplot(3, 3, 9)
ax9.axis('off')

stats_text = f"""FULL SENSOR FUSION STATISTICS

Sensors Used:
  • Camera: Optical flow ({len(flow_data)} samples)
  • Gyroscope: Rotation rate ({len(gyro_data)} samples)
  • Magnetometer: Heading reference ({len(mag_data)} samples)
  • Accelerometer: Motion detection ({len(accel_data)} samples)
  • GPS: Scale calibration ({len(gps_data)} samples)

Heading Fusion:
  • Method: Complementary filter
  • Alpha: {alpha} (gyro weight)
  • Gyro range: {np.degrees(heading_gyro.max() - heading_gyro.min()):.1f}°
  • Mag range: {np.degrees(heading_mag_smooth.max() - heading_mag_smooth.min()):.1f}°
  • Fused range: {np.degrees(heading_fused_aligned.max() - heading_fused_aligned.min()):.1f}°

Motion Detection:
  • Gravity: {gravity_mean:.2f} m/s²
  • Motion threshold: {motion_threshold} m/s²
  • Time moving: {100*np.mean(motion_detected):.1f}%

Trajectory:
  • GPS path: {gps_path_length:.1f}m
  • VIO path: {vio_path_length * scale:.1f}m
  • Scale factor: {scale:.4f} m/unit
  • Mean position error: {np.mean(position_error):.1f}m
  • Max position error: {np.max(position_error):.1f}m

Displacement:
  • GPS: {np.linalg.norm(gps_smooth[-1] - gps_smooth[0]):.1f}m
  • VIO: {np.linalg.norm(position_scaled[-1] - position_scaled[0]):.1f}m
"""

ax9.text(0.05, 0.95, stats_text, transform=ax9.transAxes,
        fontsize=9, verticalalignment='top', fontfamily='monospace',
        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

plt.tight_layout()
output_file = OUTPUT_DIR / 'full_sensor_slam.png'
plt.savefig(output_file, dpi=150, bbox_inches='tight')
plt.close()

print(f"✓ Saved: {output_file}")
print()

print("="*70)
print("FULL SENSOR FUSION COMPLETE")
print("="*70)
print(f"  Path length: {vio_path_length * scale:.1f}m")
print(f"  Mean error: {np.mean(position_error):.1f}m")
print(f"  Sensors fused: Camera + Gyro + Mag + Accel + GPS")
print("="*70)
