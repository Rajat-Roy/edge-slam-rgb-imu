#!/usr/bin/env python3
"""
SLAM without Magnetometer
- Camera: Optical flow for translation
- Gyroscope: Rotation (no magnetometer fusion)
- Accelerometer: Motion detection
- GPS: Scale calibration + initial heading
"""

import json
import numpy as np
import cv2
from pathlib import Path
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter1d

# Dataset paths
DATASET_ROOT = Path("android-slam-logger/slam_datasets/slam_session_20251119_123341")
DATASET_DIR = DATASET_ROOT / "data"
IMAGES_DIR = DATASET_ROOT / "images"
OUTPUT_DIR = Path("study_v2/slam-no-mag")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

# Phone orientation
PHONE_ORIENTATION = "landscape"

print("="*70)
print("SLAM WITHOUT MAGNETOMETER")
print("Camera + Gyro + Accelerometer + GPS")
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
    raw_data = {1: [], 4: []}  # 1=accel, 4=gyro
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

gps_data = load_jsonl(DATASET_DIR / 'gps_data.jsonl')
gps_data = [g for g in gps_data if g.get('latitude', 0) != 0]

print(f"  Camera: {len(camera_data)} frames")
print(f"  Accelerometer: {len(accel_data)} samples")
print(f"  Gyroscope: {len(gyro_data)} samples")
print(f"  GPS: {len(gps_data)} samples")
print(f"  Magnetometer: NOT USED")
print()

# ============================================================================
# Time Synchronization
# ============================================================================

camera_times = np.array([c['timestamp']/1e9 for c in camera_data])
accel_times = np.array([s['timestamp']/1e9 for s in accel_data])
gyro_times = np.array([s['timestamp']/1e9 for s in gyro_data])
gps_times = np.array([s['timestamp']/1e9 for s in gps_data])

time_offset = min(camera_times[0], accel_times[0], gyro_times[0], gps_times[0])
camera_times -= time_offset
accel_times -= time_offset
gyro_times -= time_offset
gps_times -= time_offset

# ============================================================================
# 1. GPS PROCESSING
# ============================================================================

print("1. GPS Processing...")

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
print(f"   GPS path length: {gps_path_length:.1f}m")

# ============================================================================
# 2. ACCELEROMETER - Motion Detection
# ============================================================================

print("2. Accelerometer Motion Detection...")

accel_values = np.array([s['values'] for s in accel_data])
accel_magnitude = np.linalg.norm(accel_values, axis=1)

gravity_mean = np.mean(accel_magnitude)
motion_threshold = 1.0
is_moving = np.abs(accel_magnitude - gravity_mean) > motion_threshold
motion_smooth = gaussian_filter1d(is_moving.astype(float), sigma=50)
motion_detected = motion_smooth > 0.3

print(f"   Motion detected: {100*np.mean(motion_detected):.1f}% of time")

# ============================================================================
# 3. GYROSCOPE - Heading Integration
# ============================================================================

print("3. Gyroscope Heading...")

gyro_values = np.array([s['values'] for s in gyro_data])

# Bias correction
first_sec_mask = gyro_times < 1.0
gyro_bias = np.mean(gyro_values[first_sec_mask], axis=0)
gyro_corrected = gyro_values - gyro_bias

# Integrate heading
heading = np.zeros(len(gyro_data))
for i in range(1, len(gyro_data)):
    dt = gyro_times[i] - gyro_times[i-1]
    heading[i] = heading[i-1] + gyro_corrected[i, 2] * dt

# Align to GPS initial direction
if len(gps_smooth) >= 5:
    gps_dx = gps_smooth[4, 0] - gps_smooth[0, 0]
    gps_dy = gps_smooth[4, 1] - gps_smooth[0, 1]
    initial_gps_heading = np.arctan2(gps_dy, gps_dx)
else:
    initial_gps_heading = 0

heading_offset = initial_gps_heading - heading[0]
heading_aligned = heading + heading_offset

print(f"   Heading range: {np.degrees(heading_aligned.min()):.1f}° to {np.degrees(heading_aligned.max()):.1f}°")

# ============================================================================
# 4. OPTICAL FLOW
# ============================================================================

print("4. Optical Flow...")

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

print(f"   Flow samples: {len(flow_data)}")

# ============================================================================
# 5. TRAJECTORY INTEGRATION
# ============================================================================

print("5. Trajectory Integration...")

heading_at_flow = np.interp(flow_times, gyro_times, heading_aligned)
motion_at_flow = np.interp(flow_times, accel_times, motion_detected.astype(float)) > 0.5

position = np.zeros((len(flow_data), 3))

for i in range(1, len(flow_data)):
    dt = flow_times[i] - flow_times[i-1]
    h = heading_at_flow[i]

    # Landscape mode
    flow_forward = -flow_y[i]
    flow_right = -flow_x[i]

    # Motion gating
    motion_scale = 1.0 if motion_at_flow[i] else 0.1

    vx = flow_forward * np.cos(h) - flow_right * np.sin(h)
    vy = flow_forward * np.sin(h) + flow_right * np.cos(h)

    position[i, 0] = position[i-1, 0] + vx * dt * motion_scale
    position[i, 1] = position[i-1, 1] + vy * dt * motion_scale

# ============================================================================
# 6. GPS SCALE CALIBRATION
# ============================================================================

print("6. GPS Scale Calibration...")

vio_path_length = np.sum(np.linalg.norm(np.diff(position, axis=0), axis=1))
scale = gps_path_length / vio_path_length
position_scaled = position * scale

print(f"   Scale factor: {scale:.4f} m/unit")
print(f"   VIO path: {vio_path_length * scale:.1f}m")

# ============================================================================
# 7. ERROR ANALYSIS
# ============================================================================

vio_x_at_gps = np.interp(gps_times, flow_times, position_scaled[:, 0])
vio_y_at_gps = np.interp(gps_times, flow_times, position_scaled[:, 1])
position_error = np.sqrt((vio_x_at_gps - gps_smooth[:, 0])**2 + (vio_y_at_gps - gps_smooth[:, 1])**2)

print(f"   Mean error: {np.mean(position_error):.1f}m")
print(f"   Max error: {np.max(position_error):.1f}m")

# ============================================================================
# 8. VISUALIZATION
# ============================================================================

print("7. Creating Visualization...")

fig = plt.figure(figsize=(18, 12))
fig.suptitle('SLAM without Magnetometer - slam_session_20251119_123341\nCamera + Gyro + Accelerometer + GPS',
             fontsize=14, fontweight='bold')

# 1. Trajectory comparison
ax1 = fig.add_subplot(2, 3, 1)
ax1.plot(gps_smooth[:, 0], gps_smooth[:, 1], 'orange', linewidth=2, label='GPS')
ax1.plot(position_scaled[:, 0], position_scaled[:, 1], 'b-', linewidth=1.5, label='VIO')
ax1.scatter(position_scaled[0, 0], position_scaled[0, 1], c='green', s=100, marker='o', label='Start', zorder=5)
ax1.scatter(position_scaled[-1, 0], position_scaled[-1, 1], c='red', s=100, marker='X', label='End', zorder=5)
ax1.set_xlabel('X - East (m)')
ax1.set_ylabel('Y - North (m)')
ax1.set_title('Trajectory Comparison')
ax1.legend(loc='upper left')
ax1.grid(True, alpha=0.3)
ax1.set_aspect('equal')

# 2. Gyroscope heading
ax2 = fig.add_subplot(2, 3, 2)
ax2.plot(gyro_times, np.degrees(heading_aligned), 'b-', linewidth=1)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Heading (°)')
ax2.set_title(f'Gyroscope Heading (total: {np.degrees(heading_aligned[-1] - heading_aligned[0]):.1f}°)')
ax2.grid(True, alpha=0.3)

# 3. Motion detection
ax3 = fig.add_subplot(2, 3, 3)
ax3.plot(accel_times, accel_magnitude, 'b-', linewidth=0.3, alpha=0.5)
ax3.axhline(y=gravity_mean, color='r', linestyle='--', label=f'Gravity: {gravity_mean:.1f}')
ax3.fill_between(accel_times, 0, 15, where=motion_detected, alpha=0.3, color='green', label='Motion')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Accel (m/s²)')
ax3.set_title(f'Motion Detection ({100*np.mean(motion_detected):.0f}% moving)')
ax3.legend()
ax3.grid(True, alpha=0.3)

# 4. Optical flow
ax4 = fig.add_subplot(2, 3, 4)
flow_mag = np.sqrt(flow_x**2 + flow_y**2)
ax4.plot(flow_times, flow_mag, 'b-', linewidth=0.5)
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Flow (pixels)')
ax4.set_title('Optical Flow Magnitude')
ax4.grid(True, alpha=0.3)

# 5. Position error
ax5 = fig.add_subplot(2, 3, 5)
ax5.plot(gps_times, position_error, 'b-', linewidth=1)
ax5.axhline(y=np.mean(position_error), color='r', linestyle='--', label=f'Mean: {np.mean(position_error):.1f}m')
ax5.set_xlabel('Time (s)')
ax5.set_ylabel('Error (m)')
ax5.set_title('Position Error vs GPS')
ax5.legend()
ax5.grid(True, alpha=0.3)

# 6. Statistics
ax6 = fig.add_subplot(2, 3, 6)
ax6.axis('off')

stats_text = f"""SLAM WITHOUT MAGNETOMETER

Sensors:
  • Camera: {len(flow_data)} flow samples
  • Gyroscope: {len(gyro_data)} samples
  • Accelerometer: {len(accel_data)} samples
  • GPS: {len(gps_data)} samples
  • Magnetometer: NOT USED

Heading:
  • Source: Gyroscope only
  • Bias: [{gyro_bias[0]:.4f}, {gyro_bias[1]:.4f}, {gyro_bias[2]:.4f}]
  • Range: {np.degrees(heading_aligned.max() - heading_aligned.min()):.1f}°

Motion Detection:
  • Gravity: {gravity_mean:.2f} m/s²
  • Moving: {100*np.mean(motion_detected):.1f}%

Trajectory:
  • GPS path: {gps_path_length:.1f}m
  • VIO path: {vio_path_length * scale:.1f}m
  • Scale: {scale:.4f} m/unit

Error:
  • Mean: {np.mean(position_error):.1f}m
  • Max: {np.max(position_error):.1f}m
  • Final: {position_error[-1]:.1f}m

Displacement:
  • GPS: {np.linalg.norm(gps_smooth[-1] - gps_smooth[0]):.1f}m
  • VIO: {np.linalg.norm(position_scaled[-1] - position_scaled[0]):.1f}m
"""

ax6.text(0.05, 0.95, stats_text, transform=ax6.transAxes,
        fontsize=10, verticalalignment='top', fontfamily='monospace',
        bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.5))

plt.tight_layout()
output_file = OUTPUT_DIR / 'slam_no_magnetometer.png'
plt.savefig(output_file, dpi=150, bbox_inches='tight')
plt.close()

print(f"   Saved: {output_file}")
print()

print("="*70)
print("SLAM WITHOUT MAGNETOMETER COMPLETE")
print("="*70)
print(f"  Path: {vio_path_length * scale:.1f}m")
print(f"  Mean error: {np.mean(position_error):.1f}m")
print(f"  (vs Full fusion with mag: 432.4m error)")
print("="*70)
