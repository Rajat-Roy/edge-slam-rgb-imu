#!/usr/bin/env python3
"""
SLAM with GPS Heading Correction
- Camera: Optical flow for translation
- Gyroscope: Rotation rate
- Accelerometer: Motion detection
- GPS: Scale + Heading correction + Position reset

Key improvements:
1. GPS heading correction (from velocity direction)
2. Periodic position reset to GPS
3. Complementary filter for gyro + GPS heading
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
OUTPUT_DIR = Path("study_v2/slam-gps-corrected")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

# Phone orientation
PHONE_ORIENTATION = "landscape"

print("="*70)
print("SLAM WITH GPS HEADING CORRECTION")
print("Camera + Gyro + Accel + GPS (heading + position correction)")
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
    raw_data = {1: [], 4: []}
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

# Load data
print("Loading data...")
camera_data = load_jsonl(DATASET_DIR / 'camera_frames.jsonl')
camera_data.sort(key=lambda x: x['timestamp'])

imu_data = load_imu_data()
accel_data = imu_data[1]
gyro_data = imu_data[4]

gps_data = load_jsonl(DATASET_DIR / 'gps_data.jsonl')
gps_data = [g for g in gps_data if g.get('latitude', 0) != 0]

print(f"  Camera: {len(camera_data)} frames")
print(f"  Gyroscope: {len(gyro_data)} samples")
print(f"  Accelerometer: {len(accel_data)} samples")
print(f"  GPS: {len(gps_data)} samples")
print()

# Time sync
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
# GPS Processing
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

# Compute GPS heading from velocity (direction of travel)
gps_heading = np.zeros(len(gps_data))
gps_speed = np.zeros(len(gps_data))

for i in range(1, len(gps_data)):
    dx = gps_smooth[i, 0] - gps_smooth[i-1, 0]
    dy = gps_smooth[i, 1] - gps_smooth[i-1, 1]
    dt = gps_times[i] - gps_times[i-1]

    speed = np.sqrt(dx**2 + dy**2) / dt if dt > 0 else 0
    gps_speed[i] = speed

    # Only trust heading when moving fast enough (>1 m/s)
    if speed > 1.0:
        gps_heading[i] = np.arctan2(dy, dx)
    else:
        gps_heading[i] = gps_heading[i-1] if i > 0 else 0

# Unwrap GPS heading
gps_heading = np.unwrap(gps_heading)

print(f"   GPS path: {gps_path_length:.1f}m")
print(f"   GPS heading range: {np.degrees(gps_heading.min()):.1f}° to {np.degrees(gps_heading.max()):.1f}°")
print(f"   Avg speed: {np.mean(gps_speed):.1f} m/s")

# ============================================================================
# Accelerometer - Motion Detection
# ============================================================================

print("2. Motion Detection...")

accel_values = np.array([s['values'] for s in accel_data])
accel_magnitude = np.linalg.norm(accel_values, axis=1)

gravity_mean = np.mean(accel_magnitude)
motion_threshold = 1.0
is_moving = np.abs(accel_magnitude - gravity_mean) > motion_threshold
motion_smooth = gaussian_filter1d(is_moving.astype(float), sigma=50)
motion_detected = motion_smooth > 0.3

print(f"   Moving: {100*np.mean(motion_detected):.1f}%")

# ============================================================================
# Gyroscope with GPS Heading Correction
# ============================================================================

print("3. Gyroscope + GPS Heading Fusion...")

gyro_values = np.array([s['values'] for s in gyro_data])

# Bias correction
first_sec_mask = gyro_times < 1.0
gyro_bias = np.mean(gyro_values[first_sec_mask], axis=0)
gyro_corrected = gyro_values - gyro_bias

# Integrate gyro heading
heading_gyro_only = np.zeros(len(gyro_data))
for i in range(1, len(gyro_data)):
    dt = gyro_times[i] - gyro_times[i-1]
    heading_gyro_only[i] = heading_gyro_only[i-1] + gyro_corrected[i, 2] * dt

# Fuse gyro with GPS heading using complementary filter
# Interpolate GPS heading to gyro timestamps
gps_heading_interp = np.interp(gyro_times, gps_times, gps_heading)
gps_speed_interp = np.interp(gyro_times, gps_times, gps_speed)

# Complementary filter with adaptive alpha based on GPS speed
heading_fused = np.zeros(len(gyro_data))
heading_fused[0] = gps_heading_interp[0]

for i in range(1, len(gyro_data)):
    dt = gyro_times[i] - gyro_times[i-1]

    # Gyro prediction
    heading_pred = heading_fused[i-1] + gyro_corrected[i, 2] * dt

    # GPS measurement
    heading_gps = gps_heading_interp[i]

    # Adaptive alpha: trust GPS more when moving fast
    speed = gps_speed_interp[i]
    if speed > 3.0:  # Fast motion - trust GPS heading more
        alpha = 0.95
    elif speed > 1.0:  # Moderate motion
        alpha = 0.98
    else:  # Slow/stationary - trust gyro more
        alpha = 0.995

    heading_fused[i] = alpha * heading_pred + (1 - alpha) * heading_gps

print(f"   Gyro heading range: {np.degrees(heading_gyro_only.max() - heading_gyro_only.min()):.1f}°")
print(f"   Fused heading range: {np.degrees(heading_fused.max() - heading_fused.min()):.1f}°")

# ============================================================================
# Optical Flow
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
# Trajectory Integration with GPS Position Reset
# ============================================================================

print("5. Trajectory Integration with GPS Correction...")

heading_at_flow = np.interp(flow_times, gyro_times, heading_fused)
motion_at_flow = np.interp(flow_times, accel_times, motion_detected.astype(float)) > 0.5

# First pass: integrate without GPS correction to get scale
position_unscaled = np.zeros((len(flow_data), 3))

for i in range(1, len(flow_data)):
    dt = flow_times[i] - flow_times[i-1]
    h = heading_at_flow[i]

    flow_forward = -flow_y[i]
    flow_right = -flow_x[i]

    motion_scale = 1.0 if motion_at_flow[i] else 0.1

    vx = flow_forward * np.cos(h) - flow_right * np.sin(h)
    vy = flow_forward * np.sin(h) + flow_right * np.cos(h)

    position_unscaled[i, 0] = position_unscaled[i-1, 0] + vx * dt * motion_scale
    position_unscaled[i, 1] = position_unscaled[i-1, 1] + vy * dt * motion_scale

# Compute scale
vio_path_length = np.sum(np.linalg.norm(np.diff(position_unscaled, axis=0), axis=1))
scale = gps_path_length / vio_path_length

print(f"   Scale: {scale:.4f} m/unit")

# Second pass: integrate with periodic GPS position reset
GPS_RESET_INTERVAL = 10.0  # Reset to GPS every 10 seconds
position = np.zeros((len(flow_data), 3))
last_gps_reset_time = 0

gps_resets = 0

for i in range(1, len(flow_data)):
    dt = flow_times[i] - flow_times[i-1]
    h = heading_at_flow[i]
    t = flow_times[i]

    flow_forward = -flow_y[i]
    flow_right = -flow_x[i]

    motion_scale = 1.0 if motion_at_flow[i] else 0.1

    vx = flow_forward * np.cos(h) - flow_right * np.sin(h)
    vy = flow_forward * np.sin(h) + flow_right * np.cos(h)

    # Integrate
    position[i, 0] = position[i-1, 0] + vx * dt * motion_scale * scale
    position[i, 1] = position[i-1, 1] + vy * dt * motion_scale * scale

    # GPS position reset
    if t - last_gps_reset_time >= GPS_RESET_INTERVAL:
        # Find closest GPS sample
        gps_idx = np.argmin(np.abs(gps_times - t))

        if np.abs(gps_times[gps_idx] - t) < 2.0:  # Within 2 seconds
            # Blend current VIO position with GPS
            blend_factor = 0.7  # 70% GPS, 30% VIO
            position[i, 0] = blend_factor * gps_smooth[gps_idx, 0] + (1 - blend_factor) * position[i, 0]
            position[i, 1] = blend_factor * gps_smooth[gps_idx, 1] + (1 - blend_factor) * position[i, 1]
            last_gps_reset_time = t
            gps_resets += 1

print(f"   GPS resets: {gps_resets}")

# ============================================================================
# Error Analysis
# ============================================================================

print("6. Error Analysis...")

vio_x_at_gps = np.interp(gps_times, flow_times, position[:, 0])
vio_y_at_gps = np.interp(gps_times, flow_times, position[:, 1])
position_error = np.sqrt((vio_x_at_gps - gps_smooth[:, 0])**2 + (vio_y_at_gps - gps_smooth[:, 1])**2)

# Also compute error for uncorrected version
position_unscaled_scaled = position_unscaled * scale
vio_x_uncorr = np.interp(gps_times, flow_times, position_unscaled_scaled[:, 0])
vio_y_uncorr = np.interp(gps_times, flow_times, position_unscaled_scaled[:, 1])
error_uncorrected = np.sqrt((vio_x_uncorr - gps_smooth[:, 0])**2 + (vio_y_uncorr - gps_smooth[:, 1])**2)

print(f"   Mean error (corrected): {np.mean(position_error):.1f}m")
print(f"   Mean error (uncorrected): {np.mean(error_uncorrected):.1f}m")
print(f"   Improvement: {100*(1 - np.mean(position_error)/np.mean(error_uncorrected)):.1f}%")

# ============================================================================
# Visualization
# ============================================================================

print("7. Creating Visualization...")

fig = plt.figure(figsize=(20, 12))
fig.suptitle('SLAM with GPS Heading & Position Correction\nCamera + Gyro + Accel + GPS',
             fontsize=14, fontweight='bold')

# 1. Trajectory comparison
ax1 = fig.add_subplot(2, 3, 1)
ax1.plot(gps_smooth[:, 0], gps_smooth[:, 1], 'orange', linewidth=2, label='GPS')
ax1.plot(position[:, 0], position[:, 1], 'b-', linewidth=1.5, label='VIO (corrected)')
ax1.plot(position_unscaled_scaled[:, 0], position_unscaled_scaled[:, 1], 'g--', linewidth=1, alpha=0.5, label='VIO (uncorrected)')
ax1.scatter(position[0, 0], position[0, 1], c='green', s=100, marker='o', label='Start', zorder=5)
ax1.scatter(position[-1, 0], position[-1, 1], c='red', s=100, marker='X', label='End', zorder=5)
ax1.set_xlabel('X - East (m)')
ax1.set_ylabel('Y - North (m)')
ax1.set_title('Trajectory Comparison')
ax1.legend(loc='upper left', fontsize=8)
ax1.grid(True, alpha=0.3)
ax1.set_aspect('equal')

# 2. Heading comparison
ax2 = fig.add_subplot(2, 3, 2)
ax2.plot(gyro_times, np.degrees(heading_gyro_only), 'b-', alpha=0.5, linewidth=0.5, label='Gyro only')
ax2.plot(gps_times, np.degrees(gps_heading), 'orange', linewidth=1, label='GPS heading')
ax2.plot(gyro_times, np.degrees(heading_fused), 'r-', linewidth=1, label='Fused')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Heading (°)')
ax2.set_title('Heading: Gyro vs GPS vs Fused')
ax2.legend(fontsize=8)
ax2.grid(True, alpha=0.3)

# 3. GPS speed
ax3 = fig.add_subplot(2, 3, 3)
ax3.plot(gps_times, gps_speed, 'b-', linewidth=1)
ax3.axhline(y=1.0, color='r', linestyle='--', alpha=0.5, label='Min for heading')
ax3.axhline(y=3.0, color='g', linestyle='--', alpha=0.5, label='High trust')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Speed (m/s)')
ax3.set_title('GPS Speed (for heading trust)')
ax3.legend(fontsize=8)
ax3.grid(True, alpha=0.3)

# 4. Position error comparison
ax4 = fig.add_subplot(2, 3, 4)
ax4.plot(gps_times, error_uncorrected, 'g-', linewidth=1, alpha=0.7, label=f'Uncorrected: {np.mean(error_uncorrected):.0f}m')
ax4.plot(gps_times, position_error, 'b-', linewidth=1, label=f'Corrected: {np.mean(position_error):.0f}m')
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Error (m)')
ax4.set_title('Position Error vs GPS')
ax4.legend()
ax4.grid(True, alpha=0.3)

# 5. Optical flow
ax5 = fig.add_subplot(2, 3, 5)
flow_mag = np.sqrt(flow_x**2 + flow_y**2)
ax5.plot(flow_times, flow_mag, 'b-', linewidth=0.5)
ax5.set_xlabel('Time (s)')
ax5.set_ylabel('Flow (pixels)')
ax5.set_title('Optical Flow Magnitude')
ax5.grid(True, alpha=0.3)

# 6. Statistics
ax6 = fig.add_subplot(2, 3, 6)
ax6.axis('off')

improvement = 100*(1 - np.mean(position_error)/np.mean(error_uncorrected))

stats_text = f"""GPS HEADING & POSITION CORRECTION

Sensors:
  • Camera: {len(flow_data)} samples
  • Gyroscope: {len(gyro_data)} samples
  • Accelerometer: {len(accel_data)} samples
  • GPS: {len(gps_data)} samples

GPS Corrections:
  • Heading fusion: Adaptive alpha
  • Position resets: Every {GPS_RESET_INTERVAL}s
  • Blend factor: 70% GPS / 30% VIO
  • Total resets: {gps_resets}

Heading Fusion:
  • Gyro range: {np.degrees(heading_gyro_only.max() - heading_gyro_only.min()):.1f}°
  • GPS range: {np.degrees(gps_heading.max() - gps_heading.min()):.1f}°
  • Fused range: {np.degrees(heading_fused.max() - heading_fused.min()):.1f}°

Trajectory:
  • GPS path: {gps_path_length:.1f}m
  • Scale: {scale:.4f} m/unit

Error Comparison:
  • Uncorrected: {np.mean(error_uncorrected):.1f}m mean
  • Corrected: {np.mean(position_error):.1f}m mean
  • Improvement: {improvement:.1f}%
  • Max error: {np.max(position_error):.1f}m

Displacement:
  • GPS: {np.linalg.norm(gps_smooth[-1] - gps_smooth[0]):.1f}m
  • VIO: {np.linalg.norm(position[-1] - position[0]):.1f}m
"""

ax6.text(0.05, 0.95, stats_text, transform=ax6.transAxes,
        fontsize=9, verticalalignment='top', fontfamily='monospace',
        bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))

plt.tight_layout()
output_file = OUTPUT_DIR / 'slam_gps_corrected.png'
plt.savefig(output_file, dpi=150, bbox_inches='tight')
plt.close()

print(f"   Saved: {output_file}")
print()

print("="*70)
print("SLAM WITH GPS CORRECTION COMPLETE")
print("="*70)
print(f"  Mean error: {np.mean(position_error):.1f}m")
print(f"  Improvement: {improvement:.1f}% over uncorrected")
print()
print("  Comparison:")
print(f"    - With magnetometer: 432.4m")
print(f"    - Without magnetometer: 333.1m")
print(f"    - With GPS correction: {np.mean(position_error):.1f}m")
print("="*70)
