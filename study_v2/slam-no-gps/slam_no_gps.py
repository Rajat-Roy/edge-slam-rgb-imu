#!/usr/bin/env python3
"""
SLAM with NO GPS
- Camera optical flow + Gyroscope only
- No GPS for position or scale correction
- Pure dead reckoning with VIO
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
OUTPUT_DIR = Path("study_v2/slam-no-gps")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

PHONE_ORIENTATION = "landscape"

print("="*70)
print("SLAM WITH NO GPS")
print("Pure VIO: Camera + Gyroscope Only")
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
gyro_data = imu_data[4]

# Load GPS only for ground truth comparison
gps_data = load_jsonl(DATASET_DIR / 'gps_data.jsonl')
gps_data = [g for g in gps_data if g.get('latitude', 0) != 0]

print(f"  Camera: {len(camera_data)} frames")
print(f"  Gyroscope: {len(gyro_data)} samples")
print(f"  GPS (ground truth only): {len(gps_data)} samples")
print()

# Time sync
camera_times = np.array([c['timestamp']/1e9 for c in camera_data])
gyro_times = np.array([s['timestamp']/1e9 for s in gyro_data])
gps_times = np.array([s['timestamp']/1e9 for s in gps_data])

time_offset = min(camera_times[0], gyro_times[0], gps_times[0])
camera_times -= time_offset
gyro_times -= time_offset
gps_times -= time_offset

# ============================================================================
# GPS Ground Truth (for comparison only)
# ============================================================================

print("1. GPS Ground Truth...")

lat0 = gps_data[0]['latitude']
lon0 = gps_data[0]['longitude']
meters_per_deg_lat = 111320
meters_per_deg_lon = 111320 * np.cos(np.radians(lat0))

gps_positions = []
for sample in gps_data:
    x = (sample['longitude'] - lon0) * meters_per_deg_lon
    y = (sample['latitude'] - lat0) * meters_per_deg_lat
    gps_positions.append([x, y])

gps_positions = np.array(gps_positions)

# Smooth GPS
sigma = 2
gps_smooth = np.zeros_like(gps_positions)
for i in range(2):
    gps_smooth[:, i] = gaussian_filter1d(gps_positions[:, i], sigma)

gps_path_length = np.sum(np.linalg.norm(np.diff(gps_smooth, axis=0), axis=1))
print(f"   GPS path (ground truth): {gps_path_length:.1f}m")

# ============================================================================
# Gyroscope Processing
# ============================================================================

print("2. Gyroscope Processing...")

gyro_values = np.array([s['values'] for s in gyro_data])
first_sec_mask = gyro_times < 1.0
gyro_bias = np.mean(gyro_values[first_sec_mask], axis=0)
gyro_corrected = gyro_values - gyro_bias

# Integrate heading
heading_gyro = np.zeros(len(gyro_data))
for i in range(1, len(gyro_data)):
    dt = gyro_times[i] - gyro_times[i-1]
    heading_gyro[i] = heading_gyro[i-1] + gyro_corrected[i, 2] * dt

print(f"   Gyro heading range: {np.degrees(heading_gyro.max() - heading_gyro.min()):.1f}°")

# ============================================================================
# Optical Flow Processing
# ============================================================================

print("3. Optical Flow...")

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

# Get heading at flow times
heading_at_flow = np.interp(flow_times, gyro_times, heading_gyro)

# Initial heading from GPS (only for alignment, not for correction)
if len(gps_smooth) >= 5:
    gps_dx = gps_smooth[4, 0] - gps_smooth[0, 0]
    gps_dy = gps_smooth[4, 1] - gps_smooth[0, 1]
    initial_heading = np.arctan2(gps_dy, gps_dx)
else:
    initial_heading = 0

heading_at_flow += initial_heading

# ============================================================================
# VIO Trajectory (No GPS)
# ============================================================================

print("4. Computing VIO Trajectory (No GPS)...")

# First pass: compute unscaled trajectory
position_unscaled = np.zeros((len(flow_data), 2))
for i in range(1, len(flow_data)):
    dt = flow_times[i] - flow_times[i-1]
    h = heading_at_flow[i]
    flow_forward = -flow_y[i]
    flow_right = -flow_x[i]
    vx = flow_forward * np.cos(h) - flow_right * np.sin(h)
    vy = flow_forward * np.sin(h) + flow_right * np.cos(h)
    position_unscaled[i, 0] = position_unscaled[i-1, 0] + vx * dt
    position_unscaled[i, 1] = position_unscaled[i-1, 1] + vy * dt

vio_path_length_unscaled = np.sum(np.linalg.norm(np.diff(position_unscaled, axis=0), axis=1))

# For fair comparison, we'll show two versions:
# 1. Unscaled (raw VIO)
# 2. Post-hoc scaled (using GPS path length - but this is cheating for real-time)

# Scale factor (would need GPS to compute in real scenario)
scale = gps_path_length / vio_path_length_unscaled
print(f"   Scale factor (from GPS): {scale:.4f} m/unit")

# Scaled trajectory
position_scaled = position_unscaled * scale

vio_path_length = np.sum(np.linalg.norm(np.diff(position_scaled, axis=0), axis=1))
print(f"   VIO path (scaled): {vio_path_length:.1f}m")

# ============================================================================
# Error Analysis
# ============================================================================

print("5. Error Analysis...")

# Error for scaled version
vio_x_at_gps = np.interp(gps_times, flow_times, position_scaled[:, 0])
vio_y_at_gps = np.interp(gps_times, flow_times, position_scaled[:, 1])
position_error_scaled = np.sqrt((vio_x_at_gps - gps_smooth[:, 0])**2 + (vio_y_at_gps - gps_smooth[:, 1])**2)

# Error for unscaled version
vio_x_unscaled = np.interp(gps_times, flow_times, position_unscaled[:, 0])
vio_y_unscaled = np.interp(gps_times, flow_times, position_unscaled[:, 1])
position_error_unscaled = np.sqrt((vio_x_unscaled - gps_smooth[:, 0])**2 + (vio_y_unscaled - gps_smooth[:, 1])**2)

print(f"   Scaled VIO error:")
print(f"     Mean: {np.mean(position_error_scaled):.1f}m")
print(f"     Max: {np.max(position_error_scaled):.1f}m")
print(f"     Final: {position_error_scaled[-1]:.1f}m")
print(f"   Unscaled VIO error:")
print(f"     Mean: {np.mean(position_error_unscaled):.1f}m")

# ============================================================================
# Visualization
# ============================================================================

print("6. Creating Visualization...")

fig = plt.figure(figsize=(20, 12))
fig.suptitle('SLAM with NO GPS\nPure VIO: Camera + Gyroscope Only',
             fontsize=14, fontweight='bold')

# 1. Trajectory Comparison
ax1 = fig.add_subplot(2, 3, 1)
ax1.plot(gps_smooth[:, 0], gps_smooth[:, 1], 'orange', linewidth=2, label='GPS (ground truth)')
ax1.plot(position_scaled[:, 0], position_scaled[:, 1], 'b-', linewidth=1.5, label='VIO (scaled)')
ax1.scatter(position_scaled[0, 0], position_scaled[0, 1], c='green', s=100, marker='o', label='Start', zorder=5)
ax1.scatter(position_scaled[-1, 0], position_scaled[-1, 1], c='red', s=100, marker='X', label='End', zorder=5)
ax1.set_xlabel('X - East (m)')
ax1.set_ylabel('Y - North (m)')
ax1.set_title('VIO Trajectory (Scaled)')
ax1.legend()
ax1.grid(True, alpha=0.3)
ax1.set_aspect('equal')

# 2. Unscaled Trajectory
ax2 = fig.add_subplot(2, 3, 2)
ax2.plot(gps_smooth[:, 0], gps_smooth[:, 1], 'orange', linewidth=2, label='GPS')
ax2.plot(position_unscaled[:, 0], position_unscaled[:, 1], 'g-', linewidth=1.5, label='VIO (unscaled)')
ax2.scatter(position_unscaled[0, 0], position_unscaled[0, 1], c='green', s=100, marker='o', zorder=5)
ax2.scatter(position_unscaled[-1, 0], position_unscaled[-1, 1], c='red', s=100, marker='X', zorder=5)
ax2.set_xlabel('X - East (m)')
ax2.set_ylabel('Y - North (m)')
ax2.set_title('VIO Trajectory (Unscaled - Raw)')
ax2.legend()
ax2.grid(True, alpha=0.3)
ax2.set_aspect('equal')

# 3. Heading
ax3 = fig.add_subplot(2, 3, 3)
ax3.plot(flow_times, np.degrees(heading_at_flow), 'b-', linewidth=1)
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Heading (°)')
ax3.set_title('Gyroscope Heading (Integrated)')
ax3.grid(True, alpha=0.3)

# 4. Position Error
ax4 = fig.add_subplot(2, 3, 4)
ax4.plot(gps_times, position_error_scaled, 'b-', linewidth=1, label='Scaled')
ax4.axhline(y=np.mean(position_error_scaled), color='r', linestyle='--',
            label=f'Mean: {np.mean(position_error_scaled):.1f}m')
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Error (m)')
ax4.set_title('Position Error vs GPS (Scaled VIO)')
ax4.legend()
ax4.grid(True, alpha=0.3)

# 5. Drift Analysis
ax5 = fig.add_subplot(2, 3, 5)
# Show error accumulation over distance
gps_dist = np.cumsum(np.concatenate([[0], np.linalg.norm(np.diff(gps_smooth, axis=0), axis=1)]))
ax5.plot(gps_dist, position_error_scaled, 'b-', linewidth=1)
ax5.set_xlabel('Distance traveled (m)')
ax5.set_ylabel('Position Error (m)')
ax5.set_title('Error vs Distance (Drift Analysis)')
ax5.grid(True, alpha=0.3)

# Calculate drift rate
drift_rate = position_error_scaled[-1] / gps_path_length * 100
ax5.text(0.95, 0.05, f'Drift: {drift_rate:.1f}% of path',
         transform=ax5.transAxes, ha='right', fontsize=10,
         bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.5))

# 6. Statistics
ax6 = fig.add_subplot(2, 3, 6)
ax6.axis('off')

stats_text = f"""SLAM WITHOUT GPS

Sensors Used:
  • Camera (optical flow)
  • Gyroscope (heading)
  • NO GPS for correction

Processing:
  • Pure dead reckoning
  • Gyro bias from first 1 second
  • Flow sample interval: {SAMPLE_INTERVAL} frames

Trajectory:
  • GPS path (truth): {gps_path_length:.1f}m
  • VIO path (scaled): {vio_path_length:.1f}m
  • Scale factor: {scale:.4f}

Error (Scaled VIO):
  • Mean: {np.mean(position_error_scaled):.1f}m
  • Max: {np.max(position_error_scaled):.1f}m
  • Final: {position_error_scaled[-1]:.1f}m
  • Drift rate: {drift_rate:.1f}% of path

Comparison:
  • EKF with GPS: 17.8m mean error
  • VIO without GPS: {np.mean(position_error_scaled):.1f}m mean error
  • GPS improves by: {(1 - 17.8/np.mean(position_error_scaled))*100:.0f}%

Note: Without GPS, scale is unknown.
Scaled version uses post-hoc GPS path
for fair shape comparison.
"""

ax6.text(0.05, 0.95, stats_text, transform=ax6.transAxes,
        fontsize=9, verticalalignment='top', fontfamily='monospace',
        bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.5))

plt.tight_layout()
output_file = OUTPUT_DIR / 'slam_no_gps.png'
plt.savefig(output_file, dpi=150, bbox_inches='tight')
plt.close()

print(f"   Saved: {output_file}")
print()

print("="*70)
print("SLAM WITHOUT GPS COMPLETE")
print("="*70)
print(f"  Mean error (scaled): {np.mean(position_error_scaled):.1f}m")
print(f"  Final error: {position_error_scaled[-1]:.1f}m")
print(f"  Drift rate: {drift_rate:.1f}% of path length")
print()
print("  Comparison:")
print(f"    - EKF with GPS: 17.8m")
print(f"    - VIO without GPS: {np.mean(position_error_scaled):.1f}m")
print(f"    - GPS improvement: {(1 - 17.8/np.mean(position_error_scaled))*100:.0f}%")
print("="*70)
