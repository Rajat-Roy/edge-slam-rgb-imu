#!/usr/bin/env python3
"""
SLAM with Extended Kalman Filter (No GPS Heading Correction)
- Optimal fusion of VIO + GPS position only
- State: [x, y, vx, vy, heading]
- Heading from gyroscope only (no GPS velocity heading)
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
OUTPUT_DIR = Path("study_v2/slam-ekf-no-heading")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

PHONE_ORIENTATION = "landscape"

print("="*70)
print("EKF SLAM WITHOUT GPS HEADING CORRECTION")
print("VIO + GPS Position Only (Gyro-only heading)")
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
gps_accuracies = []
for sample in gps_data:
    x = (sample['longitude'] - lon0) * meters_per_deg_lon
    y = (sample['latitude'] - lat0) * meters_per_deg_lat
    gps_positions.append([x, y])
    gps_accuracies.append(sample.get('accuracy', 5.0))

gps_positions = np.array(gps_positions)
gps_accuracies = np.array(gps_accuracies)

# Smooth GPS for path length
sigma = 2
gps_smooth = np.zeros_like(gps_positions)
for i in range(2):
    gps_smooth[:, i] = gaussian_filter1d(gps_positions[:, i], sigma)

gps_path_length = np.sum(np.linalg.norm(np.diff(gps_smooth, axis=0), axis=1))
print(f"   GPS path: {gps_path_length:.1f}m")

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

# Initial heading from GPS
if len(gps_smooth) >= 5:
    gps_dx = gps_smooth[4, 0] - gps_smooth[0, 0]
    gps_dy = gps_smooth[4, 1] - gps_smooth[0, 1]
    initial_heading = np.arctan2(gps_dy, gps_dx)
else:
    initial_heading = 0

heading_at_flow += initial_heading

# Compute scale
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

vio_path_length = np.sum(np.linalg.norm(np.diff(position_unscaled, axis=0), axis=1))
scale = gps_path_length / vio_path_length
print(f"   Scale: {scale:.4f} m/unit")

# ============================================================================
# Extended Kalman Filter (NO GPS heading correction)
# ============================================================================

print("4. Extended Kalman Filter (NO heading correction)...")

class EKF_NoHeading:
    def __init__(self):
        # State vector [x, y, vx, vy, heading]
        self.x = np.zeros(5)
        self.x[4] = initial_heading

        # State covariance
        self.P = np.diag([1.0, 1.0, 1.0, 1.0, 0.1])

        # Process noise
        self.Q = np.diag([0.5, 0.5, 0.5, 0.5, 0.01])

    def predict(self, dt, omega, flow_forward, flow_right):
        """Predict step using gyro and optical flow"""
        x, y, vx, vy, heading = self.x

        # Update heading with gyro ONLY (no GPS correction)
        heading_new = heading + omega * dt

        # Compute velocity from optical flow
        vx_new = (flow_forward * np.cos(heading_new) - flow_right * np.sin(heading_new)) * scale
        vy_new = (flow_forward * np.sin(heading_new) + flow_right * np.cos(heading_new)) * scale

        # Update position
        x_new = x + vx_new * dt
        y_new = y + vy_new * dt

        # Update state
        self.x = np.array([x_new, y_new, vx_new, vy_new, heading_new])

        # Jacobian
        F = np.eye(5)
        F[0, 2] = dt
        F[1, 3] = dt

        # Update covariance
        self.P = F @ self.P @ F.T + self.Q * dt

    def update_gps(self, z_gps, accuracy):
        """Update step using GPS position ONLY (no heading)"""
        # Measurement matrix (observe x, y only)
        H = np.array([
            [1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0]
        ])

        # Measurement noise
        R = np.diag([accuracy**2, accuracy**2])

        # Innovation
        y = z_gps - H @ self.x

        # Innovation covariance
        S = H @ self.P @ H.T + R

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update state
        self.x = self.x + K @ y

        # Update covariance
        I = np.eye(5)
        self.P = (I - K @ H) @ self.P

        return np.linalg.norm(y)

# Initialize EKF
ekf = EKF_NoHeading()

# Storage
ekf_positions = np.zeros((len(flow_data), 2))
ekf_headings = np.zeros(len(flow_data))
ekf_uncertainties = np.zeros(len(flow_data))
innovations = []

# GPS index
gps_idx = 0
gps_updates = 0

for i in range(len(flow_data)):
    t = flow_times[i]

    if i > 0:
        dt = flow_times[i] - flow_times[i-1]

        # Get gyro
        gyro_idx = np.argmin(np.abs(gyro_times - t))
        omega = gyro_corrected[gyro_idx, 2]

        # Get flow
        flow_forward = -flow_y[i]
        flow_right = -flow_x[i]

        # Predict
        ekf.predict(dt, omega, flow_forward, flow_right)

        # NO heading correction from velocity!

        # GPS update
        while gps_idx < len(gps_times) and gps_times[gps_idx] <= t:
            if gps_times[gps_idx] >= flow_times[i-1]:
                z_gps = gps_positions[gps_idx]
                accuracy = max(gps_accuracies[gps_idx], 3.0)

                innovation = ekf.update_gps(z_gps, accuracy)
                innovations.append(innovation)
                gps_updates += 1
            gps_idx += 1

    # Store
    ekf_positions[i] = ekf.x[:2]
    ekf_headings[i] = ekf.x[4]
    ekf_uncertainties[i] = np.sqrt(ekf.P[0, 0] + ekf.P[1, 1])

print(f"   GPS updates: {gps_updates}")
print(f"   Mean innovation: {np.mean(innovations):.1f}m")

# ============================================================================
# Error Analysis
# ============================================================================

print("5. Error Analysis...")

ekf_x_at_gps = np.interp(gps_times, flow_times, ekf_positions[:, 0])
ekf_y_at_gps = np.interp(gps_times, flow_times, ekf_positions[:, 1])
position_error = np.sqrt((ekf_x_at_gps - gps_smooth[:, 0])**2 + (ekf_y_at_gps - gps_smooth[:, 1])**2)

print(f"   Mean error: {np.mean(position_error):.1f}m")
print(f"   Max error: {np.max(position_error):.1f}m")

# ============================================================================
# Visualization
# ============================================================================

print("6. Creating Visualization...")

fig = plt.figure(figsize=(20, 12))
fig.suptitle('EKF SLAM without GPS Heading Correction\nVIO + GPS Position Only',
             fontsize=14, fontweight='bold')

# 1. Trajectory
ax1 = fig.add_subplot(2, 3, 1)
ax1.plot(gps_smooth[:, 0], gps_smooth[:, 1], 'orange', linewidth=2, label='GPS')
ax1.plot(ekf_positions[:, 0], ekf_positions[:, 1], 'b-', linewidth=1.5, label='EKF (no heading corr)')
ax1.scatter(ekf_positions[0, 0], ekf_positions[0, 1], c='green', s=100, marker='o', label='Start', zorder=5)
ax1.scatter(ekf_positions[-1, 0], ekf_positions[-1, 1], c='red', s=100, marker='X', label='End', zorder=5)
ax1.set_xlabel('X - East (m)')
ax1.set_ylabel('Y - North (m)')
ax1.set_title('EKF Trajectory (Gyro-only heading)')
ax1.legend()
ax1.grid(True, alpha=0.3)
ax1.set_aspect('equal')

# 2. Heading
ax2 = fig.add_subplot(2, 3, 2)
ax2.plot(flow_times, np.degrees(ekf_headings), 'b-', linewidth=1, label='EKF heading')
ax2.plot(flow_times, np.degrees(heading_at_flow), 'g--', alpha=0.5, linewidth=0.5, label='Gyro integrated')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Heading (°)')
ax2.set_title('Heading (Gyro only, no GPS correction)')
ax2.legend()
ax2.grid(True, alpha=0.3)

# 3. Uncertainty
ax3 = fig.add_subplot(2, 3, 3)
ax3.plot(flow_times, ekf_uncertainties, 'b-', linewidth=1)
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Position Uncertainty (m)')
ax3.set_title('EKF Uncertainty (1-sigma)')
ax3.grid(True, alpha=0.3)

# 4. Error
ax4 = fig.add_subplot(2, 3, 4)
ax4.plot(gps_times, position_error, 'b-', linewidth=1)
ax4.axhline(y=np.mean(position_error), color='r', linestyle='--', label=f'Mean: {np.mean(position_error):.1f}m')
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Error (m)')
ax4.set_title('Position Error vs GPS')
ax4.legend()
ax4.grid(True, alpha=0.3)

# 5. Innovation
ax5 = fig.add_subplot(2, 3, 5)
ax5.plot(innovations, 'b-', linewidth=0.5)
ax5.axhline(y=np.mean(innovations), color='r', linestyle='--', label=f'Mean: {np.mean(innovations):.1f}m')
ax5.set_xlabel('GPS Update')
ax5.set_ylabel('Innovation (m)')
ax5.set_title('GPS Innovation')
ax5.legend()
ax5.grid(True, alpha=0.3)

# 6. Statistics
ax6 = fig.add_subplot(2, 3, 6)
ax6.axis('off')

stats_text = f"""EKF WITHOUT GPS HEADING CORRECTION

Heading Source:
  • Gyroscope only (no velocity-based correction)
  • Initial heading from GPS direction

State Vector:
  [x, y, vx, vy, heading]

GPS Updates:
  • Position only (x, y)
  • No heading correction from velocity
  • Updates: {gps_updates}

Trajectory:
  • GPS path: {gps_path_length:.1f}m
  • Scale: {scale:.4f} m/unit

Error:
  • Mean: {np.mean(position_error):.1f}m
  • Max: {np.max(position_error):.1f}m
  • Final: {position_error[-1]:.1f}m

Comparison:
  • EKF with heading corr: 17.8m
  • EKF without heading corr: {np.mean(position_error):.1f}m
  • Difference: {np.mean(position_error) - 17.8:.1f}m

Note: Heading correction uses GPS velocity
direction to reduce gyro drift. Without it,
heading drift causes position errors even
with GPS position updates.
"""

ax6.text(0.05, 0.95, stats_text, transform=ax6.transAxes,
        fontsize=9, verticalalignment='top', fontfamily='monospace',
        bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.5))

plt.tight_layout()
output_file = OUTPUT_DIR / 'slam_ekf_no_heading.png'
plt.savefig(output_file, dpi=150, bbox_inches='tight')
plt.close()

print(f"   Saved: {output_file}")
print()

print("="*70)
print("EKF (NO HEADING CORRECTION) COMPLETE")
print("="*70)
print(f"  Mean error: {np.mean(position_error):.1f}m")
print()
print("  Comparison:")
print(f"    - EKF with heading correction: 17.8m")
print(f"    - EKF without heading correction: {np.mean(position_error):.1f}m")
print(f"    - Difference: {np.mean(position_error) - 17.8:.1f}m ({100*(np.mean(position_error)/17.8 - 1):.0f}% worse)")
print("="*70)
