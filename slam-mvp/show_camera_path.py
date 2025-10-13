#!/usr/bin/env python3
"""
Extract camera trajectory from VI-SLAM results
"""
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent / "src"))

from data_loader import SLAMDataLoader
from visual_inertial_slam import VisualInertialSLAM
from imu_processor import IMUProcessor
import numpy as np
import json

def extract_camera_trajectory():
    """Extract camera trajectory from the long session"""
    print("=== Extracting Camera Trajectory ===")

    # Load the longer session
    data_loader = SLAMDataLoader("data/collected_data")
    camera_matrix = data_loader.get_camera_intrinsics()

    # Setup IMU
    imu_processor = IMUProcessor()
    imu_processor.load_imu_data(data_loader)
    imu_processor.calibrate_static_bias(duration_seconds=1.0)

    # Initialize VI-SLAM
    vi_slam = VisualInertialSLAM(camera_matrix, imu_processor)

    # Use fewer features for speed
    import cv2
    vi_slam.orb = cv2.ORB_create(nfeatures=500)

    print("Processing frames to extract camera path...")

    trajectory_points = []

    # Process every 5th frame to get smooth path
    for i in range(0, min(150, data_loader.get_frame_count()), 5):
        frame, frame_data = data_loader.get_frame(i)
        if frame is not None and frame_data is not None:
            R, t = vi_slam.process_frame(frame, frame_data['timestamp'])

            # Store camera position
            position = t.flatten()
            trajectory_points.append({
                'frame': i,
                'timestamp': frame_data['timestamp'],
                'position': position.tolist(),
                'rotation': R.tolist()
            })

    print(f"Extracted {len(trajectory_points)} trajectory points")

    # Calculate path statistics
    if len(trajectory_points) > 1:
        total_distance = 0
        speeds = []

        for i in range(1, len(trajectory_points)):
            prev_pos = np.array(trajectory_points[i-1]['position'])
            curr_pos = np.array(trajectory_points[i]['position'])

            distance = np.linalg.norm(curr_pos - prev_pos)
            total_distance += distance

            # Calculate speed (rough estimate)
            time_diff = (trajectory_points[i]['timestamp'] - trajectory_points[i-1]['timestamp']) / 1e9
            if time_diff > 0:
                speed = distance / time_diff
                speeds.append(speed)

        print(f"\\nTrajectory Analysis:")
        print(f"  Total path length: {total_distance:.2f} meters")
        print(f"  Average speed: {np.mean(speeds):.2f} m/s")
        print(f"  Max speed: {np.max(speeds):.2f} m/s")

        # Get bounds
        positions = np.array([p['position'] for p in trajectory_points])
        bounds = {
            'min': positions.min(axis=0).tolist(),
            'max': positions.max(axis=0).tolist(),
            'range': (positions.max(axis=0) - positions.min(axis=0)).tolist()
        }

        print(f"  Movement range: X={bounds['range'][0]:.1f}m, Y={bounds['range'][1]:.1f}m, Z={bounds['range'][2]:.1f}m")

    # Save trajectory data
    trajectory_data = {
        'session_info': {
            'duration': (data_loader.session_metadata['endTime'] - data_loader.session_metadata['startTime']) / 1000,
            'total_frames': data_loader.session_metadata['totalFrames'],
            'processed_frames': len(trajectory_points)
        },
        'trajectory': trajectory_points,
        'statistics': {
            'total_distance': total_distance,
            'average_speed': np.mean(speeds) if speeds else 0,
            'max_speed': np.max(speeds) if speeds else 0,
            'bounds': bounds
        }
    }

    # Save to JSON file
    output_file = Path(__file__).parent / "output" / "camera_trajectory.json"
    output_file.parent.mkdir(exist_ok=True)

    with open(output_file, 'w') as f:
        json.dump(trajectory_data, f, indent=2)

    print(f"\\nCamera trajectory saved to: {output_file}")
    return trajectory_data

if __name__ == "__main__":
    extract_camera_trajectory()