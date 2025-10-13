#!/usr/bin/env python3
"""
Analyze actual movement pattern to determine correct coordinate system
"""
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent / "src"))

from data_loader import SLAMDataLoader
from visual_inertial_slam import VisualInertialSLAM
from imu_processor import IMUProcessor
import numpy as np

def analyze_real_movement():
    """Analyze movement to determine correct coordinate transformation"""
    print("=== Real Movement Analysis ===")

    # Load data
    data_loader = SLAMDataLoader("data/collected_data")
    camera_matrix = data_loader.get_camera_intrinsics()

    # Setup IMU
    imu_processor = IMUProcessor()
    imu_processor.load_imu_data(data_loader)
    imu_processor.calibrate_static_bias(duration_seconds=1.0)

    # Get gravity analysis
    imu_stats = imu_processor.get_statistics()
    gravity_vector = imu_stats['accelerometer']['mean']
    gravity_magnitude = np.linalg.norm(gravity_vector)
    gravity_normalized = gravity_vector / gravity_magnitude

    print(f"\\nGravity Analysis:")
    print(f"  Raw gravity vector: [{gravity_vector[0]:.3f}, {gravity_vector[1]:.3f}, {gravity_vector[2]:.3f}]")
    print(f"  Normalized: [{gravity_normalized[0]:.3f}, {gravity_normalized[1]:.3f}, {gravity_normalized[2]:.3f}]")
    print(f"  Magnitude: {gravity_magnitude:.3f} m/s²")

    # Determine which axis is most aligned with gravity
    abs_gravity = np.abs(gravity_normalized)
    dominant_axis = np.argmax(abs_gravity)
    axis_names = ['X', 'Y', 'Z']
    print(f"  Dominant gravity axis: {axis_names[dominant_axis]} ({'negative' if gravity_normalized[dominant_axis] < 0 else 'positive'})")

    # Initialize SLAM
    vi_slam = VisualInertialSLAM(camera_matrix, imu_processor)
    import cv2
    vi_slam.orb = cv2.ORB_create(nfeatures=300)

    print(f"\\nProcessing trajectory...")

    # Get trajectory
    trajectory = []
    for i in range(0, 60, 5):
        frame, frame_data = data_loader.get_frame(i)
        if frame is not None and frame_data is not None:
            R, t = vi_slam.process_frame(frame, frame_data['timestamp'])
            trajectory.append(t.flatten().copy())

    if len(trajectory) < 2:
        print("Not enough trajectory points")
        return

    trajectory = np.array(trajectory)
    print(f"  Got {len(trajectory)} trajectory points")

    # Analyze movement in each axis
    movement_range = trajectory.max(axis=0) - trajectory.min(axis=0)
    movement_std = trajectory.std(axis=0)

    print(f"\\nMovement Analysis:")
    print(f"  Range: X={movement_range[0]:.3f}m, Y={movement_range[1]:.3f}m, Z={movement_range[2]:.3f}m")
    print(f"  Std dev: X={movement_std[0]:.3f}m, Y={movement_std[1]:.3f}m, Z={movement_std[2]:.3f}m")

    # Identify horizontal plane
    # For horizontal movement, two axes should have large movement, one should be small
    movement_sorted_indices = np.argsort(movement_range)
    smallest_movement_axis = movement_sorted_indices[0]
    largest_movement_axes = movement_sorted_indices[1:]

    print(f"\\nMovement Pattern:")
    print(f"  Smallest movement: {axis_names[smallest_movement_axis]} ({movement_range[smallest_movement_axis]:.3f}m)")
    print(f"  Largest movements: {[axis_names[i] for i in largest_movement_axes]} ({[movement_range[i] for i in largest_movement_axes]})")

    # Determine correct coordinate system
    print(f"\\n=== Coordinate System Determination ===")

    # If you recorded horizontal movement, the smallest movement axis should be "up"
    # And gravity should be opposite to that axis

    if smallest_movement_axis == 0:  # X has smallest movement
        print("X-axis has smallest movement - could be 'up' axis")
        if gravity_normalized[0] > 0.5:
            transform_type = "x_up_positive_gravity"
        else:
            transform_type = "x_up_negative_gravity"
    elif smallest_movement_axis == 1:  # Y has smallest movement
        print("Y-axis has smallest movement - could be 'up' axis")
        if gravity_normalized[1] > 0.5:
            transform_type = "y_up_positive_gravity"
        else:
            transform_type = "y_up_negative_gravity"
    else:  # Z has smallest movement
        print("Z-axis has smallest movement - could be 'up' axis")
        if gravity_normalized[2] > 0.5:
            transform_type = "z_up_positive_gravity"
        else:
            transform_type = "z_up_negative_gravity"

    print(f"  Detected pattern: {transform_type}")

    # Apply transformations
    transformations = {
        "original": trajectory,
        "flip_y": trajectory * np.array([1, -1, 1]),
        "flip_z": trajectory * np.array([1, 1, -1]),
        "swap_yz": trajectory[:, [0, 2, 1]],  # x, z, y
        "swap_yz_flip": trajectory[:, [0, 2, 1]] * np.array([1, 1, -1]),  # x, z, -y
        "android_to_standard": trajectory[:, [0, 2, 1]] * np.array([1, -1, 1]),  # x, -z, y
    }

    print(f"\\n=== Transformation Results ===")

    best_transform = None
    best_score = float('inf')

    for name, traj in transformations.items():
        # Calculate height variation (should be minimal for horizontal movement)
        height_range = traj[:, 2].max() - traj[:, 2].min()  # Z-axis range in standard coords
        height_std = traj[:, 2].std()

        # Calculate horizontal movement (should be significant)
        horizontal_range = np.sqrt((traj[:, 0].max() - traj[:, 0].min())**2 +
                                 (traj[:, 1].max() - traj[:, 1].min())**2)

        # Score: prefer low height variation and significant horizontal movement
        score = height_range + height_std - horizontal_range * 0.1

        print(f"  {name.upper()}:")
        print(f"    Height range: {height_range:.3f}m, std: {height_std:.3f}m")
        print(f"    Horizontal range: {horizontal_range:.3f}m")
        print(f"    Score: {score:.3f} (lower is better)")

        if score < best_score:
            best_score = score
            best_transform = name

    print(f"\\n🎯 BEST TRANSFORMATION: {best_transform.upper()}")

    # Save the best trajectory
    best_trajectory = transformations[best_transform]

    import json
    output_data = {
        'analysis': {
            'gravity_vector': gravity_vector.tolist(),
            'movement_pattern': transform_type,
            'best_transformation': best_transform,
            'best_score': best_score
        },
        'trajectories': {name: traj.tolist() for name, traj in transformations.items()},
        'best_trajectory': best_trajectory.tolist()
    }

    output_file = Path(__file__).parent / "output" / "movement_analysis.json"
    with open(output_file, 'w') as f:
        json.dump(output_data, f, indent=2)

    print(f"\\nAnalysis saved to: {output_file}")
    return best_trajectory, best_transform

if __name__ == "__main__":
    analyze_real_movement()