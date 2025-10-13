#!/usr/bin/env python3
"""
Debug coordinate system and orientation issues
"""
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent / "src"))

from data_loader import SLAMDataLoader
from visual_inertial_slam import VisualInertialSLAM
from imu_processor import IMUProcessor
import numpy as np

def analyze_coordinate_system():
    """Analyze coordinate system issues"""
    print("=== Coordinate System Analysis ===")

    # Load data
    data_loader = SLAMDataLoader("data/collected_data")
    camera_matrix = data_loader.get_camera_intrinsics()

    # Setup IMU
    imu_processor = IMUProcessor()
    imu_processor.load_imu_data(data_loader)
    imu_processor.calibrate_static_bias(duration_seconds=1.0)

    # Get IMU statistics
    imu_stats = imu_processor.get_statistics()
    print("\\nIMU Analysis:")
    print(f"  Accelerometer mean: {imu_stats['accelerometer']['mean']}")
    print(f"  Accelerometer magnitude: {imu_stats['accelerometer']['magnitude_mean']:.3f} m/s²")
    print(f"  Expected gravity: ~9.81 m/s²")

    # Check gravity direction
    acc_mean = imu_stats['accelerometer']['mean']
    gravity_magnitude = np.linalg.norm(acc_mean)
    gravity_direction = acc_mean / gravity_magnitude

    print(f"\\nGravity Analysis:")
    print(f"  Gravity vector: [{gravity_direction[0]:.3f}, {gravity_direction[1]:.3f}, {gravity_direction[2]:.3f}]")
    print(f"  Magnitude: {gravity_magnitude:.3f} m/s²")

    # Determine which axis is "up"
    max_axis = np.argmax(np.abs(gravity_direction))
    axis_names = ['X', 'Y', 'Z']
    print(f"  Dominant gravity axis: {axis_names[max_axis]} ({'positive' if gravity_direction[max_axis] > 0 else 'negative'})")

    # Android coordinate system info
    print(f"\\nAndroid Coordinate System:")
    print(f"  X: Right (when holding phone upright)")
    print(f"  Y: Up (toward top of phone)")
    print(f"  Z: Out (toward user, away from screen)")
    print(f"  Gravity points DOWN in Y direction (-Y)")

    # Initialize SLAM
    vi_slam = VisualInertialSLAM(camera_matrix, imu_processor)
    import cv2
    vi_slam.orb = cv2.ORB_create(nfeatures=300)

    print(f"\\nProcessing first 10 frames to analyze movement...")

    # Process frames and analyze trajectory
    trajectory_raw = []
    for i in range(0, 50, 5):
        frame, frame_data = data_loader.get_frame(i)
        if frame is not None and frame_data is not None:
            R, t = vi_slam.process_frame(frame, frame_data['timestamp'])
            position = t.flatten()
            trajectory_raw.append(position.copy())

            if i % 10 == 0:
                print(f"  Frame {i}: Position [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]")

    # Analyze trajectory direction
    if len(trajectory_raw) > 1:
        print(f"\\nTrajectory Analysis:")

        # Calculate movement in each axis
        start_pos = np.array(trajectory_raw[0])
        end_pos = np.array(trajectory_raw[-1])
        total_movement = end_pos - start_pos

        print(f"  Start position: [{start_pos[0]:.3f}, {start_pos[1]:.3f}, {start_pos[2]:.3f}]")
        print(f"  End position: [{end_pos[0]:.3f}, {end_pos[1]:.3f}, {end_pos[2]:.3f}]")
        print(f"  Net movement: [{total_movement[0]:.3f}, {total_movement[1]:.3f}, {total_movement[2]:.3f}]")

        # Check if Z is going down (common issue)
        if total_movement[2] < -1.0:
            print(f"  ⚠️  Z-axis going down significantly ({total_movement[2]:.3f}m)")
            print(f"      This suggests coordinate system mismatch")

        # Suggest coordinate transformation
        print(f"\\nCoordinate System Fixes:")
        print(f"  1. Android uses Y-up, but camera might use Z-up")
        print(f"  2. Try flipping Z-axis: z_corrected = -z_original")
        print(f"  3. Try swapping axes: [x, y, z] -> [x, z, -y]")

    return trajectory_raw, gravity_direction

def create_corrected_trajectory(trajectory_raw, gravity_direction):
    """Create corrected trajectory with proper orientation"""
    print(f"\\n=== Creating Corrected Trajectory ===")

    # Method 1: Simple Z-flip
    trajectory_z_flip = []
    for pos in trajectory_raw:
        corrected = [pos[0], pos[1], -pos[2]]  # Flip Z
        trajectory_z_flip.append(corrected)

    # Method 2: Coordinate system conversion (Android to standard)
    # Android: X=right, Y=up, Z=out -> Standard: X=right, Y=forward, Z=up
    trajectory_std = []
    for pos in trajectory_raw:
        # Convert: [x, y, z] -> [x, z, -y]
        corrected = [pos[0], pos[2], -pos[1]]
        trajectory_std.append(corrected)

    # Method 3: Gravity-aligned correction
    trajectory_gravity = []
    if np.abs(gravity_direction[1]) > 0.7:  # Y is dominant (typical for phone)
        for pos in trajectory_raw:
            # Keep X, flip Y (up/down), keep Z
            corrected = [pos[0], -pos[1], pos[2]]
            trajectory_gravity.append(corrected)
    else:
        trajectory_gravity = trajectory_raw

    return {
        'original': trajectory_raw,
        'z_flip': trajectory_z_flip,
        'coordinate_conversion': trajectory_std,
        'gravity_aligned': trajectory_gravity
    }

def main():
    trajectory_raw, gravity_direction = analyze_coordinate_system()

    if len(trajectory_raw) > 1:
        corrected_trajectories = create_corrected_trajectory(trajectory_raw, gravity_direction)

        print(f"\\n=== Trajectory Corrections ===")

        for method, traj in corrected_trajectories.items():
            start = np.array(traj[0])
            end = np.array(traj[-1])
            movement = end - start

            print(f"\\n{method.upper()}:")
            print(f"  Net movement: [{movement[0]:.3f}, {movement[1]:.3f}, {movement[2]:.3f}]")

            # Check if trajectory looks more reasonable
            if movement[2] > -0.5:  # Z not going down much
                print(f"  ✅ Z-axis looks better!")
            else:
                print(f"  ❌ Still going down in Z")

        # Save corrected trajectory data
        import json
        output_data = {
            'analysis': {
                'gravity_direction': gravity_direction.tolist(),
                'raw_movement': (np.array(trajectory_raw[-1]) - np.array(trajectory_raw[0])).tolist()
            },
            'trajectories': corrected_trajectories
        }

        output_file = Path(__file__).parent / "output" / "corrected_trajectories.json"
        with open(output_file, 'w') as f:
            json.dump(output_data, f, indent=2)

        print(f"\\nCorrected trajectories saved to: {output_file}")

if __name__ == "__main__":
    main()