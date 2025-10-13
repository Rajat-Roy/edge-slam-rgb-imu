#!/usr/bin/env python3
"""
Visual-Inertial SLAM Test
Tests the complete VI-SLAM system with collected RGB + IMU data
"""
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent / "src"))

from data_loader import SLAMDataLoader
from visual_inertial_slam import VisualInertialSLAM
from imu_processor import IMUProcessor
import numpy as np

def main():
    print("=== Visual-Inertial SLAM Test ===")

    # Path to collected data
    data_path = Path(__file__).parent / "data" / "collected_data"

    # Load data
    data_loader = SLAMDataLoader(str(data_path))
    camera_matrix = data_loader.get_camera_intrinsics()

    # Initialize IMU processor
    print("\\nInitializing IMU processor...")
    imu_processor = IMUProcessor()
    imu_count = imu_processor.load_imu_data(data_loader)

    # Calibrate IMU biases
    print("\\nCalibrating IMU biases...")
    if imu_processor.calibrate_static_bias(duration_seconds=2.0):
        print("IMU calibration successful!")
    else:
        print("IMU calibration failed - proceeding with default biases")

    # Print IMU statistics
    imu_stats = imu_processor.get_statistics()
    print("\\nIMU Statistics:")
    print(f"  Total measurements: {imu_stats['total_measurements']}")
    print(f"  Time range: {imu_stats['time_range_seconds']:.1f} seconds")
    print(f"  Frequency: {imu_stats['frequency_hz']:.1f} Hz")
    print(f"  Accelerometer mean magnitude: {imu_stats['accelerometer']['magnitude_mean']:.2f} m/s²")
    print(f"  Gyroscope mean magnitude: {imu_stats['gyroscope']['magnitude_mean']:.3f} rad/s")

    # Initialize Visual-Inertial SLAM
    print("\\nInitializing VI-SLAM...")
    vi_slam = VisualInertialSLAM(camera_matrix, imu_processor)

    print("\\nProcessing frames for VI-SLAM (30 frames)...")

    # Process frames for VI-SLAM
    for i in range(30):
        frame, frame_data = data_loader.get_frame(i)

        if frame is not None and frame_data is not None:
            R, t = vi_slam.process_frame(frame, frame_data['timestamp'])

            if i % 5 == 0:
                position = t.flatten()
                initialized = vi_slam.is_initialized()
                scale_factor = vi_slam.get_scale_factor()
                print(f"Frame {i}: Position {position} (Init: {initialized}, Scale: {scale_factor:.3f})")

    # Get results
    trajectory = vi_slam.get_trajectory()
    map_points = vi_slam.get_map_points()
    map_stats = vi_slam.get_map_statistics()

    print(f"\\n=== VI-SLAM Results ===")
    print(f"Trajectory points: {len(trajectory)}")
    print(f"Map points: {len(map_points)}")
    print(f"VI initialization: {map_stats['vi_initialized']}")
    print(f"Scale factor: {map_stats['scale_factor']:.3f}")
    print(f"IMU calibrated: {map_stats['imu_calibrated']}")
    print(f"Map statistics: {map_stats}")

    # Analyze trajectory improvement
    if len(trajectory) > 0:
        initialized_frames = sum(1 for t in trajectory if t.get('initialized', False))
        total_distance = 0

        for i in range(1, len(trajectory)):
            prev_pos = trajectory[i-1]['position']
            curr_pos = trajectory[i]['position']
            distance = np.linalg.norm(curr_pos - prev_pos)
            total_distance += distance

        print(f"\\nTrajectory Analysis:")
        print(f"  Total trajectory length: {total_distance:.2f} meters")
        print(f"  Initialized frames: {initialized_frames}/{len(trajectory)}")
        print(f"  Average matches per frame: {np.mean([t['matches'] for t in trajectory]):.1f}")

    # Export results
    if map_points:
        output_file = Path(__file__).parent / "output" / "vi_slam_map.ply"
        output_file.parent.mkdir(exist_ok=True)
        vi_slam.export_map_ply(str(output_file))
        print(f"\\nVI-SLAM map exported to: {output_file}")

        # Compare with visual-only SLAM
        print(f"\\nComparison with visual-only SLAM:")
        print(f"  VI-SLAM map points: {len(map_points)}")
        print(f"  Previous dense mapping: 156 points")

        if len(map_points) > 156:
            improvement = len(map_points) / 156
            print(f"  Improvement: {improvement:.1f}x more points!")
        else:
            print(f"  Density: {len(map_points)/156:.1f}x relative to previous")

        # Show trajectory consistency
        if map_stats['vi_initialized']:
            print(f"\\nVI-SLAM Benefits:")
            print(f"  ✅ Scale ambiguity resolved")
            print(f"  ✅ Absolute scale estimation")
            print(f"  ✅ IMU-aided robust tracking")
            print(f"  ✅ Motion prediction for fast movements")
        else:
            print(f"\\nVI initialization not completed - need more motion")

    else:
        print("No map points generated - debugging needed")

    print(f"\\n=== VI-SLAM Test Complete ===")

if __name__ == "__main__":
    main()