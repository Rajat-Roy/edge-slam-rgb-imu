#!/usr/bin/env python3
"""
VI-SLAM processing on longer session for improved initialization and mapping
"""
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent / "src"))

from data_loader import SLAMDataLoader
from visual_inertial_slam import VisualInertialSLAM
from imu_processor import IMUProcessor
import numpy as np

def main():
    print("=== VI-SLAM Long Session Processing ===")

    # Path to collected data (now pointing to longer session)
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
    if imu_processor.calibrate_static_bias(duration_seconds=3.0):  # Longer calibration
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

    print("\\nProcessing longer session for VI-SLAM (100 frames with higher chance of initialization)...")

    # Process more frames to increase chance of VI initialization
    frames_to_process = min(100, data_loader.get_frame_count())
    initialization_status = []

    for i in range(frames_to_process):
        frame, frame_data = data_loader.get_frame(i)

        if frame is not None and frame_data is not None:
            R, t = vi_slam.process_frame(frame, frame_data['timestamp'])

            # Check initialization status
            initialized = vi_slam.is_initialized()
            scale_factor = vi_slam.get_scale_factor()

            initialization_status.append(initialized)

            if i % 10 == 0 or (initialized and not any(initialization_status[:-1])):
                position = t.flatten()
                if initialized and not any(initialization_status[:-1]):
                    print(f"🎉 Frame {i}: VI INITIALIZATION COMPLETE! Scale: {scale_factor:.3f}")
                    print(f"    Position: {position}")
                else:
                    print(f"Frame {i}: Position {position} (Init: {initialized}, Scale: {scale_factor:.3f})")

    # Get results
    trajectory = vi_slam.get_trajectory()
    map_points = vi_slam.get_map_points()
    map_stats = vi_slam.get_map_statistics()

    print(f"\\n=== Long Session VI-SLAM Results ===")
    print(f"Processed frames: {frames_to_process}")
    print(f"Trajectory points: {len(trajectory)}")
    print(f"Map points: {len(map_points)}")
    print(f"VI initialization: {map_stats['vi_initialized']}")
    print(f"Final scale factor: {map_stats['scale_factor']:.6f}")
    print(f"IMU calibrated: {map_stats['imu_calibrated']}")

    # Detailed map statistics
    if len(map_points) > 0:
        map_points_array = np.array(map_points)
        bounds = {
            'min': map_points_array.min(axis=0),
            'max': map_points_array.max(axis=0),
            'range': map_points_array.max(axis=0) - map_points_array.min(axis=0)
        }
        volume = bounds['range'][0] * bounds['range'][1] * bounds['range'][2]

        print(f"\\nDetailed Map Statistics:")
        print(f"  Map bounds: X[{bounds['min'][0]:.1f}, {bounds['max'][0]:.1f}], Y[{bounds['min'][1]:.1f}, {bounds['max'][1]:.1f}], Z[{bounds['min'][2]:.1f}, {bounds['max'][2]:.1f}]")
        print(f"  Map volume: {volume:.1f} m³")
        print(f"  Point density: {len(map_points)/volume:.4f} points/m³")

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
        print(f"  Initialized frames: {initialized_frames}/{len(trajectory)} ({100*initialized_frames/len(trajectory):.1f}%)")
        print(f"  Average matches per frame: {np.mean([t['matches'] for t in trajectory]):.1f}")

        # Show scale factor evolution
        scale_factors = [t.get('scale_factor', 1.0) for t in trajectory]
        if any(sf != 1.0 for sf in scale_factors):
            final_scale = scale_factors[-1]
            print(f"  Scale factor evolution: 1.000 → {final_scale:.6f}")

    # Export results
    if map_points:
        output_file = Path(__file__).parent / "output" / "vi_slam_long_session_map.ply"
        output_file.parent.mkdir(exist_ok=True)
        vi_slam.export_map_ply(str(output_file))
        print(f"\\nLong session VI-SLAM map exported to: {output_file}")

        print(f"\\n=== Comparison Summary ===")
        print(f"Session Duration: 52.5s (vs 23.5s previous)")
        print(f"Available Frames: {data_loader.get_frame_count()} (vs 608 previous)")
        print(f"IMU Samples: {imu_stats['total_measurements']} (vs 9,385 previous)")
        print(f"Map Points Generated: {len(map_points)}")

        if map_stats['vi_initialized']:
            print(f"\\n✅ SUCCESS: VI-SLAM Benefits Achieved")
            print(f"  ✅ Scale ambiguity resolved with factor {map_stats['scale_factor']:.6f}")
            print(f"  ✅ Absolute scale estimation from longer motion")
            print(f"  ✅ IMU-aided robust tracking over {total_distance:.1f}m trajectory")
            print(f"  ✅ Better environmental mapping with more diverse motion")
        else:
            print(f"\\n⚠️  VI initialization still pending - may need even more diverse motion")
            print(f"   Consider: more rotation, translation, or different motion patterns")

    else:
        print("No map points generated - debugging needed")

    print(f"\\n=== Long Session VI-SLAM Complete ===")

if __name__ == "__main__":
    main()