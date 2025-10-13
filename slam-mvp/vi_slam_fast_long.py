#!/usr/bin/env python3
"""
Fast VI-SLAM processing on longer session - optimized for speed
"""
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent / "src"))

from data_loader import SLAMDataLoader
from visual_inertial_slam import VisualInertialSLAM
from imu_processor import IMUProcessor
import numpy as np

def main():
    print("=== Fast VI-SLAM Long Session Processing ===")

    # Path to collected data
    data_path = Path(__file__).parent / "data" / "collected_data"

    # Load data
    data_loader = SLAMDataLoader(str(data_path))
    camera_matrix = data_loader.get_camera_intrinsics()

    # Initialize IMU processor
    print("\\nInitializing IMU processor...")
    imu_processor = IMUProcessor()
    imu_count = imu_processor.load_imu_data(data_loader)

    # Quick calibration
    if imu_processor.calibrate_static_bias(duration_seconds=2.0):
        print("IMU calibration successful!")

    # Initialize VI-SLAM with optimized settings
    print("\\nInitializing VI-SLAM with fast settings...")
    vi_slam = VisualInertialSLAM(camera_matrix, imu_processor)

    # Reduce feature count for speed
    import cv2
    vi_slam.orb = cv2.ORB_create(nfeatures=800)  # Faster processing

    print("\\nProcessing every 5th frame for fast VI-SLAM (50 frames total)...")

    # Process every 5th frame for speed but still get diverse motion
    frame_indices = list(range(0, min(250, data_loader.get_frame_count()), 5))
    initialization_achieved = False

    for i, frame_idx in enumerate(frame_indices):
        frame, frame_data = data_loader.get_frame(frame_idx)

        if frame is not None and frame_data is not None:
            R, t = vi_slam.process_frame(frame, frame_data['timestamp'])

            # Check initialization status
            initialized = vi_slam.is_initialized()
            scale_factor = vi_slam.get_scale_factor()

            if initialized and not initialization_achieved:
                initialization_achieved = True
                print(f"🎉 FRAME {frame_idx}: VI-SLAM INITIALIZED! Scale: {scale_factor:.6f}")

            if i % 5 == 0:
                position = t.flatten()
                print(f"Frame {frame_idx}: Position {position} (Init: {initialized})")

    # Get results
    trajectory = vi_slam.get_trajectory()
    map_points = vi_slam.get_map_points()
    map_stats = vi_slam.get_map_statistics()

    print(f"\\n=== RESULTS: Long Session VI-SLAM ===")
    print(f"Session: {data_loader.session_metadata['sessionId']}")
    print(f"Duration: {(data_loader.session_metadata['endTime'] - data_loader.session_metadata['startTime'])/1000:.1f}s")
    print(f"Total available frames: {data_loader.get_frame_count()}")
    print(f"Processed frames: {len(frame_indices)}")
    print(f"Map points generated: {len(map_points)}")
    print(f"VI initialization: {'✅ SUCCESS' if map_stats['vi_initialized'] else '❌ Failed'}")

    if map_stats['vi_initialized']:
        print(f"Scale factor: {map_stats['scale_factor']:.6f}")

        # Calculate actual distances
        if len(trajectory) > 1:
            total_distance = 0
            for i in range(1, len(trajectory)):
                prev_pos = trajectory[i-1]['position']
                curr_pos = trajectory[i]['position']
                distance = np.linalg.norm(curr_pos - prev_pos)
                total_distance += distance

            print(f"Trajectory length: {total_distance:.2f} meters (with scale correction)")

    # Export map
    if map_points:
        output_file = Path(__file__).parent / "output" / "vi_slam_long_session_map.ply"
        output_file.parent.mkdir(exist_ok=True)
        vi_slam.export_map_ply(str(output_file))
        print(f"\\nMap exported: {output_file}")

        # Quick statistics
        map_array = np.array(map_points)
        bounds = map_array.max(axis=0) - map_array.min(axis=0)
        print(f"Map bounds: {bounds[0]:.1f} x {bounds[1]:.1f} x {bounds[2]:.1f} meters")

        print(f"\\n=== COMPARISON ===")
        print(f"Previous (23.5s session): 103 points, no VI init")
        print(f"Current (52.5s session): {len(map_points)} points, VI init: {map_stats['vi_initialized']}")

    print(f"\\n=== Processing Complete ===")

if __name__ == "__main__":
    main()