#!/usr/bin/env python3
"""
Lightweight demo of VI-SLAM on longer session
"""
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent / "src"))

from data_loader import SLAMDataLoader
from visual_inertial_slam import VisualInertialSLAM
from imu_processor import IMUProcessor
import numpy as np

def main():
    print("=== VI-SLAM Longer Session Demo ===")

    # Load the longer session
    data_loader = SLAMDataLoader("data/collected_data")
    camera_matrix = data_loader.get_camera_intrinsics()

    # Setup IMU
    imu_processor = IMUProcessor()
    imu_processor.load_imu_data(data_loader)
    imu_processor.calibrate_static_bias(duration_seconds=1.0)

    # Initialize VI-SLAM with lightweight settings
    vi_slam = VisualInertialSLAM(camera_matrix, imu_processor)

    # Use fewer features for speed
    import cv2
    vi_slam.orb = cv2.ORB_create(nfeatures=500)

    print(f"\\nSession Info:")
    print(f"  Duration: {(data_loader.session_metadata['endTime'] - data_loader.session_metadata['startTime'])/1000:.1f}s")
    print(f"  Available frames: {data_loader.session_metadata['totalFrames']}")
    print(f"  IMU samples: {data_loader.session_metadata['totalImuSamples']}")

    print(f"\\nProcessing 30 frames from longer session...")

    # Process every 10th frame to get diverse motion quickly
    for i in range(0, 300, 10):
        if i >= data_loader.get_frame_count():
            break

        frame, frame_data = data_loader.get_frame(i)
        if frame is not None and frame_data is not None:
            R, t = vi_slam.process_frame(frame, frame_data['timestamp'])

            if i % 50 == 0:
                initialized = vi_slam.is_initialized()
                scale = vi_slam.get_scale_factor()
                pos = t.flatten()
                print(f"  Frame {i}: Init={initialized}, Scale={scale:.3f}, Pos={pos}")

    # Results
    map_points = vi_slam.get_map_points()
    map_stats = vi_slam.get_map_statistics()
    trajectory = vi_slam.get_trajectory()

    print(f"\\n=== RESULTS ===")
    print(f"Map points: {len(map_points)}")
    print(f"VI initialized: {map_stats['vi_initialized']}")
    print(f"Scale factor: {map_stats['scale_factor']:.6f}")
    print(f"Trajectory points: {len(trajectory)}")

    if map_stats['vi_initialized']:
        print(f"\\n🎉 SUCCESS: VI-SLAM initialized with longer session!")
        print(f"   Scale factor: {map_stats['scale_factor']:.6f}")

        # Calculate trajectory distance
        if len(trajectory) > 1:
            total_dist = sum(np.linalg.norm(trajectory[i]['position'] - trajectory[i-1]['position'])
                           for i in range(1, len(trajectory)))
            print(f"   Trajectory: {total_dist:.2f}m with scale correction")
    else:
        print(f"\\n⚠️  VI initialization needs more diverse motion")

    # Export map
    if map_points:
        output_file = Path(__file__).parent / "output" / "long_session_demo_map.ply"
        output_file.parent.mkdir(exist_ok=True)
        vi_slam.export_map_ply(str(output_file))
        print(f"\\nMap exported: {output_file}")

    print(f"\\n=== Demo Complete ===")
    print(f"Benefit: 2.2x longer session provides more opportunity for scale resolution")

if __name__ == "__main__":
    main()