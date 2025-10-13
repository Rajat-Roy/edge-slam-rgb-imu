#!/usr/bin/env python3
"""
Fast dense mapping with optimized parameters
"""
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent / "src"))

from data_loader import SLAMDataLoader
from visual_slam import VisualSLAM
import numpy as np

def main():
    print("=== Fast Dense 3D Mapping ===")

    # Path to collected data
    data_path = Path(__file__).parent / "data" / "collected_data"

    # Load data
    data_loader = SLAMDataLoader(str(data_path))
    camera_matrix = data_loader.get_camera_intrinsics()

    # Initialize visual SLAM with fast dense mapping settings
    visual_slam = VisualSLAM(camera_matrix)
    import cv2
    visual_slam.orb = cv2.ORB_create(nfeatures=800)  # Moderate feature count

    # Optimize mapping parameters for speed and density
    visual_slam.mapper.min_triangulation_angle = 1.0  # More permissive
    visual_slam.mapper.max_reprojection_error = 4.0  # Reasonable threshold
    visual_slam.mapper.min_track_length = 2  # Minimum

    print("Processing every 5th frame for fast dense mapping (40 frames total)...")

    # Process every 5th frame for good coverage with speed
    processed_frames = 0
    for i in range(0, 200, 5):  # Every 5th frame up to frame 200
        frame, frame_data = data_loader.get_frame(i)

        if frame is not None and frame_data is not None:
            R, t = visual_slam.process_frame(frame, frame_data['timestamp'])
            if processed_frames % 5 == 0:
                print(f"Frame {i}: Position {t.flatten()}")
            processed_frames += 1

    # Get mapping results
    map_points = visual_slam.get_map_points()
    map_stats = visual_slam.get_map_statistics()

    print(f"\nFast Dense Mapping Results:")
    print(f"Map points: {len(map_points)}")
    print(f"Processed frames: {processed_frames}")
    print(f"Statistics: {map_stats}")

    if map_points:
        # Save dense point cloud
        output_file = Path(__file__).parent / "output" / "fast_dense_map.ply"
        output_file.parent.mkdir(exist_ok=True)
        visual_slam.export_map_ply(str(output_file))
        print(f"Fast dense map exported to: {output_file}")

        # Show density improvement
        print(f"\nDensity comparison:")
        print(f"  Previous result: 100 points")
        print(f"  Fast dense (40 frames): {len(map_points)} points")
        print(f"  Points per frame: {len(map_points)/processed_frames:.1f}")

        if len(map_points) > 100:
            print(f"  SUCCESS: {len(map_points)/100:.1f}x more points!")
        else:
            print(f"  Still working on optimization...")
    else:
        print("No map points generated - debugging needed")

if __name__ == "__main__":
    main()