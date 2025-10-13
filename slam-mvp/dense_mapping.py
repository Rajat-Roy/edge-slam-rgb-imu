#!/usr/bin/env python3
"""
Dense mapping with optimized parameters
"""
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent / "src"))

from data_loader import SLAMDataLoader
from visual_slam import VisualSLAM
import numpy as np

def main():
    print("=== Dense 3D Mapping ===")

    # Path to collected data
    data_path = Path(__file__).parent / "data" / "collected_data"

    # Load data
    data_loader = SLAMDataLoader(str(data_path))
    camera_matrix = data_loader.get_camera_intrinsics()

    # Initialize visual SLAM with optimized dense mapping settings
    visual_slam = VisualSLAM(camera_matrix)
    import cv2
    visual_slam.orb = cv2.ORB_create(nfeatures=1500)  # Balanced feature count

    # Make mapping parameters more permissive for denser points
    visual_slam.mapper.min_triangulation_angle = 1.5  # More permissive angle
    visual_slam.mapper.max_reprojection_error = 6.0  # More permissive error
    visual_slam.mapper.min_track_length = 2  # Keep minimum

    print("Processing every 3rd frame for balanced density (67 frames total)...")

    # Process every 3rd frame for balanced performance and density
    for i in range(0, 200, 3):  # Every 3rd frame up to frame 200
        frame, frame_data = data_loader.get_frame(i)

        if frame is not None and frame_data is not None:
            R, t = visual_slam.process_frame(frame, frame_data['timestamp'])
            if i % 10 == 0:
                print(f"Frame {i}: Position {t.flatten()}")

    # Get mapping results
    map_points = visual_slam.get_map_points()
    map_stats = visual_slam.get_map_statistics()

    print(f"\nDense Mapping Results:")
    print(f"Map points: {len(map_points)}")
    print(f"Statistics: {map_stats}")

    if map_points:
        # Save dense point cloud
        output_file = Path(__file__).parent / "output" / "dense_map.ply"
        output_file.parent.mkdir(exist_ok=True)
        visual_slam.export_map_ply(str(output_file))
        print(f"Dense map exported to: {output_file}")

        # Show density improvement
        print(f"\nDensity comparison:")
        print(f"  Previous (50 frames): 100 points")
        print(f"  Current (67 frames): {len(map_points)} points")
        print(f"  Improvement: {len(map_points)/100:.1f}x denser")
    else:
        print("No map points generated - debugging needed")

if __name__ == "__main__":
    main()