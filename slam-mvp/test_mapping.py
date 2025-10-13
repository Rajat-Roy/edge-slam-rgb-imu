#!/usr/bin/env python3
"""
Quick test of 3D mapping functionality
"""
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent / "src"))

from data_loader import SLAMDataLoader
from visual_slam import VisualSLAM
import numpy as np

def main():
    print("=== Testing 3D Mapping ===")

    # Path to collected data
    data_path = Path(__file__).parent / "data" / "collected_data"

    # Load data
    data_loader = SLAMDataLoader(str(data_path))
    camera_matrix = data_loader.get_camera_intrinsics()

    # Initialize visual SLAM
    visual_slam = VisualSLAM(camera_matrix)

    print("Processing 100 frames for dense mapping...")

    # Process more frames for denser mapping
    for i in range(100):
        frame, frame_data = data_loader.get_frame(i)

        if frame is not None and frame_data is not None:
            R, t = visual_slam.process_frame(frame, frame_data['timestamp'])
            print(f"Frame {i}: Position {t.flatten()}")

    # Get mapping results
    map_points = visual_slam.get_map_points()
    map_stats = visual_slam.get_map_statistics()

    print(f"\nMapping Results:")
    print(f"Map points: {len(map_points)}")
    print(f"Statistics: {map_stats}")

    if map_points:
        # Save simple point cloud
        output_file = Path(__file__).parent / "output" / "test_map.ply"
        output_file.parent.mkdir(exist_ok=True)
        visual_slam.export_map_ply(str(output_file))
    else:
        print("No map points generated - debugging needed")

if __name__ == "__main__":
    main()