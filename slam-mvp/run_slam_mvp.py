#!/usr/bin/env python3
"""
SLAM MVP - Run Visual-Inertial SLAM on collected Android data
"""
import sys
import os
from pathlib import Path

# Add src directory to path
sys.path.append(str(Path(__file__).parent / "src"))

from slam_pipeline import VisualInertialSLAM

def main():
    """Run SLAM MVP on collected data"""
    print("=== SLAM MVP - Visual-Inertial SLAM ===")
    print("Processing Android data collection...\n")

    # Path to collected data
    data_path = Path(__file__).parent / "data" / "collected_data"

    if not data_path.exists():
        print(f"Error: Data path not found: {data_path}")
        print("Make sure you have copied the collected data to slam-mvp/data/")
        return

    try:
        # Initialize SLAM pipeline
        slam = VisualInertialSLAM(str(data_path))

        print("\n" + "="*50)
        print("Running SLAM Pipeline...")
        print("="*50)

        # Run visual-inertial SLAM
        results = slam.run_visual_inertial()

        print("\n" + "="*50)
        print("Analyzing Results...")
        print("="*50)

        # Analyze results
        analysis = slam.analyze_results()

        # Print analysis
        for method, data in analysis.items():
            print(f"\n{method.upper()} SLAM Results:")
            print(f"  Total distance traveled: {data['total_distance']:.2f} m")
            print(f"  Maximum displacement: {data['max_displacement']:.2f} m")
            print(f"  Number of poses: {data['trajectory_length']}")
            print(f"  Start position: [{data['start_position'][0]:.2f}, {data['start_position'][1]:.2f}, {data['start_position'][2]:.2f}] m")
            print(f"  End position: [{data['end_position'][0]:.2f}, {data['end_position'][1]:.2f}, {data['end_position'][2]:.2f}] m")

        print("\n" + "="*50)
        print("Generating Visualization...")
        print("="*50)

        # Visualize results
        output_path = Path(__file__).parent / "output" / "slam_results.png"
        output_path.parent.mkdir(exist_ok=True)

        slam.visualize_results(str(output_path))

        # Export 3D map if available
        if slam.results['map_points']:
            map_file = output_path.parent / "3d_map.ply"
            slam.visual_slam.export_map_ply(str(map_file))
            print(f"3D map exported to: {map_file}")

        print(f"\nSLAM MVP completed successfully!")
        print(f"Results saved to: {output_path}")

        # Save trajectory data
        import json
        import numpy as np

        class NumpyEncoder(json.JSONEncoder):
            def default(self, obj):
                if isinstance(obj, np.ndarray):
                    return obj.tolist()
                return super().default(obj)

        results_file = output_path.parent / "slam_results.json"
        with open(results_file, 'w') as f:
            json.dump(slam.get_results(), f, indent=2, cls=NumpyEncoder)

        print(f"Trajectory data saved to: {results_file}")

    except Exception as e:
        print(f"Error running SLAM MVP: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()