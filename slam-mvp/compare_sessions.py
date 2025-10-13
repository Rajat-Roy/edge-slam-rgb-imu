#!/usr/bin/env python3
"""
Compare results between short and long sessions
"""
import numpy as np

def read_ply_stats(filename):
    """Read PLY file and get basic statistics"""
    try:
        points = []
        with open(filename, 'r') as f:
            # Skip header
            line = f.readline()
            while line.strip() != 'end_header':
                line = f.readline()
            # Read points
            for line in f:
                parts = line.strip().split()
                if len(parts) >= 3:
                    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                    points.append([x, y, z])

        if points:
            points = np.array(points)
            bounds = points.max(axis=0) - points.min(axis=0)
            volume = bounds[0] * bounds[1] * bounds[2] if all(bounds > 0) else 0
            return {
                'points': len(points),
                'bounds': bounds,
                'volume': volume,
                'center': points.mean(axis=0),
                'spread': np.std(points, axis=0)
            }
    except FileNotFoundError:
        return None

def main():
    print("=== SESSION COMPARISON ANALYSIS ===")

    # Session metadata comparison
    print("\\n📊 SESSION METADATA:")
    print("Short Session (slam_session_20250923_090335):")
    print("  Duration: 23.5 seconds")
    print("  Frames: 608")
    print("  IMU samples: 9,391")
    print("  Frame rate: 25.8 fps")
    print("  IMU rate: 398.8 Hz")

    print("\\nLong Session (slam_session_20250923_102213):")
    print("  Duration: 52.5 seconds (+123% longer)")
    print("  Frames: 1,571 (+158% more)")
    print("  IMU samples: 20,954 (+123% more)")
    print("  Frame rate: 29.9 fps")
    print("  IMU rate: 399.1 Hz")

    # Map comparison
    print("\\n🗺️  MAP POINT COMPARISON:")

    maps = [
        ("output/vi_slam_map.ply", "Short Session VI-SLAM", "23.5s"),
        ("output/long_session_demo_map.ply", "Long Session VI-SLAM", "52.5s"),
        ("output/fast_dense_map.ply", "Fast Dense Mapping", "Reference"),
    ]

    results = []
    for filename, name, duration in maps:
        stats = read_ply_stats(filename)
        if stats:
            results.append((name, duration, stats))
            print(f"\\n{name} ({duration}):")
            print(f"  Points: {stats['points']:,}")
            print(f"  Volume: {stats['volume']:.1f} m³")
            print(f"  Bounds: {stats['bounds'][0]:.1f} x {stats['bounds'][1]:.1f} x {stats['bounds'][2]:.1f} m")
            if stats['volume'] > 0:
                density = stats['points'] / stats['volume']
                print(f"  Density: {density:.4f} points/m³")
        else:
            print(f"\\n{name}: File not found")

    # Performance analysis
    if len(results) >= 2:
        short_stats = next((s for n, d, s in results if "Short" in n), None)
        long_stats = next((s for n, d, s in results if "Long" in n), None)

        if short_stats and long_stats:
            print("\\n📈 IMPROVEMENT ANALYSIS:")
            point_improvement = long_stats['points'] / short_stats['points']
            print(f"  Map points: {point_improvement:.1f}x more ({long_stats['points']} vs {short_stats['points']})")

            volume_ratio = long_stats['volume'] / short_stats['volume'] if short_stats['volume'] > 0 else 0
            if volume_ratio > 0:
                print(f"  Map volume: {volume_ratio:.1f}x larger")

            print(f"\\n✅ BENEFITS OF LONGER SESSION:")
            print(f"  • {long_stats['points'] - short_stats['points']:,} additional map points")
            print(f"  • More diverse motion patterns")
            print(f"  • Better environmental coverage")
            print(f"  • Higher chance for VI initialization")

    # Technical insights
    print("\\n🔬 TECHNICAL INSIGHTS:")
    print("\\nVI-SLAM Initialization:")
    print("  • Requires sufficient camera motion and parallax")
    print("  • Longer sessions provide more opportunities")
    print("  • IMU data helps resolve scale ambiguity")
    print("  • Need diverse 6-DOF motion patterns")

    print("\\nMap Quality Factors:")
    print("  • More frames → more feature observations")
    print("  • Longer trajectory → better triangulation geometry")
    print("  • Higher IMU rate → more accurate motion prediction")
    print("  • Diverse motion → reduced scale ambiguity")

    print("\\n🎯 RECOMMENDATIONS:")
    print("  1. For VI initialization: Need more rotational motion")
    print("  2. For dense mapping: Process more frames from long session")
    print("  3. For scale accuracy: Ensure good IMU calibration")
    print("  4. For best results: Combine long session + diverse motion")

    print("\\n=== COMPARISON COMPLETE ===")

if __name__ == "__main__":
    main()