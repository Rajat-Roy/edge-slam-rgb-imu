#!/usr/bin/env python3
"""
Simple text-based map visualization
"""
import numpy as np

def read_ply_points(filename):
    """Read PLY file and extract 3D points"""
    points = []

    with open(filename, 'r') as f:
        # Skip header
        line = f.readline()
        while line.strip() != 'end_header':
            line = f.readline()

        # Read point data
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 3:
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                points.append([x, y, z])

    return np.array(points)

def show_map_statistics(points):
    """Show detailed map statistics"""
    print(f"=== 3D Map Visualization ===")
    print(f"Total points: {len(points)}")

    if len(points) == 0:
        print("No points to display")
        return

    # Calculate bounds
    min_vals = points.min(axis=0)
    max_vals = points.max(axis=0)
    ranges = max_vals - min_vals

    print(f"\nMap Bounds:")
    print(f"  X: [{min_vals[0]:.2f}, {max_vals[0]:.2f}] (range: {ranges[0]:.2f}m)")
    print(f"  Y: [{min_vals[1]:.2f}, {max_vals[1]:.2f}] (range: {ranges[1]:.2f}m)")
    print(f"  Z: [{min_vals[2]:.2f}, {max_vals[2]:.2f}] (range: {ranges[2]:.2f}m)")

    # Calculate statistics
    center = (min_vals + max_vals) / 2
    distances_from_center = np.linalg.norm(points - center, axis=1)

    print(f"\nPoint Distribution:")
    print(f"  Center: ({center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f})")
    print(f"  Average distance from center: {distances_from_center.mean():.2f}m")
    print(f"  Max distance from center: {distances_from_center.max():.2f}m")
    print(f"  Point density: {len(points)/(ranges[0]*ranges[1]*ranges[2]):.3f} points/m³")

def create_2d_projection(points, view='top'):
    """Create 2D projection of 3D points"""
    if len(points) == 0:
        return

    print(f"\n=== {view.upper()} VIEW (2D Projection) ===")

    if view == 'top':  # X-Y plane (looking down)
        x_coords = points[:, 0]
        y_coords = points[:, 1]
        xlabel, ylabel = 'X (meters)', 'Y (meters)'
    elif view == 'front':  # X-Z plane (looking from front)
        x_coords = points[:, 0]
        y_coords = points[:, 2]
        xlabel, ylabel = 'X (meters)', 'Z (meters)'
    else:  # side view, Y-Z plane
        x_coords = points[:, 1]
        y_coords = points[:, 2]
        xlabel, ylabel = 'Y (meters)', 'Z (meters)'

    # Create ASCII plot
    width, height = 60, 20

    # Normalize coordinates to fit in ASCII grid
    x_min, x_max = x_coords.min(), x_coords.max()
    y_min, y_max = y_coords.min(), y_coords.max()

    if x_max == x_min:
        x_max = x_min + 1
    if y_max == y_min:
        y_max = y_min + 1

    x_norm = ((x_coords - x_min) / (x_max - x_min) * (width - 1)).astype(int)
    y_norm = ((y_coords - y_min) / (y_max - y_min) * (height - 1)).astype(int)

    # Create grid
    grid = [[' ' for _ in range(width)] for _ in range(height)]

    # Plot points
    for x, y in zip(x_norm, y_norm):
        if 0 <= x < width and 0 <= y < height:
            grid[height-1-y][x] = '●'  # Flip Y for proper display

    # Print grid with labels
    print(f"  {ylabel}")
    print(f"  ↑")
    for i, row in enumerate(grid):
        if i == 0:
            print(f"{y_max:6.1f}│{''.join(row)}")
        elif i == height-1:
            print(f"{y_min:6.1f}│{''.join(row)}")
        else:
            print(f"      │{''.join(row)}")

    print(f"      └{'─' * width}→ {xlabel}")
    print(f"       {x_min:.1f}{' ' * (width-12)}{x_max:.1f}")

def show_point_clusters(points):
    """Show point clustering information"""
    if len(points) < 10:
        return

    print(f"\n=== Point Cloud Analysis ===")

    # Calculate distances between all points
    distances = []
    for i in range(len(points)):
        for j in range(i+1, len(points)):
            dist = np.linalg.norm(points[i] - points[j])
            distances.append(dist)

    distances = np.array(distances)

    print(f"Inter-point distances:")
    print(f"  Minimum: {distances.min():.3f}m")
    print(f"  Average: {distances.mean():.3f}m")
    print(f"  Maximum: {distances.max():.3f}m")
    print(f"  Median: {np.median(distances):.3f}m")

    # Find clusters (simple distance-based)
    close_pairs = np.sum(distances < 1.0)  # Points within 1m
    total_pairs = len(distances)

    print(f"  Point pairs within 1m: {close_pairs}/{total_pairs} ({100*close_pairs/total_pairs:.1f}%)")

def main():
    # Show VI-SLAM map
    print("Loading VI-SLAM map...")
    try:
        points = read_ply_points('output/vi_slam_map.ply')
        print(f"\n🗺️  VI-SLAM MAP VISUALIZATION")
        show_map_statistics(points)
        create_2d_projection(points, 'top')
        create_2d_projection(points, 'front')
        show_point_clusters(points)
    except FileNotFoundError:
        print("VI-SLAM map file not found")

    # Compare with previous maps
    print(f"\n" + "="*60)
    print(f"COMPARISON WITH PREVIOUS MAPS")

    maps_to_compare = [
        ('output/fast_dense_map.ply', 'Fast Dense Mapping'),
        ('output/dense_map.ply', 'Dense Mapping'),
        ('output/test_map.ply', 'Test Mapping')
    ]

    for filename, name in maps_to_compare:
        try:
            points = read_ply_points(filename)
            print(f"\n📍 {name}: {len(points)} points")
            if len(points) > 0:
                ranges = points.max(axis=0) - points.min(axis=0)
                volume = ranges[0] * ranges[1] * ranges[2]
                density = len(points) / volume if volume > 0 else 0
                print(f"   Volume: {volume:.1f}m³, Density: {density:.3f} pts/m³")
        except FileNotFoundError:
            print(f"\n📍 {name}: File not found")

if __name__ == "__main__":
    main()