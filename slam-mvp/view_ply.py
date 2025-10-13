#!/usr/bin/env python3
"""
Quick PLY point cloud viewer using matplotlib
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def read_ply(filename):
    """Read PLY file and extract points"""
    points = []
    colors = []

    with open(filename, 'r') as f:
        # Skip header
        line = f.readline()
        while line.strip() != 'end_header':
            line = f.readline()

        # Read point data
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 6:
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                r, g, b = int(parts[3]), int(parts[4]), int(parts[5])
                points.append([x, y, z])
                colors.append([r/255.0, g/255.0, b/255.0])

    return np.array(points), np.array(colors)

def view_ply(filename):
    """View PLY point cloud"""
    print(f"Loading PLY file: {filename}")
    points, colors = read_ply(filename)

    print(f"Loaded {len(points)} points")
    print(f"Point cloud bounds:")
    print(f"  X: [{points[:, 0].min():.2f}, {points[:, 0].max():.2f}]")
    print(f"  Y: [{points[:, 1].min():.2f}, {points[:, 1].max():.2f}]")
    print(f"  Z: [{points[:, 2].min():.2f}, {points[:, 2].max():.2f}]")

    # Create 3D plot
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot points
    ax.scatter(points[:, 0], points[:, 1], points[:, 2],
              c=colors, s=20, alpha=0.8)

    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_zlabel('Z (meters)')
    ax.set_title(f'3D Environmental Map ({len(points)} points)')

    # Equal aspect ratio
    max_range = np.array([points[:, 0].max()-points[:, 0].min(),
                         points[:, 1].max()-points[:, 1].min(),
                         points[:, 2].max()-points[:, 2].min()]).max() / 2.0

    mid_x = (points[:, 0].max()+points[:, 0].min()) * 0.5
    mid_y = (points[:, 1].max()+points[:, 1].min()) * 0.5
    mid_z = (points[:, 2].max()+points[:, 2].min()) * 0.5

    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    view_ply("output/fast_dense_map.ply")