"""
Simple Visual-Inertial SLAM Pipeline
Combines visual odometry with IMU integration
"""
import numpy as np
import cv2
from typing import Dict, List, Tuple
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from data_loader import SLAMDataLoader
from visual_odometry import VisualOdometry
from visual_slam import VisualSLAM
from imu_integration import IMUIntegration

class VisualInertialSLAM:
    def __init__(self, data_path: str):
        """
        Initialize Visual-Inertial SLAM pipeline

        Args:
            data_path: Path to SLAM dataset
        """
        self.data_loader = SLAMDataLoader(data_path)

        # Get camera intrinsics
        self.camera_matrix = self.data_loader.get_camera_intrinsics()
        print(f"Camera matrix:\n{self.camera_matrix}")

        # Initialize subsystems
        self.visual_odometry = VisualOdometry(self.camera_matrix)
        self.visual_slam = VisualSLAM(self.camera_matrix)  # Enhanced SLAM with mapping
        self.imu_integration = IMUIntegration()

        # Results
        self.results = {
            'visual_trajectory': [],
            'visual_slam_trajectory': [],
            'map_points': [],
            'imu_trajectory': [],
            'fused_trajectory': [],
            'timestamps': []
        }

    def run_visual_only(self) -> Dict:
        """
        Run visual-only SLAM (pure visual odometry)

        Returns:
            Dictionary with visual odometry results
        """
        print("Running visual-only SLAM...")

        frame_count = self.data_loader.get_frame_count()
        visual_trajectory = []

        for i in range(min(frame_count, 50)):  # Process first 50 frames for demo
            frame, frame_data = self.data_loader.get_frame(i)

            if frame is not None and frame_data is not None:
                R, t = self.visual_odometry.process_frame(frame, frame_data['timestamp'])

                visual_trajectory.append({
                    'timestamp': frame_data['timestamp'],
                    'position': t.flatten().copy(),
                    'rotation': R.copy(),
                    'frame_number': frame_data['frameNumber']
                })

                if i % 10 == 0:
                    print(f"Processed frame {i}/{frame_count}")

        self.results['visual_trajectory'] = visual_trajectory
        print(f"Visual odometry completed: {len(visual_trajectory)} poses")
        return visual_trajectory

    def run_visual_slam(self) -> Dict:
        """
        Run enhanced visual SLAM with 3D mapping

        Returns:
            Dictionary with visual SLAM results including 3D map
        """
        print("Running enhanced visual SLAM with 3D mapping...")

        frame_count = self.data_loader.get_frame_count()
        visual_slam_trajectory = []

        for i in range(min(frame_count, 50)):  # Process first 50 frames for demo
            frame, frame_data = self.data_loader.get_frame(i)

            if frame is not None and frame_data is not None:
                R, t = self.visual_slam.process_frame(frame, frame_data['timestamp'])

                visual_slam_trajectory.append({
                    'timestamp': frame_data['timestamp'],
                    'position': t.flatten().copy(),
                    'rotation': R.copy(),
                    'frame_number': frame_data['frameNumber']
                })

                if i % 10 == 0:
                    print(f"Processed frame {i}/{frame_count}")

        # Get 3D map points
        map_points = self.visual_slam.get_map_points()
        map_stats = self.visual_slam.get_map_statistics()

        self.results['visual_slam_trajectory'] = visual_slam_trajectory
        self.results['map_points'] = map_points

        print(f"Visual SLAM completed: {len(visual_slam_trajectory)} poses, {len(map_points)} map points")
        print(f"Map statistics: {map_stats}")

        return {
            'trajectory': visual_slam_trajectory,
            'map_points': map_points,
            'statistics': map_stats
        }

    def run_imu_only(self) -> Dict:
        """
        Run IMU-only integration

        Returns:
            Dictionary with IMU integration results
        """
        print("Running IMU-only integration...")

        # Get all IMU data
        accel_data = self.data_loader.accel_data
        gyro_data = self.data_loader.gyro_data

        # Integrate IMU data
        self.imu_integration.integrate_sequence(accel_data, gyro_data)

        imu_trajectory = self.imu_integration.get_trajectory()
        self.results['imu_trajectory'] = imu_trajectory

        print(f"IMU integration completed: {len(imu_trajectory)} poses")
        return imu_trajectory

    def run_visual_inertial(self) -> Dict:
        """
        Run visual-inertial SLAM with simple fusion

        Returns:
            Dictionary with fused SLAM results
        """
        print("Running visual-inertial SLAM...")

        # First run individual systems
        visual_traj = self.run_visual_only()
        visual_slam_results = self.run_visual_slam()
        imu_traj = self.run_imu_only()

        # Simple fusion: combine visual and IMU estimates
        fused_trajectory = []

        for i, visual_pose in enumerate(visual_traj):
            timestamp = visual_pose['timestamp']

            # Find closest IMU pose
            imu_pose = None
            min_time_diff = float('inf')

            for imu_p in imu_traj:
                time_diff = abs(imu_p['timestamp'] - timestamp)
                if time_diff < min_time_diff:
                    min_time_diff = time_diff
                    imu_pose = imu_p

            if imu_pose is not None:
                # Simple fusion: weighted average of positions
                # In practice, this would be done with a Kalman filter or similar
                visual_pos = visual_pose['position']
                imu_pos = imu_pose['position']

                # Weight visual odometry more for position (IMU drifts)
                fused_pos = 0.7 * visual_pos + 0.3 * imu_pos

                fused_trajectory.append({
                    'timestamp': timestamp,
                    'position': fused_pos,
                    'visual_position': visual_pos.copy(),
                    'imu_position': imu_pos.copy(),
                    'frame_number': visual_pose['frame_number']
                })

        self.results['fused_trajectory'] = fused_trajectory
        print(f"Visual-inertial fusion completed: {len(fused_trajectory)} poses")

        return {
            'visual': visual_traj,
            'visual_slam': visual_slam_results,
            'imu': imu_traj,
            'fused': fused_trajectory
        }

    def analyze_results(self) -> Dict:
        """
        Analyze SLAM results and compute metrics

        Returns:
            Dictionary with analysis results
        """
        analysis = {}

        if self.results['visual_trajectory']:
            visual_traj = self.results['visual_trajectory']
            positions = np.array([pose['position'] for pose in visual_traj])

            analysis['visual'] = {
                'total_distance': np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1)),
                'max_displacement': np.max(np.linalg.norm(positions, axis=1)),
                'trajectory_length': len(visual_traj),
                'start_position': positions[0],
                'end_position': positions[-1]
            }

        if self.results['imu_trajectory']:
            imu_traj = self.results['imu_trajectory']
            positions = np.array([pose['position'] for pose in imu_traj])

            analysis['imu'] = {
                'total_distance': np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1)),
                'max_displacement': np.max(np.linalg.norm(positions, axis=1)),
                'trajectory_length': len(imu_traj),
                'start_position': positions[0],
                'end_position': positions[-1]
            }

        if self.results['fused_trajectory']:
            fused_traj = self.results['fused_trajectory']
            positions = np.array([pose['position'] for pose in fused_traj])

            analysis['fused'] = {
                'total_distance': np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1)),
                'max_displacement': np.max(np.linalg.norm(positions, axis=1)),
                'trajectory_length': len(fused_traj),
                'start_position': positions[0],
                'end_position': positions[-1]
            }

        return analysis

    def visualize_results(self, save_path: str = None):
        """
        Visualize SLAM results

        Args:
            save_path: Optional path to save the plot
        """
        fig = plt.figure(figsize=(20, 12))

        # 3D trajectory and map plot
        ax1 = fig.add_subplot(231, projection='3d')

        if self.results['visual_trajectory']:
            visual_pos = np.array([pose['position'] for pose in self.results['visual_trajectory']])
            ax1.plot(visual_pos[:, 0], visual_pos[:, 1], visual_pos[:, 2], 'b-', label='Visual Odometry', linewidth=2)

        if self.results['imu_trajectory']:
            # Sample IMU trajectory to match visual frequency
            imu_traj = self.results['imu_trajectory']
            imu_pos = np.array([pose['position'] for pose in imu_traj[::50]])  # Subsample
            ax1.plot(imu_pos[:, 0], imu_pos[:, 1], imu_pos[:, 2], 'r-', label='IMU Integration', linewidth=2)

        if self.results['fused_trajectory']:
            fused_pos = np.array([pose['position'] for pose in self.results['fused_trajectory']])
            ax1.plot(fused_pos[:, 0], fused_pos[:, 1], fused_pos[:, 2], 'g-', label='Fused SLAM', linewidth=2)

        # Plot 3D map points
        if self.results['map_points']:
            map_points = np.array(self.results['map_points'])
            ax1.scatter(map_points[:, 0], map_points[:, 1], map_points[:, 2],
                       c='red', s=1, alpha=0.6, label='Map Points')

        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.set_title('3D Trajectory + Map Points')
        ax1.legend()

        # Top-down view (X-Y plane)
        ax2 = fig.add_subplot(232)

        if self.results['visual_trajectory']:
            visual_pos = np.array([pose['position'] for pose in self.results['visual_trajectory']])
            ax2.plot(visual_pos[:, 0], visual_pos[:, 1], 'b-', label='Visual Odometry', linewidth=2)
            ax2.scatter(visual_pos[0, 0], visual_pos[0, 1], c='blue', s=100, marker='o', label='Start')
            ax2.scatter(visual_pos[-1, 0], visual_pos[-1, 1], c='blue', s=100, marker='s', label='End')

        if self.results['fused_trajectory']:
            fused_pos = np.array([pose['position'] for pose in self.results['fused_trajectory']])
            ax2.plot(fused_pos[:, 0], fused_pos[:, 1], 'g-', label='Fused SLAM', linewidth=2)

        # Plot map points in top-down view
        if self.results['map_points']:
            map_points = np.array(self.results['map_points'])
            ax2.scatter(map_points[:, 0], map_points[:, 1], c='red', s=1, alpha=0.6, label='Map Points')

        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        ax2.set_title('Top-Down View (X-Y Plane)')
        ax2.legend()
        ax2.grid(True)
        ax2.axis('equal')

        # 3D Map Points Only
        ax3 = fig.add_subplot(233, projection='3d')

        if self.results['map_points']:
            map_points = np.array(self.results['map_points'])
            ax3.scatter(map_points[:, 0], map_points[:, 1], map_points[:, 2],
                       c='red', s=2, alpha=0.8)
            ax3.set_xlabel('X (m)')
            ax3.set_ylabel('Y (m)')
            ax3.set_zlabel('Z (m)')
            ax3.set_title(f'3D Environmental Map ({len(map_points)} points)')
        else:
            ax3.text(0.5, 0.5, 0.5, 'No map points generated',
                    ha='center', va='center', transform=ax3.transAxes)
            ax3.set_title('3D Environmental Map')

        # Position components over time
        ax4 = fig.add_subplot(234)

        if self.results['visual_trajectory']:
            timestamps = [(pose['timestamp'] - self.results['visual_trajectory'][0]['timestamp']) / 1e9
                         for pose in self.results['visual_trajectory']]
            visual_pos = np.array([pose['position'] for pose in self.results['visual_trajectory']])

            ax4.plot(timestamps, visual_pos[:, 0], 'b-', label='X (Visual)', linewidth=2)
            ax4.plot(timestamps, visual_pos[:, 1], 'r-', label='Y (Visual)', linewidth=2)
            ax4.plot(timestamps, visual_pos[:, 2], 'g-', label='Z (Visual)', linewidth=2)

        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Position (m)')
        ax4.set_title('Position Components vs Time')
        ax4.legend()
        ax4.grid(True)

        # SLAM statistics (span bottom two subplots)
        ax5 = fig.add_subplot(2, 3, (5, 6))
        ax5.axis('off')

        analysis = self.analyze_results()
        stats_text = "SLAM Results Summary:\n\n"

        for method, data in analysis.items():
            stats_text += f"{method.upper()}:\n"
            stats_text += f"  Total distance: {data['total_distance']:.2f} m\n"
            stats_text += f"  Max displacement: {data['max_displacement']:.2f} m\n"
            stats_text += f"  Trajectory points: {data['trajectory_length']}\n"
            stats_text += f"  Final position: [{data['end_position'][0]:.2f}, {data['end_position'][1]:.2f}, {data['end_position'][2]:.2f}]\n\n"

        # Add map statistics if available
        if self.results['map_points']:
            map_points = np.array(self.results['map_points'])
            stats_text += "3D ENVIRONMENTAL MAP:\n"
            stats_text += f"  Total map points: {len(map_points)}\n"
            if len(map_points) > 0:
                bounds = {
                    'min': map_points.min(axis=0),
                    'max': map_points.max(axis=0),
                    'range': map_points.max(axis=0) - map_points.min(axis=0)
                }
                stats_text += f"  Map bounds (X): [{bounds['min'][0]:.2f}, {bounds['max'][0]:.2f}] m\n"
                stats_text += f"  Map bounds (Y): [{bounds['min'][1]:.2f}, {bounds['max'][1]:.2f}] m\n"
                stats_text += f"  Map bounds (Z): [{bounds['min'][2]:.2f}, {bounds['max'][2]:.2f}] m\n"
                stats_text += f"  Map volume: {np.prod(bounds['range']):.2f} m³\n"

        ax5.text(0.05, 0.95, stats_text, transform=ax5.transAxes, fontsize=9,
                verticalalignment='top', fontfamily='monospace')

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Plot saved to: {save_path}")

        plt.show()

    def get_results(self) -> Dict:
        """Get all SLAM results"""
        return self.results