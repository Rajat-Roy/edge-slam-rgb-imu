"""
Data Loader for Android SLAM Logger Dataset
Loads synchronized RGB images and IMU data for SLAM processing
"""
import json
import cv2
import numpy as np
from typing import List, Dict, Tuple, Optional
import os
from pathlib import Path

class SLAMDataLoader:
    def __init__(self, data_path: str):
        """
        Initialize data loader for Android SLAM dataset

        Args:
            data_path: Path to the slam_session directory
        """
        self.data_path = Path(data_path)
        self.images_path = self.data_path / "images"
        self.camera_frames = []
        self.imu_data = []
        self.session_metadata = {}

        self._load_metadata()
        self._load_camera_frames()
        self._load_imu_data()
        self._synchronize_data()

    def _load_metadata(self):
        """Load session metadata"""
        metadata_file = self.data_path / "session_metadata.json"
        with open(metadata_file, 'r') as f:
            self.session_metadata = json.load(f)

        print(f"Loaded session: {self.session_metadata['sessionId']}")
        print(f"Duration: {(self.session_metadata['endTime'] - self.session_metadata['startTime'])/1000:.1f}s")
        print(f"Frames: {self.session_metadata['totalFrames']}")
        print(f"IMU samples: {self.session_metadata['totalImuSamples']}")

    def _load_camera_frames(self):
        """Load camera frame metadata"""
        camera_file = self.data_path / "data" / "camera_frames.jsonl"
        with open(camera_file, 'r') as f:
            content = f.read().strip()

            # Handle pretty-printed JSON (multi-line) by splitting on }{ pattern
            json_objects = []
            current_obj = ""
            brace_count = 0

            for char in content:
                current_obj += char
                if char == '{':
                    brace_count += 1
                elif char == '}':
                    brace_count -= 1
                    if brace_count == 0:
                        # Complete JSON object
                        try:
                            frame_data = json.loads(current_obj.strip())
                            self.camera_frames.append(frame_data)
                        except json.JSONDecodeError:
                            pass
                        current_obj = ""

        print(f"Loaded {len(self.camera_frames)} camera frames")

    def _load_imu_data(self):
        """Load IMU sensor data"""
        imu_file = self.data_path / "data" / "imu_data.jsonl"
        with open(imu_file, 'r') as f:
            content = f.read().strip()

            # Handle pretty-printed JSON (multi-line)
            current_obj = ""
            brace_count = 0

            for char in content:
                current_obj += char
                if char == '{':
                    brace_count += 1
                elif char == '}':
                    brace_count -= 1
                    if brace_count == 0:
                        # Complete JSON object
                        try:
                            imu_sample = json.loads(current_obj.strip())
                            self.imu_data.append(imu_sample)
                        except json.JSONDecodeError:
                            pass
                        current_obj = ""

        print(f"Loaded {len(self.imu_data)} IMU samples")

    def _synchronize_data(self):
        """Sort data by timestamp and prepare for synchronized access"""
        # Sort camera frames by timestamp
        self.camera_frames.sort(key=lambda x: x['timestamp'])

        # Sort IMU data by timestamp and separate accel/gyro
        self.imu_data.sort(key=lambda x: x['timestamp'])

        # Separate accelerometer and gyroscope data
        self.accel_data = [sample for sample in self.imu_data if sample['sensorType'] == 1]
        self.gyro_data = [sample for sample in self.imu_data if sample['sensorType'] == 4]

        print(f"Accelerometer samples: {len(self.accel_data)}")
        print(f"Gyroscope samples: {len(self.gyro_data)}")

        # Get time range
        start_time = min(self.camera_frames[0]['timestamp'],
                        self.imu_data[0]['timestamp'])
        end_time = max(self.camera_frames[-1]['timestamp'],
                      self.imu_data[-1]['timestamp'])

        print(f"Time range: {(end_time - start_time) / 1e9:.2f} seconds")

    def get_frame(self, frame_idx: int) -> Tuple[np.ndarray, Dict]:
        """
        Get camera frame by index

        Args:
            frame_idx: Frame index

        Returns:
            Tuple of (image, frame_metadata)
        """
        if frame_idx >= len(self.camera_frames):
            return None, None

        frame_data = self.camera_frames[frame_idx]
        image_path = self.images_path / frame_data['fileName']

        if not image_path.exists():
            return None, frame_data

        image = cv2.imread(str(image_path))
        return image, frame_data

    def get_imu_between_frames(self, frame_idx1: int, frame_idx2: int) -> Tuple[List, List]:
        """
        Get IMU data between two frames

        Args:
            frame_idx1: Starting frame index
            frame_idx2: Ending frame index

        Returns:
            Tuple of (accelerometer_data, gyroscope_data)
        """
        if frame_idx1 >= len(self.camera_frames) or frame_idx2 >= len(self.camera_frames):
            return [], []

        t1 = self.camera_frames[frame_idx1]['timestamp']
        t2 = self.camera_frames[frame_idx2]['timestamp']

        accel_between = [sample for sample in self.accel_data
                        if t1 <= sample['timestamp'] <= t2]
        gyro_between = [sample for sample in self.gyro_data
                       if t1 <= sample['timestamp'] <= t2]

        return accel_between, gyro_between

    def get_camera_intrinsics(self) -> np.ndarray:
        """
        Get camera intrinsic matrix (estimated for mobile camera)

        Returns:
            3x3 camera intrinsic matrix
        """
        # Get image dimensions from first frame
        img, _ = self.get_frame(0)
        height, width = img.shape[:2]

        # Estimated camera parameters for mobile phone
        # Focal length approximately 0.8 * image_width for typical mobile cameras
        fx = fy = 0.8 * width
        cx = width / 2.0
        cy = height / 2.0

        K = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ])

        return K

    def get_frame_count(self) -> int:
        """Get total number of frames"""
        return len(self.camera_frames)

    def get_time_range(self) -> Tuple[int, int]:
        """Get timestamp range (start, end) in nanoseconds"""
        start_time = min(self.camera_frames[0]['timestamp'],
                        self.imu_data[0]['timestamp'])
        end_time = max(self.camera_frames[-1]['timestamp'],
                      self.imu_data[-1]['timestamp'])
        return start_time, end_time

    def interpolate_imu_at_timestamp(self, timestamp: int) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Interpolate IMU data at specific timestamp

        Args:
            timestamp: Target timestamp in nanoseconds

        Returns:
            Tuple of (accelerometer_vector, gyroscope_vector)
        """
        # Find closest accelerometer samples
        accel_before = None
        accel_after = None
        for sample in self.accel_data:
            if sample['timestamp'] <= timestamp:
                accel_before = sample
            elif sample['timestamp'] > timestamp and accel_after is None:
                accel_after = sample
                break

        # Find closest gyroscope samples
        gyro_before = None
        gyro_after = None
        for sample in self.gyro_data:
            if sample['timestamp'] <= timestamp:
                gyro_before = sample
            elif sample['timestamp'] > timestamp and gyro_after is None:
                gyro_after = sample
                break

        # Interpolate accelerometer
        accel_interp = None
        if accel_before and accel_after:
            t_range = accel_after['timestamp'] - accel_before['timestamp']
            if t_range > 0:
                alpha = (timestamp - accel_before['timestamp']) / t_range
                accel_interp = np.array(accel_before['values']) * (1 - alpha) + \
                              np.array(accel_after['values']) * alpha
        elif accel_before:
            accel_interp = np.array(accel_before['values'])

        # Interpolate gyroscope
        gyro_interp = None
        if gyro_before and gyro_after:
            t_range = gyro_after['timestamp'] - gyro_before['timestamp']
            if t_range > 0:
                alpha = (timestamp - gyro_before['timestamp']) / t_range
                gyro_interp = np.array(gyro_before['values']) * (1 - alpha) + \
                             np.array(gyro_after['values']) * alpha
        elif gyro_before:
            gyro_interp = np.array(gyro_before['values'])

        return accel_interp, gyro_interp

    def get_imu_data(self) -> Dict:
        """
        Get all IMU data organized by sensor type

        Returns:
            Dictionary with accelerometer and gyroscope data
        """
        return {
            'accelerometer': self.accel_data,
            'gyroscope': self.gyro_data
        }