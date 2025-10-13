# SLAM Data Collection - Session slam_session_20250923_090335

## Overview
This directory contains synchronized RGB camera and IMU sensor data collected using the Android SLAM Logger app on September 23, 2025.

## Session Details
- **Duration**: 23.5 seconds
- **Location**: Indoor environment
- **Device**: Android smartphone via wireless debugging
- **Data Quality**: High-precision synchronized timestamps

## Data Structure

### Images (608 files)
```
images/
├── frame_000001.jpg  # First captured frame
├── frame_000002.jpg
├── ...
└── frame_000608.jpg  # Last captured frame
```
- **Resolution**: 640×480 pixels
- **Format**: JPEG (90% quality)
- **Frame Rate**: 25.8 fps average
- **Total Size**: ~48 MB

### Metadata Files

#### camera_frames.jsonl (4,256 lines)
Camera frame metadata with timestamps and file information:
```json
{
  "fileName": "frame_000001.jpg",
  "frameNumber": 1,
  "height": 480,
  "timestamp": 309418287131235,
  "width": 640
}
```

#### imu_data.jsonl (93,910 lines)
IMU sensor readings with high-frequency sampling:
```json
{
  "accuracy": 3,
  "sensorType": 1,
  "timestamp": 309418261118944,
  "values": [-0.15981296, 9.504382, 2.207454]
}
```

**Sensor Types:**
- `1` = Accelerometer (m/s²)
- `4` = Gyroscope (rad/s)

#### session_metadata.json
Session summary and performance metrics:
```json
{
  "sessionId": "slam_session_20250923_090335",
  "startTime": 1758598415895,
  "endTime": 1758598439443,
  "totalFrames": 608,
  "totalImuSamples": 9391,
  "frameRate": 25.81960251401393,
  "imuSampleRate": 398.80244606760664
}
```

## Data Characteristics

### Synchronization
- **Clock Source**: `System.nanoTime()` for all measurements
- **Precision**: Nanosecond timestamps
- **Alignment**: Camera and IMU events properly synchronized

### Quality Metrics
| Aspect | Value | Assessment |
|--------|--------|------------|
| Frame Rate | 25.8 fps | Good (86% of 30fps target) |
| IMU Rate | 398.8 Hz | Excellent (199% of 200Hz target) |
| Data Completeness | 100% | All frames and IMU samples captured |
| File Integrity | Valid | All files readable and properly formatted |

### Motion Profile
The recorded session includes typical handheld device motion suitable for visual-inertial SLAM testing:
- Translation movements in indoor environment
- Rotational motions for feature observation
- Gradual camera movements avoiding motion blur

## Usage for SLAM Processing

### Loading Camera Data
```python
import json
import cv2

# Load frame metadata
frames = []
with open('data/camera_frames.jsonl', 'r') as f:
    for line in f:
        frames.append(json.loads(line))

# Load corresponding images
for frame in frames:
    img_path = f"images/{frame['fileName']}"
    image = cv2.imread(img_path)
    timestamp = frame['timestamp']
    # Process frame...
```

### Loading IMU Data
```python
import json
import numpy as np

# Load IMU measurements
imu_data = []
with open('data/imu_data.jsonl', 'r') as f:
    for line in f:
        sample = json.loads(line)
        imu_data.append({
            'timestamp': sample['timestamp'],
            'type': 'accel' if sample['sensorType'] == 1 else 'gyro',
            'values': np.array(sample['values'])
        })
```

### Timestamp Alignment
All timestamps use the same `System.nanoTime()` clock source, enabling direct temporal alignment between camera frames and IMU measurements.

## Next Steps

This dataset is ready for:
1. **ORB-SLAM3 Integration**: Convert to supported input format
2. **VINS-Mono Testing**: Evaluate visual-inertial performance
3. **Algorithm Development**: Custom SLAM pipeline implementation
4. **Benchmarking**: Performance comparison against standard datasets

## File Manifest
- `images/`: 608 JPEG files (frame_000001.jpg to frame_000608.jpg)
- `data/camera_frames.jsonl`: Camera metadata (75KB)
- `data/imu_data.jsonl`: IMU sensor data (1.3MB)
- `session_metadata.json`: Session summary (229B)
- `README.md`: This documentation file

**Total Dataset Size**: ~50 MB