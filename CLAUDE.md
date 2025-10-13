# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an MTech project at IIT Jodhpur (2025-26) focused on developing **Efficient Visual-Inertial SLAM on Edge Devices** using RGB + IMU fusion. The project is currently in early stages with completed Android data collection infrastructure and planned SLAM implementation.

**Current Status**: Phase 1 (Data Collection) ✅ COMPLETED, Phase 2 (SLAM Implementation) in planning

## Architecture

### Android Data Collection System
The main working component is an Android application (`android-slam-logger/`) that captures synchronized RGB camera frames, IMU sensor data, and GPS coordinates:

- **MainActivity.kt**: Core camera capture and UI management using CameraX
- **SensorDataCollector.kt**: High-frequency (200Hz) IMU data collection from accelerometer/gyroscope
- **GPSDataCollector.kt**: GPS location tracking at 1Hz with network fallback
- **DataLogger.kt**: File storage and session management with JSON Lines format
- **Data Pipeline**: CameraX ImageAnalysis → YUV to JPEG conversion → synchronized timestamping across all sensors

### Data Format Structure
```
slam_session_YYYYMMDD_HHMMSS/
├── images/frame_000001.jpg → frame_000608.jpg     # RGB frames @ 30fps
├── data/
│   ├── camera_frames.jsonl    # Frame metadata (timestamps, dimensions)
│   ├── imu_data.jsonl         # IMU measurements @ 200Hz
│   └── gps_data.jsonl         # GPS coordinates @ 1Hz
└── session_metadata.json      # Performance summary and statistics
```

**Key Technical Details**:
- All timestamps use `System.nanoTime()` for perfect synchronization across all sensors
- IMU data includes accelerometer (m/s²) and gyroscope (rad/s) measurements
- GPS data includes latitude/longitude, altitude, accuracy, speed, and bearing
- JPEG compression at 90% quality with 640×480 resolution
- Streaming JSON Lines format prevents memory issues during long sessions
- Network location fallback when GPS unavailable

## Development Commands

### Android Development
```bash
# Build and install Android app
cd android-slam-logger
./gradlew assembleDebug
./gradlew installDebug

# Wireless ADB setup (recommended for natural movement)
adb tcpip 5555
adb connect <DEVICE_IP>:5555

# Monitor data collection in real-time
adb logcat -s SLAM

# Retrieve collected data
adb pull /sdcard/Android/data/com.iitj.slamlogger/files/Documents/SLAMLogger/ ./data/
```

### Testing and Validation
```bash
# Verify data quality after collection
ls data/slam_session_*/images/ | wc -l                    # Count frames
wc -l data/slam_session_*/data/imu_data.jsonl            # Count IMU samples
wc -l data/slam_session_*/data/gps_data.jsonl            # Count GPS samples
cat data/slam_session_*/session_metadata.json             # Check performance metrics
```

## Project Timeline and Current Phase

**Completed (Phase 1)**:
- ✅ Android MVP with 25.8 fps camera capture
- ✅ 398.8 Hz IMU sampling (exceeds 200Hz target)
- ✅ Wireless debugging workflow
- ✅ Validated data synchronization quality

**Next Steps (Phase 2 - Planning)**:
- Literature review of mobile SLAM frameworks (ORB-SLAM3, VINS-Mono)
- Implementation of visual-inertial SLAM pipeline
- CUDA/TensorRT optimization for edge deployment
- Benchmarking on EuRoC/TUM datasets

## Key Technical Specifications

### Performance Targets (Achieved)
- **Camera Frame Rate**: 30 fps (achieved 25.8 fps)
- **IMU Sampling**: 200 Hz (achieved 398.8 Hz)
- **GPS Sampling**: 1 Hz (standard GPS limitation)
- **Data Storage**: ~1-2 MB per second plus GPS data
- **Synchronization**: Nanosecond precision timestamps across all sensors

### Android Dependencies
- **Camera**: CameraX 1.3.0 for robust frame capture
- **JSON**: Gson 2.10.1 for structured data logging
- **Coroutines**: Kotlinx 1.7.3 for asynchronous operations
- **Location**: Android LocationManager for GPS tracking
- **Target SDK**: API 24+ (Android 7.0+)

### Wireless Development Workflow
The project uses wireless ADB debugging to enable natural device movement during data collection. This workflow has been validated and documented for stable operation.

## Data Quality and SLAM Readiness

The collected data is structured for compatibility with existing SLAM frameworks:
- **ORB-SLAM3**: Requires format conversion from JSON Lines
- **VINS-Mono**: Compatible with JSON preprocessing, GPS integration possible
- **GPS-Enhanced SLAM**: Direct access to synchronized GPS + visual + inertial data
- **Standard Datasets**: Comparable to EuRoC/TUM format structure with GPS addition
- **Custom Pipelines**: Direct access to raw sensor streams with perfect synchronization

## Development Environment Notes

- **Primary Development**: macOS with Android Studio support
- **Target Hardware**: Android mobile, Jetson Nano (2GB), Mac M1
- **Language Stack**: Kotlin (Android), planned C++/Python (SLAM)
- **Build System**: Gradle for Android, CMake planned for SLAM components

## Future Architecture (Planned)

Phase 2 will add:
- SLAM algorithm integration (C++ core with Python bindings)
- CUDA/TensorRT optimization modules
- Dataset conversion utilities (JSON → standard SLAM formats)
- Visualization and trajectory analysis tools
- Performance benchmarking framework

The current Android infrastructure provides high-quality synchronized sensor data suitable for visual-inertial SLAM research and development.