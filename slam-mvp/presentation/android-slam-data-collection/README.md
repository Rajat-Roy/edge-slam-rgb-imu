# Android Sensor Data Collection App

## Project Overview
**Android Sensor Data Logger** - Mobile application for synchronized RGB camera and IMU data collection

**Project**: MTech Research Project (IIT Jodhpur, 2025–26)

**Student**: Rajat Roy

**Supervisor**: Dr. Hardik Jain

## What We Built

### Core Features
- **Synchronized RGB + IMU Recording**: Camera at 29.9 fps, IMU at 399.5 Hz
- **Wireless Development**: Complete development without USB cables
- **JSONL Data Format**: Streaming format for high-frequency sensor data
- **Real-time Preview**: Live camera feed during recording

### Results Achieved
- **52.5-second recording session** with zero data loss
- **1,571 camera frames** captured
- **20,954 IMU samples** collected
- **82MB total dataset** successfully generated

## Technical Implementation

### Android App Architecture
```
MainActivity.kt           // UI and recording control
├── SensorDataCollector  // IMU integration (400Hz)
├── DataLogger          // JSONL file writing
└── CameraX            // RGB frame capture
```

### Key Innovations
- **JSONL Streaming**: Real-time sensor data format
- **Wireless Development**: ADB wireless debugging workflow
- **Multi-threaded Design**: Separate camera, IMU, and file I/O threads
- **Nanosecond Synchronization**: Precise timestamp alignment

## Demo Results

### Session Statistics
| Metric | Value |
|--------|-------|
| Duration | 52.5 seconds |
| RGB Frames | 1,571 frames |
| IMU Samples | 20,954 samples |
| File Size | 82MB |
| Data Loss | 0% |

### Data Quality
- **Camera**: 29.9 fps consistent frame rate
- **IMU**: 399.5 Hz sampling (near maximum mobile rate)
- **Synchronization**: Nanosecond precision timestamps
- **Storage**: Zero corruption, perfect integrity

## Current Status

**Phase 1:**
- Android data collection app fully functional
- Successful wireless development workflow
- High-quality multi-modal dataset collected
- Ready for next phase development

## Package Contents

```
android-slam-data-collection/
├── app-source/        # Complete Android Studio project
├── demo-data/         # 52.5s recording session
│   ├── images/        # 1,571 camera frames
│   └── data/          # IMU + camera metadata
└── screenshots/       # App UI (if needed)
```

---

**Android data collection system successfully implemented and validated**