# Android SLAM Logger

Camera + IMU data collection app for visual-inertial SLAM research.

## Features

- **Synchronized Data Collection**: Captures RGB camera frames at 30fps and IMU data at 200Hz
- **High-precision Timestamps**: Uses `System.nanoTime()` for synchronized timestamping
- **Structured Storage**: Saves data in JSON Lines format with organized directory structure
- **Real-time Monitoring**: Live frame and IMU sample counters
- **Session Management**: Each recording creates a unique session with metadata

## Data Format

Each recording session creates a directory structure:
```
slam_session_YYYYMMDD_HHMMSS/
├── images/
│   ├── frame_000001.jpg
│   ├── frame_000002.jpg
│   └── ...
├── data/
│   ├── camera_frames.jsonl
│   └── imu_data.jsonl
└── session_metadata.json
```

### Camera Data Format
```json
{
  "timestamp": 123456789000000,
  "frameNumber": 1,
  "fileName": "frame_000001.jpg",
  "width": 1920,
  "height": 1080
}
```

### IMU Data Format
```json
{
  "timestamp": 123456789000000,
  "sensorType": 1,
  "values": [0.1, -0.2, 9.8],
  "accuracy": 3
}
```

Where `sensorType` is:
- `1` = Accelerometer (m/s²)
- `4` = Gyroscope (rad/s)

## Build and Deploy

### Prerequisites
- Android Studio or Gradle
- Android device with API 24+ (Android 7.0+)
- USB debugging enabled

### Wireless Debugging Setup

1. **Enable Developer Options** on your Android device:
   - Go to Settings → About Phone
   - Tap "Build Number" 7 times

2. **Enable Wireless Debugging**:
   - Settings → Developer Options
   - Turn on "Wireless debugging"
   - Note the IP address and port (e.g., 192.168.1.100:5555)

3. **Connect via ADB**:
   ```bash
   # First connect via USB
   adb devices

   # Enable TCP/IP mode
   adb tcpip 5555

   # Disconnect USB and connect wirelessly
   adb connect <DEVICE_IP>:5555

   # Verify connection
   adb devices
   ```

### Build Commands

```bash
# Build debug APK
./gradlew assembleDebug

# Install on connected device
./gradlew installDebug

# Build and install in one step
./gradlew installDebug

# View logs
adb logcat -s SLAM
```

## Usage

1. **Launch the app** on your Android device
2. **Grant permissions** for camera and storage when prompted
3. **Position your device** for data collection
4. **Tap "Start Recording"** to begin synchronized capture
5. **Move the device** through your environment
6. **Tap "Stop Recording"** when done
7. **Data is saved** to `/Android/data/com.iitj.slamlogger/files/Documents/SLAMLogger/`

## Data Transfer

Retrieve data via ADB:
```bash
# List recording sessions
adb shell ls /sdcard/Android/data/com.iitj.slamlogger/files/Documents/SLAMLogger/

# Pull specific session
adb pull /sdcard/Android/data/com.iitj.slamlogger/files/Documents/SLAMLogger/slam_session_YYYYMMDD_HHMMSS ./

# Pull all sessions
adb pull /sdcard/Android/data/com.iitj.slamlogger/files/Documents/SLAMLogger/ ./data/
```

## Technical Notes

- **Frame Rate**: Target 30fps camera capture (actual rate depends on device performance)
- **IMU Rate**: 200Hz accelerometer and gyroscope sampling
- **Storage**: JPEG compression at 90% quality for images
- **Synchronization**: All timestamps use the same `System.nanoTime()` clock
- **Memory**: Efficient streaming writes, minimal memory footprint

## Troubleshooting

**Permission Issues**: Ensure camera and storage permissions are granted in device settings.

**Storage Issues**: Check available storage space. Each minute of recording uses ~50-100MB.

**Wireless Connection**: If ADB connection drops, reconnect via USB and re-run `adb tcpip 5555`.

**Build Issues**: Ensure you have Android SDK 34 and Kotlin 1.9+ installed.