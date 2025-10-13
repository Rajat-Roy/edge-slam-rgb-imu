# Android SLAM Logger

Camera + IMU data collection app for visual-inertial SLAM research.

## Features

- **Synchronized Data Collection**: Captures RGB camera frames at 30fps, IMU data at 200Hz, and GPS coordinates at 1Hz
- **High-precision Timestamps**: Uses `System.nanoTime()` for synchronized timestamping across all sensors
- **Structured Storage**: Saves data in JSON Lines format with organized directory structure
- **Real-time Monitoring**: Live frame, IMU, and GPS sample counters with GPS status indicator
- **Session Management**: Each recording creates a unique session with comprehensive metadata
- **GPS Integration**: Records location, altitude, accuracy, speed, and bearing data for outdoor SLAM

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
│   ├── imu_data.jsonl
│   └── gps_data.jsonl
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

### GPS Data Format
```json
{
  "timestamp": 123456789000000,
  "latitude": 28.2345,
  "longitude": 73.1234,
  "altitude": 250.5,
  "accuracy": 5.0,
  "speed": 2.5,
  "bearing": 45.0,
  "provider": "gps"
}
```

GPS fields:
- `latitude/longitude`: WGS84 coordinates in degrees
- `altitude`: Height above sea level in meters (null if unavailable)
- `accuracy`: Horizontal accuracy in meters (null if unavailable)
- `speed`: Speed over ground in m/s (null if stationary/unavailable)
- `bearing`: Direction of travel in degrees (null if stationary/unavailable)
- `provider`: "gps" or "network"

## Build and Deploy

### Prerequisites
- Android Studio or Gradle
- Android device with API 24+ (Android 7.0+)
- USB debugging enabled
- GPS-enabled device (for location data)
- Outdoor location for GPS testing

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
2. **Grant permissions** for camera, storage, and location when prompted
3. **Ensure GPS is available** (check GPS status indicator - green = available, red = unavailable)
4. **Position your device** for data collection
5. **Tap "Start Recording"** to begin synchronized capture
6. **Move the device** through your environment (outdoor for GPS data)
7. **Tap "Stop Recording"** when done
8. **Data is saved** to `/Android/data/com.iitj.slamlogger/files/Documents/SLAMLogger/`

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
- **GPS Rate**: 1Hz location updates (standard GPS limitation)
- **Storage**: JPEG compression at 90% quality for images
- **Synchronization**: All timestamps use the same `System.nanoTime()` clock
- **Memory**: Efficient streaming writes, minimal memory footprint
- **GPS Fallback**: Network location used when GPS unavailable

## Troubleshooting

**Permission Issues**: Ensure camera, storage, and location permissions are granted in device settings.

**GPS Issues**:
- Ensure location services are enabled in device settings
- For best GPS reception, record outdoors with clear sky view
- App will continue recording without GPS if unavailable

**Storage Issues**: Check available storage space. Each minute of recording uses ~50-100MB plus GPS data.

**Wireless Connection**: If ADB connection drops, reconnect via USB and re-run `adb tcpip 5555`.

**Build Issues**: Ensure you have Android SDK 34 and Kotlin 1.9+ installed.

**GPS Data Quality**:
- GPS accuracy varies with environment (indoors vs outdoors)
- Network location has lower accuracy than GPS
- Check GPS status indicator before starting important recordings