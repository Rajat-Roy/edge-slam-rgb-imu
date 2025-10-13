# Build Instructions for Android SLAM Logger

Since Android Studio and Gradle are not installed on this system, here are the steps to build and deploy the app:

## Setup Development Environment

### Option 1: Android Studio (Recommended)
1. **Download Android Studio**: https://developer.android.com/studio
2. **Install Android SDK**: API level 34 minimum
3. **Open Project**: Import the `android-slam-logger` folder
4. **Sync Gradle**: Android Studio will automatically download dependencies

### Option 2: Command Line with Gradle
1. **Install Android SDK Command Line Tools**
2. **Install Gradle**: `brew install gradle` (macOS)
3. **Set ANDROID_HOME**: Point to SDK installation directory

## Wireless ADB Setup

### Step 1: Enable Developer Options
1. Go to **Settings → About Phone**
2. Tap **Build Number** 7 times to enable Developer Options
3. Go to **Settings → Developer Options**
4. Enable **USB Debugging**
5. Enable **Wireless Debugging** (Android 11+)

### Step 2: Connect Device
```bash
# First, connect via USB cable
adb devices

# Switch to TCP/IP mode (port 5555)
adb tcpip 5555

# Find device IP address (Settings → About Phone → Status)
# Or use: adb shell ip addr show wlan0

# Disconnect USB cable and connect wirelessly
adb connect <DEVICE_IP>:5555

# Example:
adb connect 192.168.1.100:5555

# Verify wireless connection
adb devices
# Should show: 192.168.1.100:5555    device
```

## Build and Deploy Commands

### Using Android Studio
1. **Open Project**: File → Open → Select `android-slam-logger` folder
2. **Sync Project**: Let Gradle sync and download dependencies
3. **Connect Device**: Ensure wireless ADB connection is active
4. **Run App**: Click the green "Run" button or Shift+F10

### Using Command Line
```bash
cd android-slam-logger

# Build debug APK
./gradlew assembleDebug

# Install on connected device
./gradlew installDebug

# Or combine both steps
./gradlew installDebug
```

## Testing the App

### Launch and Test
1. **Open SLAM Logger** on your device
2. **Grant Permissions**: Allow camera and storage access
3. **Start Recording**: Tap "Start Recording" button
4. **Move Device**: Walk around to collect data
5. **Stop Recording**: Tap "Stop Recording"

### Monitor Output
```bash
# View app logs
adb logcat -s SLAM

# View all device logs
adb logcat

# Clear logs first
adb logcat -c && adb logcat -s SLAM
```

### Retrieve Data
```bash
# List recording sessions
adb shell ls /sdcard/Android/data/com.iitj.slamlogger/files/Documents/SLAMLogger/

# Pull specific session to your computer
adb pull /sdcard/Android/data/com.iitj.slamlogger/files/Documents/SLAMLogger/slam_session_20241001_143022 ./collected_data/

# Pull all sessions
adb pull /sdcard/Android/data/com.iitj.slamlogger/files/Documents/SLAMLogger/ ./slam_datasets/
```

## Expected Data Structure

After recording, you should see:
```
collected_data/
└── slam_session_20241001_143022/
    ├── images/
    │   ├── frame_000001.jpg
    │   ├── frame_000002.jpg
    │   └── ... (30 images per second)
    ├── data/
    │   ├── camera_frames.jsonl    # Frame metadata
    │   └── imu_data.jsonl         # IMU measurements
    └── session_metadata.json      # Session summary
```

## Verification Steps

### Check Data Quality
```bash
# Count images captured
ls collected_data/slam_session_*/images/ | wc -l

# Check IMU data samples
wc -l collected_data/slam_session_*/data/imu_data.jsonl

# View session metadata
cat collected_data/slam_session_*/session_metadata.json
```

### Expected Rates
- **Camera**: ~30 frames per second
- **IMU**: ~200 samples per second (100 accelerometer + 100 gyroscope)
- **Storage**: ~1-2 MB per second of recording

## Troubleshooting

### Permission Denied
```bash
# Grant storage permissions manually
adb shell pm grant com.iitj.slamlogger android.permission.WRITE_EXTERNAL_STORAGE
adb shell pm grant com.iitj.slamlogger android.permission.READ_EXTERNAL_STORAGE
```

### ADB Connection Issues
```bash
# Reset ADB server
adb kill-server
adb start-server

# Re-enable TCP/IP mode
adb tcpip 5555
adb connect <DEVICE_IP>:5555
```

### Build Issues
- Ensure Java 8+ is installed
- Check Android SDK is properly configured
- Verify internet connection for Gradle dependencies

## Next Steps

Once you have collected data:
1. **Verify synchronization** between camera and IMU timestamps
2. **Test with ORB-SLAM3** or other SLAM frameworks
3. **Analyze data quality** and collection rates
4. **Document indoor/outdoor performance** differences

The collected data will be used for the next phase: integrating with SLAM algorithms and benchmarking performance.