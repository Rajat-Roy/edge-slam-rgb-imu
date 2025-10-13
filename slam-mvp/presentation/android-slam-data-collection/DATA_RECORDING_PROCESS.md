# Data Recording Process

## Sensor Data Collection Overview

### IMU Data Recording
The DataLogger continuously captures data from two primary IMU sensors:

**Accelerometer (Linear Acceleration)**
- Sampling Rate: 399.5 Hz (near maximum mobile capability)
- Range: ±16g (suitable for human motion)
- Data Format: [x, y, z] acceleration values in m/s²
- Coordinate System: Device-relative coordinates

**Gyroscope (Angular Velocity)**
- Sampling Rate: 399.5 Hz (synchronized with accelerometer)
- Range: ±2000°/s (adequate for normal device rotation)
- Data Format: [x, y, z] angular velocity in rad/s
- Coordinate System: Device-relative coordinates

### Camera Data Recording
**RGB Frame Capture**
- Frame Rate: 29.9 fps (consistent timing)
- Resolution: High-quality JPEG compression
- Format: Individual image files with timestamp metadata
- Synchronization: Nanosecond precision with IMU data

## Data Storage Architecture

### JSONL Streaming Format
Each sensor reading is immediately written as a single JSON line:

**IMU Data Structure:**
```json
{"timestamp": 1703123456789012345, "type": "accelerometer", "values": [0.12, 9.81, 0.34]}
{"timestamp": 1703123456791512345, "type": "gyroscope", "values": [0.01, -0.02, 0.15]}
```

**Camera Frame Metadata:**
```json
{"timestamp": 1703123456823456789, "frameNumber": 1, "fileName": "frame_000001.jpg", "width": 1920, "height": 1080}
```

**Core DataLogger Implementation:**
```kotlin
suspend fun logIMUData(imuData: IMUData) = withContext(Dispatchers.IO) {
    imuWriter?.let { writer ->
        val json = gson.toJson(imuData)
        writer.appendLine(json)
        writer.flush()  // Immediate write for reliability
        imuCount++
    }
}

suspend fun logCameraFrame(timestamp: Long, frameNumber: Int, fileName: String, width: Int, height: Int) = withContext(Dispatchers.IO) {
    val cameraFrame = CameraFrame(timestamp, frameNumber, fileName, width, height)
    cameraWriter?.let { writer ->
        val json = gson.toJson(cameraFrame)
        writer.appendLine(json)
        writer.flush()  // Ensure data persistence
        frameCount++
    }
}
```

### File Organization
```
session_data/
├── session_metadata.json    # Session summary
├── data/
│   ├── imu_data.jsonl      # Streaming IMU readings
│   └── camera_frames.jsonl # Frame timing metadata
└── images/
    └── frame_XXXXXX.jpg    # Individual camera frames
```

## Recording Process Flow

### Session Initialization
1. **Sensor Registration**: Register accelerometer and gyroscope listeners
2. **Camera Setup**: Initialize CameraX with preview and capture
3. **File Creation**: Create session directory and JSONL files
4. **Timestamp Sync**: Establish common time base using `System.nanoTime()`

**Session Setup Code:**
```kotlin
suspend fun startSession(): String = withContext(Dispatchers.IO) {
    val timestamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.getDefault()).format(Date())
    sessionId = "slam_session_$timestamp"

    // Create session directory structure
    sessionDir = File(appDir, sessionId)
    sessionDir?.mkdirs()
    File(sessionDir, "images").mkdirs()
    File(sessionDir, "data").mkdirs()

    // Initialize JSONL data files
    imuWriter = FileWriter(File(sessionDir, "data/imu_data.jsonl"))
    cameraWriter = FileWriter(File(sessionDir, "data/camera_frames.jsonl"))

    startTime = System.currentTimeMillis()
    frameCount = 0
    imuCount = 0

    sessionId
}
```

### Active Recording
1. **IMU Data Flow**:
   - Sensor events trigger at 400Hz target rate
   - Each reading immediately serialized to JSON
   - Written to `imu_data.jsonl` with flush for reliability

2. **Camera Data Flow**:
   - Frame capture at 30fps target rate
   - Image saved as JPEG file
   - Metadata written to `camera_frames.jsonl`
   - Timestamp recorded at moment of capture

3. **Thread Management**:
   - Main UI thread handles user interaction
   - Sensor thread processes IMU data
   - Camera thread manages frame capture
   - Background thread handles file I/O

### Data Integrity Measures
- **Immediate Writing**: Each measurement written immediately (no buffering)
- **File Flushing**: Force write to storage after each entry
- **Error Handling**: Continue recording even if individual samples fail
- **Atomic Operations**: Each JSONL line is complete and independent

**Session Completion with Metadata:**
```kotlin
suspend fun endSession() = withContext(Dispatchers.IO) {
    val endTime = System.currentTimeMillis()
    val duration = (endTime - startTime) / 1000.0

    // Close data writers
    imuWriter?.close()
    cameraWriter?.close()

    // Generate session summary
    val metadata = SessionMetadata(
        sessionId = sessionId,
        startTime = startTime,
        endTime = endTime,
        totalFrames = frameCount,
        totalImuSamples = imuCount,
        frameRate = frameCount / duration,
        imuSampleRate = imuCount / duration
    )

    // Save session metadata
    File(sessionDir, "session_metadata.json").writeText(gson.toJson(metadata))
}
```

## Synchronization Strategy

### Timestamp Precision
- **Base Timer**: `System.nanoTime()` provides nanosecond resolution
- **Consistent Reference**: Same timer used for all sensors
- **Capture Timing**: Timestamp recorded at sensor event, not file write

### Multi-Modal Alignment
- Camera frames and IMU samples share identical time base
- Post-processing can interpolate between high-frequency IMU and lower-frequency camera data
- Temporal alignment suitable for sensor fusion algorithms

## Performance Characteristics

### Achieved Metrics
- **IMU Sampling**: 399.5 Hz sustained rate (99.9% of target)
- **Camera Capture**: 29.9 fps sustained rate (99.7% of target)
- **Data Loss**: 0% across entire 52.5-second session
- **File Size**: ~1.6MB per second of recording

### Resource Management
- **Memory Usage**: Minimal buffering, immediate disk writes
- **CPU Load**: Distributed across multiple threads
- **Storage I/O**: Optimized JSONL format for streaming writes
- **Battery Impact**: Continuous sensor usage with camera preview

## Quality Assurance

### Data Validation
- **Timestamp Monotonicity**: Verify increasing timestamp sequence
- **Sampling Rate**: Monitor actual vs target frequency
- **File Integrity**: Validate JSONL format compliance
- **Completeness**: Ensure all expected files are created

### Error Recovery
- **Sensor Failures**: Continue with available sensors
- **Storage Errors**: Attempt retry with exponential backoff
- **Memory Pressure**: Aggressive garbage collection
- **App Lifecycle**: Proper cleanup on interruption

---

**This recording process ensures high-quality, synchronized multi-modal sensor data suitable for computer vision and robotics research applications.**