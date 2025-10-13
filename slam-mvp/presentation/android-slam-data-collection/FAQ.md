# Frequently Asked Questions

## Technical Questions

**Q: Why use JSONL format instead of regular JSON?**
A: JSONL (JSON Lines) allows streaming data processing and fault tolerance. If one line gets corrupted, we don't lose the entire dataset. Critical for high-frequency IMU data at 400Hz.

**Q: How accurate is the timestamp synchronization?**
A: Nanosecond precision using `System.nanoTime()`. Both camera frames and IMU samples share the same time base, ensuring proper alignment for sensor fusion.

**Q: Why wireless development instead of USB?**
A: Wireless debugging allows natural phone movement during data collection. USB cables would constrain motion and affect the quality of recorded sensor data.

**Q: What's the maximum recording duration?**
A: Currently limited by storage space. Our 52.5-second session generated 82MB. Could theoretically record much longer sessions with sufficient storage.

**Q: How do you handle phone orientation during recording?**
A: The app records raw IMU data without orientation correction. Gravity vector analysis shows the phone orientation, which can be corrected in post-processing.

## Development Questions

**Q: Can this run on any Android device?**
A: Requires Android API 24+ (Android 7.0). Works best on devices with high-quality IMU sensors and sufficient processing power.

**Q: How reliable is the data collection?**
A: Achieved 0% data loss across our test session with 20,954 IMU samples and 1,571 camera frames. Multi-threaded architecture prevents blocking.

**Q: Can the app collect data while the phone is locked?**
A: Currently requires the app to be active. Could be enhanced with background services for longer autonomous collection.

## Research Questions

**Q: What makes this different from existing data collection apps?**
A: Synchronized multi-modal recording (RGB+IMU), wireless development workflow, and JSONL streaming format optimized for research use.

**Q: How does this compare to commercial solutions?**
A: Most commercial apps don't provide synchronized timestamps or high-frequency IMU data. This system is specifically designed for computer vision research.

**Q: What's the next development phase?**
A: Integration with computer vision algorithms, data preprocessing pipeline, and potential real-time processing capabilities.

## Practical Questions

**Q: How much storage space is needed?**
A: Approximately 1.6MB per second of recording (82MB for 52.5 seconds). Includes full-resolution images and high-frequency IMU data.

**Q: Can multiple phones be synchronized?**
A: Current version supports single device. Multi-device synchronization would require network time protocols or external timing references.

**Q: Is the source code available?**
A: Complete Android Studio project is included in the package for reproducibility and further development.