# Full Sensor SLAM

## Overview
SLAM using all available sensors: Camera + Gyroscope + Accelerometer + Magnetometer + GPS

## Results
- **Mean Error: 432.4m**
- Max Error: 866.9m
- GPS path: 2020.4m

## Sensors Used
- Camera: 8959 frames (optical flow)
- Gyroscope: 61550 samples (heading integration)
- Accelerometer: 61556 samples (motion detection)
- Magnetometer: 30773 samples (heading reference)
- GPS: 321 samples (scale calibration)

## Key Observations

### Magnetometer Interference
The magnetometer was severely corrupted by the scooty's motor and metal frame:
- Expected: Stable magnetic north reference
- Actual: Large variations correlated with engine RPM and position

### Complementary Filter Issue
Used alpha=0.98 for gyro/magnetometer fusion:
```
heading_fused = 0.98 * gyro_heading + 0.02 * mag_heading
```
Even with 2% weight, corrupted magnetometer caused significant heading errors.

### Motion Detection
Accelerometer used for stationary detection worked well:
- Threshold: 0.5 m/s² variance
- Helped identify stops at traffic lights

## Conclusion
**Magnetometer is unusable on motorized vehicles** due to electromagnetic interference. This baseline shows worst-case performance when trusting corrupted sensors.

## Next Step
Remove magnetometer → see `slam-no-mag/`
