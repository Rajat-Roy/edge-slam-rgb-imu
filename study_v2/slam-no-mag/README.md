# SLAM without Magnetometer

## Overview
SLAM using Camera + Gyroscope + Accelerometer + GPS (no magnetometer)

## Results
- **Mean Error: 333.1m**
- Improvement over full-sensor: **23%**
- GPS path: 2020.4m

## Why Remove Magnetometer?
The scooty's motor creates strong electromagnetic interference that corrupts magnetometer readings. Without it, we rely on:
- Gyroscope for heading (with bias correction)
- GPS for scale calibration

## Key Observations

### Gyroscope-Only Heading
- Heading drift: ~3° over 5 minutes
- Bias correction from first 1 second helps
- Still accumulates error over long trajectories

### Remaining Issues
1. **No heading correction** - Gyro drift is unbounded
2. **No position correction** - Dead reckoning accumulates error
3. **Scale only from GPS** - No continuous position updates

## Heading Drift Analysis
```
Total heading change: 192.6°
Expected (from GPS): ~180° (approximate loop)
Drift: ~12.6° over 308 seconds
Drift rate: ~0.04°/second
```

## Conclusion
Removing magnetometer improved accuracy by 23%, confirming it was a noise source. However, gyroscope drift still causes significant errors over 5 minutes.

## Next Step
Add GPS heading correction → see `slam-gps-corrected/`
