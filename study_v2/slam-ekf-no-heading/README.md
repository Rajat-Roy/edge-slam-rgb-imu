# SLAM with EKF (No GPS Heading Correction)

## Overview
EKF with GPS position updates only - no heading correction from velocity direction.

## Results
- **Mean Error: 16.4m**
- vs EKF with heading: **8% better** (unexpected!)
- GPS path: 2020.4m

## What Was Removed
The heading correction from velocity direction:
```python
# REMOVED:
if speed > 2.0 m/s:
    heading_from_vel = atan2(vy, vx)
    heading += 0.1 * angle_diff(heading_from_vel, heading)
```

Now heading comes purely from gyroscope integration.

## Key Observations

### Surprising Result
Expected worse performance without heading correction, but got **better**!

### Why This Happened
1. **GPS velocity noise** - Position noise creates noisy velocity estimates
2. **Heading correction adds noise** - At 1Hz GPS rate, velocity direction is jittery
3. **Good gyroscope** - Phone gyro is stable over 5 minutes
4. **Position updates sufficient** - GPS position corrects trajectory shape

### Gyroscope Stability
On this dataset:
- Gyro heading range: 192.6°
- Drift over 5 minutes: ~3°
- This is acceptable for 2km trajectory

### When Heading Correction Would Help
- Longer trajectories (>10 minutes)
- Higher gyro drift
- Straighter paths (less GPS course changes)

## Conclusion
For this 5-minute scooty ride, gyroscope-only heading performs better than GPS-corrected heading. The GPS position updates are sufficient to correct trajectory shape without explicit heading correction.

**Key insight**: GPS velocity-based heading can add noise if GPS update rate is low or position accuracy is poor.

## Comparison
| Method | Mean Error |
|--------|------------|
| EKF with heading | 17.8m |
| EKF without heading | **16.4m** |

## Next Step
Test without GPS entirely → see `slam-no-gps/`
