# SLAM with GPS Heading & Position Correction

## Overview
SLAM using Camera + Gyroscope + GPS with heading correction from GPS velocity direction and periodic position resets.

## Results
- **Mean Error: 68.6m**
- Improvement over no-mag: **79%**
- Improvement over baseline: **88.9%**
- GPS path: 2020.4m

## Key Techniques

### 1. GPS Heading from Velocity
Instead of magnetometer, use GPS velocity direction for heading reference:
```python
gps_heading = atan2(gps_vy, gps_vx)
```
This works because direction of travel = heading (for vehicles).

### 2. Adaptive Alpha Complementary Filter
Trust GPS heading more when moving fast:
```python
if speed > 3.0 m/s:
    alpha = 0.95  # Trust gyro 95%
elif speed > 1.0 m/s:
    alpha = 0.98  # Trust gyro 98%
else:
    alpha = 0.995 # Mostly gyro (GPS unreliable at low speed)
```

### 3. Periodic Position Resets
Every 10 seconds, blend VIO position with GPS:
```python
position = 0.7 * gps_position + 0.3 * vio_position
```
This bounds position drift while keeping VIO smoothness.

## Key Observations

### GPS Heading Quality
- Works well at speeds > 3 m/s
- Unreliable at low speeds (GPS noise dominates)
- ~30 position resets over 308 seconds

### Remaining Issues
1. **Discrete resets cause jumps** - Not smooth fusion
2. **Fixed blend ratio** - Doesn't adapt to GPS accuracy
3. **No uncertainty modeling** - Can't weight measurements optimally

## Conclusion
GPS heading correction is very effective (79% improvement). However, the discrete reset approach is suboptimal - need continuous fusion.

## Next Step
Extended Kalman Filter for optimal fusion → see `slam-ekf/`
