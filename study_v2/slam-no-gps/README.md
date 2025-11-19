# SLAM without GPS

## Overview
Pure Visual-Inertial Odometry (VIO) using only Camera + Gyroscope. No GPS corrections.

## Results
- **Mean Error: 383.9m**
- Max Error: 704.6m
- Final Error: 320.8m
- **Drift Rate: 15.9% of path length**
- GPS path: 2020.4m

## Why Test Without GPS?
To understand how much GPS contributes to accuracy and what pure dead reckoning achieves.

## Sensors Used
- Camera: Optical flow for velocity
- Gyroscope: Heading integration
- GPS: Only for scale calibration (post-hoc, not real-time)

## Key Observations

### Scale Problem
Without GPS, scale is unknown. We used GPS path length post-hoc:
```python
scale = gps_path_length / vio_path_length = 1.3005
```
In real scenarios, this would need alternative calibration.

### Drift Analysis
- Error grows linearly with distance
- At 2km: ~320m drift
- Rate: 16% of traveled distance

### Heading Drift Dominates
The main error source is gyroscope heading drift:
- ~3° drift over 5 minutes
- Causes trajectory to curve away from truth
- Position integrates heading error over distance

### Trajectory Shape
The VIO trajectory has similar "shape" to GPS but:
- Rotated due to heading drift
- Offset accumulates over time
- End point ~320m from truth

## Why GPS is Critical

| Metric | Without GPS | With EKF+GPS |
|--------|-------------|--------------|
| Mean Error | 383.9m | 16.4m |
| Improvement | - | **96%** |

GPS provides:
1. **Absolute position reference** - Bounds drift
2. **Continuous corrections** - Prevents error accumulation
3. **Scale calibration** - Converts pixels to meters

## Conclusion
Pure VIO drifts ~16% of path length. For a 2km trajectory, this is unacceptable (320m error). **GPS is essential** for bounding position errors on longer trajectories.

## Key Insight
The fundamental limitation of dead reckoning:
> Without absolute position reference, errors always accumulate unboundedly.

## Next Step
Add accelerometer to see if it helps → see `slam-ekf-accel/`
