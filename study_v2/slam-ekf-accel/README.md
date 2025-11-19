# SLAM with EKF + Accelerometer

## Overview
EKF SLAM with accelerometer added to improve velocity estimation.

## Results
- **Mean Error: 16.0m**
- vs EKF without accel: **2.3% better**
- GPS path: 2020.4m

## Accelerometer Integration

### Processing Pipeline
1. **Gravity removal**: Estimate from first 1 second stationary
2. **Low-pass filter**: 5Hz cutoff to reduce noise
3. **Coordinate transform**: Body frame to forward/right

### Velocity Fusion
Blend optical flow velocity with accelerometer:
```python
alpha_accel = 0.3  # 30% accelerometer weight
vx = 0.7 * vx_flow + 0.3 * (vx + ax * dt)
vy = 0.7 * vy_flow + 0.3 * (vy + ay * dt)
```

## Key Observations

### Marginal Improvement
Only 2.3% better than without accelerometer. Why?

1. **Noisy sensor**: Linear accel range ±3-4 m/s² after filtering
2. **Optical flow sufficient**: Already provides good velocity
3. **GPS dominates**: Position corrections bound errors anyway

### Accelerometer Quality
```
Gravity magnitude: 9.80 m/s² (good)
Linear accel noise: High variability even after 5Hz filter
```

### When Accelerometer Helps More
- Camera occlusion (tunnel, darkness)
- Low-texture environments
- Rapid acceleration/braking
- Higher update rate fusion

### Velocity Smoothing
Accelerometer does help smooth velocity estimates between optical flow samples, visible in velocity plot.

## Comparison
| Method | Mean Error |
|--------|------------|
| EKF without accel | 16.4m |
| EKF with accel | **16.0m** |

## Conclusion
Accelerometer provides minimal improvement (0.4m) when:
- GPS is available for position correction
- Optical flow already estimates velocity well

The phone's consumer-grade accelerometer is too noisy to significantly improve dead reckoning. Its main value is in scenarios where optical flow fails.

## Next Step
Test accelerometer without GPS → see `slam-ekf-accel-no-gps/`
