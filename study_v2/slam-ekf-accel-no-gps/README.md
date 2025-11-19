# SLAM with EKF + Accelerometer (No GPS)

## Overview
EKF with accelerometer but NO GPS corrections - pure dead reckoning with IMU-enhanced motion model.

## Results
- **Mean Error: 596.6m**
- Max Error: 1010.6m
- Final Error: 715.6m
- **Drift Rate: 35.4% of path**
- GPS path: 2020.4m

## Critical Finding
**Accelerometer made things WORSE!**

| Method | Mean Error | Drift Rate |
|--------|------------|------------|
| VIO only (no accel) | 383.9m | 15.9% |
| EKF+Accel (no GPS) | **596.6m** | **35.4%** |

## Why Accelerometer Hurt Performance

### 1. Noise Amplification
Phone accelerometer has high noise (±3-4 m/s²). Without GPS to correct errors:
- Noise integrates into velocity
- Velocity error integrates into position
- Double integration = squared noise growth

### 2. Velocity Corruption
The 30% accelerometer weight corrupts optical flow velocity:
```python
vx = 0.7 * vx_flow + 0.3 * (vx + ax * dt)
```
With noisy `ax`, this adds error to good optical flow estimates.

### 3. Unbounded Uncertainty
EKF uncertainty grows to **3147m** without GPS updates:
- No measurements to reduce covariance
- Process noise accumulates
- Kalman gain becomes meaningless

## Key Insight
> Accelerometer is only useful WITH absolute position corrections.

Without GPS to bound errors, accelerometer noise dominates and makes dead reckoning worse. The optical flow alone is more stable.

## When Accelerometer Would Help
1. **With GPS corrections** - Noise gets corrected
2. **High-quality IMU** - Tactical/navigation grade
3. **Short durations** - Before noise accumulates
4. **ZUPT** - Zero-velocity updates at stops

## Conclusion
**Do NOT use accelerometer for dead reckoning on phones.** The consumer-grade sensor is too noisy. It only helps when GPS or other absolute references can correct the integrated errors.

## Comparison Summary
| Method | Mean Error | Notes |
|--------|------------|-------|
| VIO only | 383.9m | Optical flow + gyro |
| EKF+Accel (no GPS) | 596.6m | **Worse due to accel noise** |
| EKF+Accel+GPS | 16.0m | GPS bounds errors |

## Lesson Learned
The accelerometer's value is:
- **With GPS**: Marginal (2% improvement)
- **Without GPS**: Negative (55% worse!)

For phone-based SLAM, **GPS is the critical sensor**, not accelerometer.
