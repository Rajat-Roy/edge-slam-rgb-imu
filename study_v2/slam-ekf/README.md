# SLAM with Extended Kalman Filter

## Overview
Optimal sensor fusion using Extended Kalman Filter (EKF) for VIO + GPS.

## Results
- **Mean Error: 17.8m**
- Improvement over GPS-corrected: **74%**
- Improvement over baseline: **96%**
- GPS path: 2020.4m

## EKF Design

### State Vector
```
x = [x, y, vx, vy, heading]
```
- Position (x, y) in meters
- Velocity (vx, vy) in m/s
- Heading in radians

### Prediction Step
Uses gyroscope and optical flow:
1. Update heading: `heading += omega * dt`
2. Compute velocity from optical flow in world frame
3. Update position: `pos += vel * dt`
4. Update covariance: `P = F*P*F' + Q*dt`

### GPS Update Step
Kalman update with GPS position measurement:
```python
H = [[1,0,0,0,0], [0,1,0,0,0]]  # Observe x, y
R = diag([accuracy², accuracy²])  # Measurement noise
K = P * H' * inv(H*P*H' + R)  # Kalman gain
x = x + K * (gps - H*x)  # State update
P = (I - K*H) * P  # Covariance update
```

### Heading Correction from Velocity
Soft update heading based on velocity direction:
```python
if speed > 2.0 m/s:
    heading_from_vel = atan2(vy, vx)
    heading += 0.1 * angle_diff(heading_from_vel, heading)
```

## Key Observations

### Continuous vs Discrete Updates
- 321 GPS updates (every ~1 second)
- Mean innovation: 22.4m (prediction error before update)
- Smooth trajectory without jumps

### Uncertainty Bounded
- Starts at ~1m uncertainty
- Grows during prediction
- Reduced by GPS updates
- Stabilizes around 8m (1-sigma)

### Optimal Weighting
EKF automatically weights measurements by:
- GPS accuracy (from phone)
- Current state uncertainty
- Process noise model

## Why EKF Works Better

1. **Continuous fusion** - No discrete resets
2. **Uncertainty tracking** - Knows when to trust each sensor
3. **Optimal gain** - Mathematically optimal Kalman gain
4. **Smooth output** - Natural low-pass filtering

## Parameters Used
```python
P_init = diag([1, 1, 1, 1, 0.1])  # Initial uncertainty
Q = diag([0.5, 0.5, 0.5, 0.5, 0.01])  # Process noise
```

## Conclusion
EKF provides **96% improvement** over baseline by optimally fusing VIO and GPS. This is the best result achieved.

## Next Step
Test without heading correction → see `slam-ekf-no-heading/`
