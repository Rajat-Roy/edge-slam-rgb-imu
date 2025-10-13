# SLAM MVP - Visual-Inertial SLAM Pipeline

A minimal viable product implementation of visual-inertial SLAM using data collected from the Android SLAM Logger app.

## Overview

This SLAM MVP processes synchronized RGB camera frames and IMU sensor data to estimate device trajectory using three approaches:

1. **Visual Odometry**: Camera-based motion estimation using ORB features
2. **IMU Integration**: Inertial navigation with accelerometer and gyroscope
3. **Sensor Fusion**: Weighted combination of visual and inertial estimates

## Features

- **Data Loading**: Parses Android SLAM Logger dataset format
- **Visual Processing**: ORB feature detection and matching
- **IMU Integration**: Dead reckoning with bias calibration
- **Sensor Fusion**: Simple weighted combination strategy
- **Visualization**: 3D trajectory plots and performance analysis
- **Metrics**: Comprehensive motion and accuracy statistics

## Requirements

```bash
# Python packages (installed in virtual environment)
opencv-python>=4.12.0
numpy>=2.2.0
matplotlib>=3.10.0
scipy>=1.16.0
scikit-learn>=1.7.0
```

## Usage

### Setup
```bash
# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install opencv-python numpy matplotlib scipy scikit-learn
```

### Run SLAM MVP
```bash
# Ensure data is in slam-mvp/data/collected_data/
python run_slam_mvp.py
```

### Expected Output
```
=== SLAM MVP - Visual-Inertial SLAM ===
Processing Android data collection...

Loaded session: slam_session_20250923_090335
Duration: 23.5s
Frames: 608
IMU samples: 9391
...
SLAM MVP completed successfully!
Results saved to: output/slam_results.png
```

## Implementation Details

### Visual Odometry (`src/visual_odometry.py`)
- ORB feature detection (1000 features)
- Brute-force feature matching with cross-check
- Essential matrix estimation using RANSAC
- Pose recovery and trajectory integration

### IMU Integration (`src/imu_integration.py`)
- Static bias calibration (2-second window)
- Gyroscope integration for orientation
- Accelerometer integration with gravity compensation
- Position tracking through double integration

### Data Loader (`src/data_loader.py`)
- Parses Android JSON format (pretty-printed)
- Synchronizes camera and IMU timestamps
- Provides interpolation for temporal alignment
- Estimates camera intrinsic parameters

### SLAM Pipeline (`src/slam_pipeline.py`)
- Coordinates visual and inertial processing
- Implements sensor fusion strategy
- Generates comprehensive analysis and visualization
- Exports results in multiple formats

## Results Summary

### Performance Metrics (Example Dataset)
- **Visual Odometry**: 99.00m total distance, 11.13m max displacement
- **IMU Integration**: 612.27m total distance (with drift)
- **Fused SLAM**: 69.29m total distance, improved accuracy

### Generated Outputs
- `output/slam_results.png`: Visualization plots
- `output/slam_results.json`: Raw trajectory data
- Analysis summary with performance statistics

## File Structure

```
slam-mvp/
├── src/
│   ├── data_loader.py          # Dataset parsing and synchronization
│   ├── visual_odometry.py      # ORB-based visual tracking
│   ├── imu_integration.py      # Inertial navigation
│   └── slam_pipeline.py        # Main SLAM coordination
├── data/
│   └── collected_data/         # Android dataset (copied from logger)
├── output/                     # Generated results and plots
├── venv/                       # Python virtual environment
├── run_slam_mvp.py            # Main execution script
└── README.md                   # This file
```

## Technical Notes

### Camera Model
- Estimated intrinsic matrix for mobile camera
- Focal length: ~80% of image width
- Principal point: Image center
- No distortion correction (simplified model)

### IMU Calibration
- Static bias estimation from initial 2 seconds
- Gravity vector magnitude validation
- Simple constant bias model

### Coordinate Systems
- Camera: Right-handed (X: right, Y: down, Z: forward)
- IMU: Device frame aligned with camera
- World: Gravity-aligned (Z: up)

### Known Limitations
- Simple fusion strategy (no optimal estimation)
- No loop closure detection
- Constant IMU bias assumption
- Monocular scale ambiguity (resolved through motion)

## Future Improvements

1. **Advanced Fusion**: Extended Kalman Filter implementation
2. **Loop Closure**: Place recognition and pose graph optimization
3. **Bundle Adjustment**: Global trajectory refinement
4. **Real-time Processing**: Optimization for live deployment
5. **Robust Features**: Alternative feature descriptors
6. **IMU Preintegration**: More sophisticated inertial modeling

## Research Context

This SLAM MVP serves as a baseline implementation for the Edge SLAM RGB-IMU project, demonstrating:
- Feasibility of mobile visual-inertial SLAM
- Quality of Android sensor data for research
- Foundation for advanced algorithm development
- Benchmark for optimization and deployment phases

## Dependencies

- **OpenCV**: Computer vision and feature processing
- **NumPy**: Numerical computation and array operations
- **Matplotlib**: Visualization and plotting
- **SciPy**: Scientific computing and spatial transformations
- **scikit-learn**: Optional machine learning utilities

## Contributing

This implementation focuses on clarity and educational value. For production deployment, consider:
- Computational optimization
- Memory management improvements
- Error handling and robustness
- Real-time performance tuning