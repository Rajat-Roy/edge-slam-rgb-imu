# SLAM and Multi-Sensor Fusion Literature Survey

**Author**: Rajat Roy
**Project**: Efficient Visual-Inertial SLAM on Edge Devices (RGB + IMU + GPS)
**Date**: October 2025
**Supervisor**: Dr. Hardik Jain, IIT Jodhpur

---

## Abstract

This document presents a comprehensive literature survey on Simultaneous Localization and Mapping (SLAM) with focus on multi-sensor fusion approaches combining visual, inertial, and global navigation satellite system (GNSS) data. The survey covers fundamental SLAM paradigms, recent advances in visual-inertial odometry (VIO), GPS-visual-inertial integration, and mobile/edge computing considerations for real-time deployment.

---

## 1. Introduction

SLAM has evolved from single-sensor approaches to sophisticated multi-sensor fusion systems that leverage complementary strengths of different sensors. The integration of visual cameras, inertial measurement units (IMUs), and GPS/GNSS systems enables robust localization in diverse environments and addresses the limitations of individual sensors.

### 1.1 Research Motivation

- **Visual-only SLAM**: Suffers from scale ambiguity, motion blur sensitivity, and poor performance in low-texture environments
- **IMU-only Integration**: Provides high-frequency motion estimates but drifts over time
- **GPS Integration**: Offers global positioning constraints but suffers from urban canyon effects and indoor unavailability
- **Multi-sensor Fusion**: Combines complementary strengths to achieve robust, accurate, and continuous localization

---

## 2. Foundational SLAM Approaches

### 2.1 Visual SLAM Paradigms

**Overview**: Visual SLAM systems use cameras to help devices understand their position and build maps of surroundings by analyzing visual features like corners, edges, and textures.

#### Feature-Based Methods
- **ORB-SLAM3** (Campos et al., 2021): State-of-the-art feature-based system supporting monocular, stereo, and RGB-D cameras with IMU integration
  - **Overview**: Recognizes distinctive landmarks and patterns in images to track position, similar to how humans recognize familiar places to navigate
  - Key innovation: Co-visibility graph optimization for superior accuracy
  - Supports multi-map SLAM and automatic map merging
  - Tight coupling in local/global optimization, loose coupling in initialization

#### Direct Methods
- **DSO** (Engel et al., 2018): Direct Sparse Odometry
  - **Overview**: Uses entire image pixel patterns to track movement instead of specific landmarks, following the flow of textures in a video
  - Operates directly on pixel intensities without feature extraction
  - Advantages: Better performance in low-texture environments
  - Limitations: Sensitive to illumination changes

#### Semi-Direct Methods
- **SVO** (Forster et al., 2014): Semi-Direct Visual Odometry
  - **Overview**: Combines landmark recognition with overall image pattern tracking, offering balanced approach for fast and accurate positioning
  - Hybrid approach combining features and direct methods
  - Suitable for micro-aerial vehicles with computational constraints

### 2.2 Visual-Inertial SLAM (VI-SLAM)

**Overview**: Visual-Inertial SLAM combines camera vision with motion sensors to track movement more accurately, similar to how humans use both their eyes and sense of balance to navigate.

#### Tight vs. Loose Coupling
- **Loose Coupling**: Separate processing pipelines with late fusion
  - **Overview**: Uses separate navigation systems for landmarks and motion sensing, combining results at the end
  - Example: Early ORB-SLAM versions with separate VIO integration
  - Simpler implementation but suboptimal performance

- **Tight Coupling**: Joint optimization of visual and inertial measurements
  - **Overview**: Continuously combines visual and motion data in real-time, making navigation more natural and accurate
  - Current state-of-the-art approach
  - Better handling of rapid motions and sensor failures

#### Leading VI-SLAM Systems

**VINS-Mono/Fusion** (Qin et al., 2018):
- **Overview**: Complete positioning system using one camera combined with motion sensors, providing personal navigation that works even indoors
- First complete monocular visual-inertial system
- Tight coupling throughout the pipeline
- Sliding window optimization for real-time performance
- Supports GPS integration in VINS-Fusion

**ORB-SLAM3** (Campos et al., 2021):
- **Overview**: Advanced system that recognizes landmarks and uses motion sensors together, capable of creating maps of entire buildings and remembering them for future visits
- Latest advancement in visual-inertial SLAM
- Superior accuracy due to co-visibility graph optimization
- Supports multiple camera types and multi-session mapping

**MSCKF** (Mourikis and Roumeliotis, 2007):
- **Overview**: Foundational approach that treats camera features and motion measurements as puzzle pieces, gradually building complete movement picture over time
- Multi-State Constraint Kalman Filter
- Foundation for many modern VIO systems
- EKF-based approach with delayed feature initialization

---

## 3. GPS/Visual-Inertial Fusion

**Overview**: GPS/Visual-Inertial Fusion combines satellite positioning, camera vision, and motion sensors to create a complete navigation system that works both outdoors (with GPS) and indoors (without GPS), providing multiple methods for position determination.

### 3.1 Integration Strategies

#### Loose Coupling Approaches
- **Overview**: Uses GPS coordinates as anchor points to correct visual-inertial navigation, similar to checking GPS occasionally while following directions indoors
- GPS provides position constraints to VI-SLAM
- Simple implementation but doesn't fully exploit sensor correlations
- Suitable for basic navigation applications

#### Tight Coupling Approaches
- **Overview**: Continuously blends GPS signals with visual and motion data to create smooth navigation system that handles GPS dropouts seamlessly
- Joint optimization of GPS, visual, and inertial measurements
- Better handling of GPS-denied environments
- Improved accuracy and robustness

### 3.2 Recent Advances

**LGVINS** (Irfan et al., 2024):
- **Overview**: Comprehensive system using lasers, GPS, cameras, and motion sensors together, providing versatile navigation tools for drones that can handle any situation
- LiDAR-GPS-Visual-Inertial System
- Multi-sensor fusion for UAV applications
- Provides integrity and redundancy for autonomous navigation

**M2C-GVIO** (Yu et al., 2024):
- **Overview**: Smart navigation for cars that understands road constraints and traffic patterns, using this knowledge to improve GPS-camera-motion sensor accuracy
- Motion Manifold Constraint aided GNSS-Visual-Inertial Odometry
- Designed for ground vehicle applications
- Exploits vehicle motion constraints for improved accuracy

**GPC-LIVO** (Ye et al., 2025):
- **Overview**: Latest evolution adding laser scanning to create detailed 3D maps while tracking position with high accuracy
- Point-wise LiDAR-inertial-visual odometry
- Recent advancement in multi-sensor fusion
- Demonstrates the trend towards comprehensive sensor integration

### 3.3 GPS-Challenged Environments

**Urban Canyons**:
- **Overview**: Tall buildings that block or reflect GPS signals like a maze, causing GPS to jump around or lose signal completely
- Multi-path reflections and signal blockage
- Solution: Visual-inertial bridging during GPS outages

**Indoor Navigation**:
- **Overview**: Complete GPS unavailability indoors, as in basements where satellites cannot reach, forcing navigation to rely entirely on visual landmarks and motion sensors
- Complete GPS unavailability
- Solution: Visual-inertial SLAM with WiFi/magnetic field augmentation

---

## 4. Mobile and Edge Computing Considerations

**Overview**: Mobile Edge Computing enables complex SLAM systems to work on phones and tablets by intelligently managing processing power, distributing heavy calculations between the device and cloud while handling basic tracking locally.

### 4.1 Computational Constraints

#### Real-Time Requirements
- **Overview**: System needs to process data as fast as human vision (30-60 times per second) to eliminate lag between movement and map updates
- **Target Frame Rates**: 30-60 Hz for interactive applications
- **Latency Requirements**: <50ms for responsive user experience
- **Memory Constraints**: Mobile devices typically 4-8GB RAM

#### Optimization Strategies
- **Overview**: Intelligent selection of which landmarks to remember and which to discard, keeping only essential information to avoid overwhelming the processor
- **Feature Selection**: Adaptive feature extraction based on computational budget
- **Keyframe Selection**: Smart selection to reduce optimization complexity
- **Cloud Offloading**: Processing heavy computations on edge servers

### 4.2 Mobile SLAM Frameworks

#### ARCore (Google)
- **Overview**: Google's toolkit that enables Android apps to understand and map 3D spaces, powering AR experiences like placing virtual furniture in rooms
- Cross-platform AR SDK for Android and iOS
- Built-in visual-inertial odometry
- Environmental understanding capabilities
- Limited extensibility for research purposes

#### ARKit (Apple)
- **Overview**: Apple's AR framework for iPhones and iPads, known for highly accurate tracking and smooth AR experiences
- iOS-only AR framework
- High-performance visual-inertial tracking
- Scene reconstruction capabilities
- Closed-source, limited research access

#### Open-Source Mobile Solutions
- **Overview**: Custom-built navigation systems that researchers can modify and study, providing blueprints to understand and improve SLAM functionality
- **VINS-Mobile**: Mobile-optimized VINS implementation
- **ORB-SLAM3 Mobile**: Resource-constrained version
- Custom implementations using OpenCV and Android native libraries

### 4.3 Edge Computing Integration

**Trine Framework** (2024):
- **Overview**: Intelligent system that distributes work between phones, nearby computers, and cloud, utilizing each component's strengths
- Cloud-Edge-Device cooperative video analysis
- Demonstrates potential for SLAM computation offloading
- Adaptive processing based on network conditions

**MEC (Mobile Edge Computing)**:
- **Overview**: Uses nearby computers instead of distant cloud servers to reduce delay, providing local processing capabilities
- Low-latency computation offloading
- Real-time SLAM processing with cloud support
- Challenges: Network latency and bandwidth constraints

---

## 5. Datasets and Benchmarks

**Overview**: Datasets provide standardized test data for researchers to compare different SLAM systems fairly, using real-world recordings with known correct positions.

### 5.1 Standard SLAM Datasets

#### Visual-Inertial Datasets
- **EuRoC MAV Dataset** (Burri et al., 2016):
  - **Overview**: Drone flight recordings with cameras and motion sensors, providing perfect GPS tracks to compare against when testing navigation algorithms
  - Stereo visual-inertial data from MAV flights
  - Ground truth from motion capture systems
  - Standard benchmark for VI-SLAM evaluation

- **TUM RGB-D Dataset** (Sturm et al., 2012):
  - **Overview**: Indoor recordings from Microsoft Kinect cameras capturing both color and depth, ideal for testing SLAM systems inside buildings
  - RGB-D sequences with IMU data
  - Large-scale indoor environments
  - Well-suited for visual-inertial evaluation

#### Multi-Sensor Datasets
- **KITTI Vision Benchmark** (Geiger et al., 2012):
  - **Overview**: Car driving recordings with multiple sensors (cameras, lasers, GPS) testing SLAM systems in real-world traffic conditions
  - Autonomous driving scenarios
  - Stereo, LiDAR, and GPS data
  - Challenging outdoor environments

- **TUM Visual-Inertial Dataset**:
  - **Overview**: Collection of challenging indoor recordings specifically designed to test visual-inertial systems, like obstacle courses for navigation algorithms
  - Diverse sequences in different scenes
  - Specifically designed for VI odometry evaluation
  - Challenging conditions including dynamic objects

### 5.2 Recent Datasets (2024-2025)

**InCrowd-VI** (Bamdad et al., 2024):
- **Overview**: Recordings of people walking through crowded indoor areas, testing SLAM systems in busy environments like shopping malls or train stations
- Visual-inertial dataset for human navigation
- Indoor pedestrian-rich environments
- 5 hours of real-world footage

**Monado SLAM Dataset** (2025):
- **Overview**: Massive collection of real-world recordings testing SLAM systems in everyday situations, providing hundreds of different routes through various environments
- 64 non-calibration recordings
- 5 hours and 15 minutes of footage
- Challenging real-world scenarios

---

## 6. Performance Metrics and Evaluation

**Overview**: Performance metrics provide standardized ways to measure SLAM system quality, evaluating accuracy, speed, and efficiency.

### 6.1 Accuracy Metrics

#### Trajectory Error
- **Overview**: Measures deviation between estimated and actual paths, similar to checking route tracing accuracy against GPS tracks
- **Absolute Trajectory Error (ATE)**: Overall path accuracy
- **Relative Pose Error (RPE)**: Local consistency
- **Scale Error**: Critical for monocular systems

#### Mapping Quality
- **Overview**: Evaluates completeness and accuracy of created maps, like verifying floor plan correctness and room coverage
- **Map Completeness**: Coverage of mapped environment
- **Map Accuracy**: Correctness of reconstructed geometry
- **Loop Closure Detection**: Robustness to revisited areas

### 6.2 Computational Metrics

#### Runtime Performance
- **Overview**: Measures system speed and resource usage, evaluating if navigation apps cause overheating or rapid battery drain
- **Frame Processing Time**: Critical for real-time applications
- **Memory Usage**: Peak and average memory consumption
- **CPU/GPU Utilization**: Resource efficiency metrics

#### Energy Efficiency
- **Overview**: Tracks power consumption of SLAM systems, monitoring if navigation apps last through extended trips
- **Battery Consumption**: Important for mobile deployment
- **Power Usage**: Per-sensor power requirements
- **Thermal Management**: Device heating considerations

---

## 7. Research Gaps and Future Directions

### 7.1 Identified Research Gaps

#### Multi-Sensor Integration
- **Optimal Fusion Strategies**: Need for adaptive sensor weighting
- **Sensor Failure Handling**: Graceful degradation during sensor loss
- **Calibration Automation**: Online calibration for multi-sensor systems

#### Edge Computing Challenges
- **Computational Optimization**: Balance between accuracy and efficiency
- **Network-Dependent Processing**: Handling variable connectivity
- **Heterogeneous Hardware**: Adapting to different device capabilities

### 7.2 Future Research Directions

#### Deep Learning Integration
- **Learned Feature Extraction**: Deep features for better matching
- **End-to-End SLAM**: Neural SLAM approaches
- **Sensor Fusion Networks**: Deep multi-sensor integration

#### Robust SLAM Systems
- **Dynamic Environment Handling**: SLAM in crowded scenes
- **All-Weather Operation**: Performance in adverse conditions
- **Long-Term Localization**: Mapping across seasonal and appearance changes

#### Edge-Optimized Solutions
- **Adaptive Algorithms**: Quality-performance trade-offs
- **Distributed SLAM**: Collaborative mapping across devices
- **Quantized Models**: Efficient deployment on resource-constrained devices

---

## 8. Implementation Considerations for Edge SLAM

### 8.1 Hardware Optimization

#### Processor Selection
- **CPU vs GPU**: Balance between serial and parallel processing
- **NPU Integration**: Neural processing units for deep learning components
- **DSP Usage**: Digital signal processors for sensor preprocessing

#### Memory Management
- **Streaming Processing**: Avoid loading entire datasets
- **Frame Buffer Optimization**: Efficient image storage and access
- **Garbage Collection**: Minimize runtime overhead

### 8.2 Software Architecture

#### Modular Design
- **Sensor Abstraction**: Unified interface for different sensors
- **Pluggable Algorithms**: Easy algorithm comparison and switching
- **Configuration Management**: Runtime parameter adjustment

#### Real-Time Constraints
- **Thread Prioritization**: Critical path optimization
- **Lock-Free Data Structures**: Minimize synchronization overhead
- **Frame Dropping**: Graceful handling of computational overload

---

## 9. Conclusion

This literature survey demonstrates the maturity of visual-inertial SLAM and the growing importance of multi-sensor fusion approaches. Key findings include:

1. **Tight coupling** of sensors provides superior performance compared to loose coupling approaches
2. **ORB-SLAM3** represents the current state-of-the-art in feature-based VI-SLAM
3. **GPS integration** is essential for large-scale applications but requires careful handling of outages
4. **Edge computing constraints** significantly impact algorithm design and implementation choices
5. **Deep learning integration** presents promising future directions but requires careful computational budgeting

The proposed edge SLAM system with RGB-IMU-GPS integration aligns with current research trends and addresses the growing need for robust localization in mobile applications.

---

## References

1. Campos, C., Elvira, R., Rodríguez, J. J. G., Montiel, J. M. M., & Tardós, J. D. (2021). ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM. *IEEE Transactions on Robotics*.

2. Qin, T., Li, P., & Shen, S. (2018). VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator. *IEEE Transactions on Robotics*.

3. Engle, J., Schöps, T., & Cremers, D. (2018). Direct Sparse Odometry. *IEEE Transactions on Pattern Analysis and Machine Intelligence*.

4. Burri, M., Nikolic, J., Gohl, P., Schneider, T., Rehder, J., Omari, S., ... & Siegwart, R. (2016). The EuRoC micro aerial vehicle datasets. *The International Journal of Robotics Research*.

5. Irfan, M., et al. (2024). LGVINS: LiDAR-GPS-Visual and Inertial System Based on Multi-Sensor Fusion for Autonomous UAV Operation. *IEEE Sensors Journal*.

6. Ye, C., et al. (2025). GPC-LIVO: Point-wise LiDAR-inertial-visual odometry with Geometric Point Correspondence. *Robotics and Autonomous Systems*.

7. Bamdad, M., et al. (2024). InCrowd-VI: A Realistic Visual–Inertial Dataset for Human Navigation in Crowded Indoor Environments. *Sensors*.

8. Sturm, J., Engelhard, N., Endres, F., Burgard, W., & Cremers, D. (2012). A benchmark for RGB-D SLAM systems. *IEEE/RSJ International Conference on Intelligent Robots and Systems*.

9. Yu, Y., et al. (2024). M2C-GVIO: Motion Manifold Constraint aided GNSS-Visual-Inertial Odometry for Ground Vehicles. *IEEE Transactions on Intelligent Transportation Systems*.

10. Forster, C., Pizzoli, M., & Scaramuzza, D. (2014). SVO: Fast Semi-Direct Monocular Visual Odometry. *IEEE International Conference on Robotics and Automation*.

---

**Document Status**: Draft 1.0
**Last Updated**: October 13, 2025
**Next Review**: December 2025