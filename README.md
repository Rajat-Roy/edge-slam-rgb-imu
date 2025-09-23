# Efficient Visual-Inertial SLAM on Edge Devices (RGB + IMU)

This repository hosts the **MTech Project (IIT Jodhpur, 2025–26)** of Rajat Roy under the supervision of **Dr. Hardik Jain**.  
The project explores **RGB + IMU fusion** for real-time SLAM, optimized for mobile and Jetson-class edge devices.

---

## 📌 Project Overview
- **Objective:** Design a **visual-inertial SLAM pipeline** robust to real-world noise and deployable on edge hardware.  
- **Scope:**
  - Semester 1:  
    - Build a mobile app to collect **RGB + IMU data**.  
    - Survey what other mobile sensors (barometer, magnetometer, depth APIs) can support SLAM.  
    - Review existing mobile/edge SLAM frameworks (ARCore, ARKit, VINS-Mono, ORB-SLAM3).  
  - Semester 2:  
    - Implement a full **SLAM pipeline** (tracking + mapping + loop closure).  
    - Optimize using **CUDA/TensorRT, pruning, quantization**.  
    - Benchmark on standard datasets (TUM, EuRoC, KITTI) and mobile logs.

---

## 🛠️ Tech Stack
- **Languages:** C++14/17, Python, Kotlin (Android app).  
- **Frameworks:** ROS2, OpenCV, PyTorch/TensorRT.  
- **Mobile SDKs:** ARCore (Android), Sensor APIs.  
- **Simulation/Testing:** Gazebo, Isaac Sim, EuRoC/TUM datasets.  
- **Hardware:** Android mobile, Jetson Nano (2GB), Mac M1, GTX desktop.

---

## 🚀 Deliverables
- Mobile app for sensor logging.  
- Literature + sensor survey.  
- Prototype SLAM pipeline.  
- CUDA/TensorRT optimized benchmarks.  
- Documentation + reproducible code.  

---

## 📅 Timeline
- **Months 1–2:** Baseline setup, data collection app, surveys.  
- **Months 3–4:** Sensor fusion with RGB+IMU, optional depth.  
- **Months 5–6:** CUDA/TensorRT optimization, quantization.  
- **Months 7–8:** Benchmarking on datasets.  
- **Months 9–10:** Refinements, robustness testing, final report & demo.

---

## 📒 Meeting Log
Progress notes and supervision logs are maintained [here](prof_meeting_log.md).

---

## 🎓 Career Alignment
This project doubles as a **flagship portfolio piece**, aligned with my career goal to become a **GPU-accelerated Computer Vision Engineer**.  
More details: [Career Strategy](mtech_cv_job_plan.md)

---

## 📜 License
TBD (MIT/Apache 2.0, depending on institutional requirements).
