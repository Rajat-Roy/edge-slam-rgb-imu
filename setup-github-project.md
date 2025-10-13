# GitHub Project Board Setup

Since GitHub CLI (`gh`) is not available, here are the manual steps to set up your project board:

## 1. Create Project Board (Manual Steps)

1. Go to your GitHub repository: `https://github.com/[username]/edge-slam-rgb-imu`
2. Click on "Projects" tab
3. Click "Link a project" → "New project"
4. Choose "Board" template
5. Name: "Edge SLAM RGB-IMU Development"
6. Description: "MTech Project: Visual-Inertial SLAM pipeline for edge devices using RGB + IMU fusion"

## 2. Configure Kanban Columns

Replace default columns with:
- **Backlog** - Ideas, not yet started
- **To Do** - Committed tasks for current sprint
- **In Progress** - Actively working
- **Review/Testing** - Needs verification or cleanup
- **Done** - Completed tasks

## 3. Create Milestones

Go to Issues → Milestones → New milestone:

### Milestone 1: Setup & Data Collection
- **Due date:** October 31, 2025
- **Description:** Android app development and initial data collection

### Milestone 2: Sensor Survey & Literature Review
- **Due date:** November 30, 2025
- **Description:** Comprehensive sensor analysis and framework evaluation

### Milestone 3: Baseline SLAM Prototype
- **Due date:** March 31, 2026
- **Description:** Working SLAM pipeline with existing frameworks

### Milestone 4: Optimization & Edge Deployment
- **Due date:** June 30, 2026
- **Description:** GPU acceleration and mobile optimization

### Milestone 5: Final Benchmarking & Report
- **Due date:** November 30, 2026
- **Description:** Complete testing, documentation, and thesis submission

## 4. Create Labels

Go to Issues → Labels → New label:

- `app-dev` (Color: #1f77b4) - Mobile app tasks
- `slam-core` (Color: #ff7f0e) - SLAM algorithm tasks
- `sensor-fusion` (Color: #2ca02c) - IMU integration, filtering
- `optimization` (Color: #d62728) - GPU/TensorRT work
- `docs` (Color: #9467bd) - Notes, surveys, reports
- `demo` (Color: #8c564b) - Visualization, final presentation

## 5. Initial Issues to Create

### Issue 1: App: Camera + IMU logger MVP
- **Milestone:** Setup & Data Collection
- **Labels:** app-dev
- **Description:** Create Android app that records synchronized camera frames and IMU data

### Issue 2: Docs: Sensor Survey Draft
- **Milestone:** Sensor Survey & Literature Review
- **Labels:** docs
- **Description:** Research and document available sensors in modern smartphones

### Issue 3: Baseline: ORB-SLAM3 Integration
- **Milestone:** Baseline SLAM Prototype
- **Labels:** slam-core
- **Description:** Integrate collected data with ORB-SLAM3 and log pose output

### Issue 4: Optimization: Profile Jetson Nano
- **Milestone:** Optimization & Edge Deployment
- **Labels:** optimization
- **Description:** Measure FPS, CPU/GPU usage on target hardware

### Issue 5: Demo: GitHub Pages Setup
- **Milestone:** Final Benchmarking & Report
- **Labels:** demo
- **Description:** Create gh-pages branch for project status and demo showcase

## Alternative: GitHub CLI Setup

If you want to install GitHub CLI for future automation:

```bash
# macOS
brew install gh

# Then authenticate
gh auth login

# Create project (after CLI setup)
gh project create "Edge SLAM RGB-IMU Development" --body "MTech Project: Visual-Inertial SLAM pipeline for edge devices"
```