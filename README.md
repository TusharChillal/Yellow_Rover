# Yellow Rover – Autonomous 4-Wheel Differential Drive Rover (ROS 2)

Yellow Rover is a research-oriented autonomous ground robot developed by converting a
4-wheel differential drive rover into a fully autonomous platform using **ROS 2 Jazzy**,
**Nav2**, and **slam_toolbox**.

The primary goal of this project is to build a **robust, modular, and extensible mobile robot**
that can be used for:
- Autonomous navigation research
- Localization and mapping experiments
- Complex task execution
- Future **multi-robot / swarm robotics** research

---

## System Overview

The robot performs autonomous navigation using:
- 2D LiDAR-based SLAM
- Lidar odometry fused with IMU data using an EKF
- Nav2 for planning, control, and recovery behaviors

The system is designed to be **hardware-agnostic**, **network-capable**, and **scalable** for
multi-robot setups.

---

## Hardware Configuration

| Component | Description |
|--------|------------|
| SBC | Raspberry Pi 5 (16GB RAM) |
| Motors | Waveshare DDSM115 Smart Motors |
| Motor Driver | Waveshare DDSM Driver HAT (UART → ESP32) |
| LiDAR | YDLiDAR G4 (2D) |
| IMU | Hiwonder 10-Axis IMU |
| Camera | OBSBOT Meet SE (Full HD USB Camera) |
| Chassis | Custom CNC-machined Aluminium Shell |

---

## Software Stack

| Layer | Technology |
|-----|------------|
| OS | Ubuntu + ROS 2 Jazzy |
| Mapping | slam_toolbox |
| Localization | AMCL |
| Navigation | Nav2 |
| Odometry | RF2O LiDAR Odometry |
| Sensor Fusion | robot_localization (EKF) |
| Motor Control | Custom C++ UART driver |
| MCU | ESP32 (on DDSM Driver HAT) |

---

## Localization & Odometry

- **Primary Odometry**: RF2O LiDAR odometry
- **Orientation Source**: IMU (yaw stabilization)
- **Fusion Method**: Extended Kalman Filter (EKF)

This combination provides:
- Smooth odometry
- Reduced wheel slip effects
- Stable localization for Nav2

---

## Navigation Stack

- **slam_toolbox**  
  Used for real-time mapping and map serialization.

- **AMCL**  
  Used for probabilistic localization on known maps.

- **Nav2**  
  Used for:
  - Global planning
  - Local control
  - Obstacle avoidance
  - Recovery behaviors

---

## Motor Control Architecture

- Custom **C++ ROS 2 node** handles velocity commands
- UART communication to ESP32 on the DDSM Driver HAT
- High-frequency, low-latency control loop
- Designed for deterministic motor response

This approach avoids Python overhead and improves real-time behavior.

---

## Repository Structure
Yellow_Rover/
├── camera_controll/ # Camera integration and utilities
├── cpp_motor/ # C++ motor control node
├── rover_description/ # URDF, meshes, and robot description
├── rf2o_laser_odometry/ # RF2O odometry (git submodule)
├── witmotion_IMU_ros/ # IMU driver (git submodule)
├── nav2_param.yaml # Nav2 parameters
├── mapper_param.yaml # slam_toolbox parameters


---

## Third-Party Packages

This repository uses the following open-source ROS 2 packages as **Git submodules**:

### rf2o_laser_odometry
- Author: MAPIR Lab
- Repository: https://github.com/MAPIRlab/rf2o_laser_odometry
- License: BSD

### witmotion_IMU_ros
- Author: Elettra Scientific Computing 
- Repository: https://github.com/ElettraSciComp/witmotion_IMU_ros
- License: MIT

All credit for these packages belongs to the original authors.

---

## Clone Instructions

```bash
git clone --recurse-submodules https://github.com/TusharChillal/Yellow_Rover.git

