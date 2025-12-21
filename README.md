# Yellow Bot – Autonomous 4-Wheel Differential Drive Rover 

**Yellow Rover** is a research-oriented autonomous ground robot developed by converting a
4-wheel differential drive rover into a fully autonomous platform using **ROS 2 Jazzy**,
**slam_toolbox**, and **Nav2**.

The project is designed as a **modular, extensible robotics research platform**, with a
long-term goal of enabling **complex autonomous tasks** and **future multi-robot (swarm)
experiments**.

---

## Autonomous Navigation Demo

**Video: Autonomous Navigation using Nav2**

> *(Add your video link here — YouTube / Drive / GitHub video)*

The demo shows real-world autonomous navigation including localization,and goal-directed motion.

---

## System Overview

The robot achieves autonomy through:
- 2D LiDAR-based SLAM
- LiDAR odometry fused with IMU data using an EKF
- Map-based localization and navigation using Nav2

---

## Hardware Configuration

| Component | Description |
|--------|------------|
| SBC | Raspberry Pi 5 (16GB RAM) |
| Motors | Waveshare DDSM115 Smart Motors |
| Motor Driver | Waveshare DDSM Driver HAT (ESP32-based) |
| LiDAR | YDLiDAR G4 (2D) |
| IMU | Hiwonder 10-Axis IMU |
| Camera | OBSBOT Meet SE (USB Full HD) |
| Chassis | Custom Aluminium Rover Shell |

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
| Motor Control | Custom C++ ROS 2 node |
| Teleoperation | PS5 joystick (C++) |

---

## Localization & Navigation

- **Primary odometry** is obtained from RF2O LiDAR odometry
- **IMU yaw-rate** is fused using an EKF to improve rotational stability
- **slam_toolbox** is used for mapping
- **AMCL** is used for localization on known maps
- **Nav2** handles global planning, local control, and recovery behaviors

Wheel odometry is intentionally avoided due to slip in 4WD platforms.

---

## Motor Control (`cpp_motor`)

- Implemented as a **ROS 2 C++ node**
- Subscribes to `/cmd_vel`
- Computes individual wheel velocities for a 4-wheel differential drive
- Sends commands via **UART** to an ESP32 on the DDSM Driver HAT
- Motors and driver provide **built-in closed-loop control**

This design keeps ROS-side control lightweight, deterministic, and fully compatible with Nav2.

---

## Teleoperation

- Dedicated **C++ PS5 joystick node**
- Runs on a desktop/laptop
- Publishes velocity commands to `/cmd_vel`
- Commands transmitted over Wi-Fi to the rover

Manual and autonomous control share the same interface, allowing seamless switching.

---

## Vision & Perception (`camera_controll`)

- Python-based camera publisher with compressed image streaming
- Lightweight YOLOv11 object detection node
- Detection results published as ROS topics
- Intended for perception experiments and future semantic navigation work

---


## Third-Party Packages

This repository includes the following open-source ROS 2 packages :

- **rf2o_laser_odometry**  
  Author: MAPIR Lab  
  Repository: https://github.com/MAPIRlab/rf2o_laser_odometry  

- **witmotion_IMU_ros**  This now reads like something from:
  Author: Elettra SciComp  
  Repository: https://github.com/ElettraSciComp/witmotion_IMU_ros  


All credit belongs to the original authors.

---

## Clone Instructions

```bash
git clone --recurse-submodules https://github.com/TusharChillal/Yellow_Rover.git


