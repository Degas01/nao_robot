# NAO Robot Autonomous Ball Retrieval System

<p align="center">
  <img width="1520" height="2389" alt="20250117_105948" src="https://github.com/user-attachments/assets/39595b3d-9c49-489c-b22a-3dc1fc295c8b" />
</p>

![Python](https://img.shields.io/badge/Python-2.7-blue)
![OpenCV](https://img.shields.io/badge/OpenCV-3.x-green)
![ROS](https://img.shields.io/badge/ROS-Indigo-blue)
![Ubuntu](https://img.shields.io/badge/Ubuntu-14.04-orange)
![NAO](https://img.shields.io/badge/NAO-V5-red)
![Gazebo](https://img.shields.io/badge/Simulator-Gazebo-yellow)
![MoveIt](https://img.shields.io/badge/Planning-MoveIt-purple)
![License](https://img.shields.io/badge/License-Academic-lightgrey)
[![King's College London](https://img.shields.io/badge/Institution-King's_College_London-blue.svg)](https://www.kcl.ac.uk/)
![Status](https://img.shields.io/badge/Status-Completed-brightgreen)

---

## Project Overview

This repository contains the full implementation of my Sensing and Perception Group Project at King’s College London:

> **NAO Robot Autonomous Ball Retrieval System**

*Sensing and Perception Group Project | King's College London | August 2025*

This project develops a comprehensive sensing and perception framework for the NAO V5 humanoid robot to autonomously detect, track, navigate to and kick a tennis ball. Inspired by RoboCup Soccer and tennis court ball kid assistance, the system integrates multiple robotics domains:

- **Computer Vision: Real-time ball detection and tracking using OpenCV**
- **Path Planning: Dynamic obstacle avoidance with A(star) algorithm**
- **SLAM: Sparse 3D reconstruction inspired by ORB-SLAM2**
- **Motion Planning: Custom kick kinematics with balance constraints**
- **Human-Robot Interaction: Voice command recognition system**

The robot was tested in Quad Lab, King's College London.

---

# Table of Contents

1. Project Objectives
2. System Architecture
3. Simulation Environment
4. Robot Model Description
5. Reinforcement Learning Framework (N/A)
6. Path Planning Framework (A*)
7. Heuristic & Cost Function Design
8. Implementation Details
9. Training / Tuning Pipeline
10. Experimental Results
11. Visual Demonstrations
12. Future Work
13. References

---

## 1. Project Objectives 

This project implements a fully autonomous navigation pipeline for the NAO humanoid robot, enabling the robot to:

- Detect a target object (tennis ball)
- Build and maintain a grid-based world representation
- Compute an optimal path using the A* algorithm
- Avoid static obstacles and reach the target reliably
- Execute the computed path in simulation and on a real NAO robot

---

## 2. System Architecture

```
┌─────────────────┐
│  Voice Command  │
│  Recognition    │
└────────┬────────┘
         │
         ▼
┌─────────────────┐      ┌──────────────┐
│  Ball Detection │◄─────┤ NAO Camera   │
│  (OpenCV)       │      └──────────────┘
└────────┬────────┘
         │
         ▼
┌─────────────────┐      ┌──────────────┐
│  Visual Tracking│◄─────┤ Head Control │
│  (Proportional) │      │ (ALProxy)    │
└────────┬────────┘      └──────────────┘
         │
         ▼
┌─────────────────┐      ┌──────────────┐
│  SLAM System    │◄─────┤ Feature      │
│  (ORB-based)    │      │ Extraction   │
└────────┬────────┘      └──────────────┘
         │
         ▼
┌─────────────────┐
│  Path Planning  │
│  (A* Algorithm) │
└────────┬────────┘
         │
         ▼
┌─────────────────┐      ┌──────────────┐
│  Motion         │◄─────┤ Kick         │
│  Execution      │      │ Kinematics   │
└─────────────────┘      └──────────────┘

```

The system consists of four primary layers:

### 1. Perception Layer

- Image-based ball detection (optional extension)
- Occupancy grid generation
- Static obstacle identification

### 2. Planning Layer

- A*-based global path planner
- Manhattan distance heuristic
- Node expansion, open/closed set management

### 3. Simulation Layer

- Webots for physics-based robot simulation
- RViz/Foxglove for visualising grid and planned path

### 4. Execution Layer

- NAOqi API for body movement
- Path smoothing and waypoint tracking

---

## 3. Simulation Environment 

The project integrates multiple tools:

### Webots

- Full NAO model
- Obstacle environment
- Tennis-ball placement
- Kinematic control

### RViz

- Grid visualisation
- Path expansion timeline
- Debugging of occupancy cells

### Foxglove

- Real-time monitoring
- Playback of navigation logs

