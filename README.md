# NAO Robot Autonomous Ball Retrieval System

<p align="center">
  <img width="1520" height="2389" alt="20250117_105948" src="https://github.com/user-attachments/assets/39595b3d-9c49-489c-b22a-3dc1fc295c8b" />
</p>

![Python](https://img.shields.io/badge/Python-2.7-blue)
![OpenCV](https://img.shields.io/badge/OpenCV-3.x-green)
![ROS](https://img.shields.io/badge/ROS-Noetic-blue)
![Ubuntu](https://img.shields.io/badge/Ubuntu-14.04-orange)
![NAO](https://img.shields.io/badge/NAO-V5-red)
![Gazebo](https://img.shields.io/badge/Simulator-Gazebo-yellow)
![MoveIt](https://img.shields.io/badge/Planning-MoveIt-purple)
![License](https://img.shields.io/badge/License-Academic-lightgrey)
[![King's College London](https://img.shields.io/badge/Institution-King's_College_London-blue.svg)](https://www.kcl.ac.uk/)

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
4. Technical Implementations
   4.1 Ball Detection & Tracking
   4.2 Path Planning (A* Algorithm)
   4.3 SLAM Implementation
   4.4 Kick Kinematics
   4.5 Speech Recognition
10. Training / Tuning Pipeline
11. Experimental Results
12. Visual Demonstrations
13. Future Work
14. References

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

<p align="center">
  <img width="252" height="242" alt="Screenshot 2025-12-11 225246" src="https://github.com/user-attachments/assets/9cac2305-2d5b-4ce1-a409-999777916a35" />
  <br>
  <em>RViz Simulation Environment</em>
</p>

### Foxglove

- Real-time monitoring
- Playback of navigation logs

### Gazebo + ROS Noetic + MoveIt

- More realistic integration with ROS tools and MoveIt planning
- More fragile on newer Ubuntu versions

<p align="center">
  <img width="580" height="263" alt="Screenshot 2025-12-11 225800" src="https://github.com/user-attachments/assets/4aa58c85-6538-4739-9c1d-38b08e64934b" />
  <br>
  <em>(1) Nao in Gazebo environment with ball (2) Covisibility graph of landmarks and robot camera trajectory</em>
</p>

---

## 4. Technical Implementations

### 4.1 Ball Detection & Tracking

Algorithm Steps:

- Image Acquisition: Capture RGB frames from NAO's camera (320×240 resolution)
- Color Filtering: Apply HSV color space conversion and yellow mask
- Noise Reduction: Morphological operations (erosion + dilation)
- Contour Detection: Identify closed contours using OpenCV
- Circle Validation: Filter circular contours and compute center coordinates

### Mathematical Model:

Proportional control for head tracking:

```
θ = k × (x - x_center)
```

Where:

θ = angular adjustment
k = proportional gain constant
x = ball center x-coordinate
x_center = image frame center

Distance estimation from radius:

```
distance ≈ f(radius) [inverse relationship]
```

### Visual Results

<p align="center">
  <img width="250" height="212" alt="Screenshot 2025-12-12 092626" src="https://github.com/user-attachments/assets/1fa3dc3b-14bd-4aef-bcf4-767cdd70c85c" />
  <br>
  <em>Ball Detection from the NAO camera </em>
</p>

<p align="center">
  <img width="352" height="83" alt="Screenshot 2025-12-12 093243" src="https://github.com/user-attachments/assets/a3015f1f-e2b4-4c6c-9f09-3436cce08e69" />
  <br>
  <em>Coordinates and radius of ball </em>
</p>

### Code Snippet

```python
# Ball Detection Core Logic
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

for contour in contours:
    ((x, y), radius) = cv2.minEnclosingCircle(contour)
    if radius > min_radius:
        cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
        theta = k * (x - x_center)  # Proportional control
```

### 4.2 Path Planning (A* Algorithm)

### Algorithm Overview

The A* implementation uses Manhattan distance heuristic for efficient pathfinding on a 2D grid:

```
h(n) = |x_n - x_goal| + |y_n - y_goal|
```

### Key Features

- 8-way movement (diagonal movement allowed)
- Dynamic obstacle detection and avoidance
- Real-time path replanning (15-50ms per obstacle)
- Optimal path reconstruction via parent node tracking

### Performance Metrics

<div align="center">
<table>

| Metric | A* Algorithm | Dijkstra Algorithm | Improvement |
|--------|--------------|--------------------|-------------|
| Success Rate |92% (50+ runs) | 88% | +4.5% |
| Path Length | Optimized | Baseline | 12% shorter |
| Replanning Time | 15-50ms | 25-70ms | 40% faster |
| Memory Usage | Moderate | High | Lower |

</table>
</div>

### Visual Results

<p align="center">
  <img width="370" height="288" alt="Screenshot 2025-12-12 094646" src="https://github.com/user-attachments/assets/70d9b4ae-cb33-4c92-92cb-c6ac1ab4a2a7" />
  <br>
  <em>2D Simulation of A* Algorithm </em>
</p>

### Code Implementation

```python
def a_star(start, goal, grid):
    open_set = PriorityQueue()
    open_set.put((0, start))
    came_from = {}
    g_score = {start: 0}
    
    while not open_set.empty():
        current = open_set.get()[1]
        
        if current == goal:
            return reconstruct_path(came_from, current)
        
        for neighbor in get_neighbors(current, grid):
            tentative_g = g_score[current] + 1
            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + manhattan_distance(neighbor, goal)
                open_set.put((f_score, neighbor))
```

### 4.3 SLAM Implementation

### ORB-SLAM2 Inspired Pipeline

This visual SLAM system adapts ORB-SLAM2 architecture to Python 2.7 constraints:

### Pipeline Stages:

1. Feature Extraction: ORB (Oriented FAST and Rotated BRIEF) feature detection (up to 3000 features)
2. Feature Matching: FLANN-based descriptor matching across frames
3. Motion Estimation: Essential matrix computation with RANSAC outlier rejection
4. Keyframe Selection: Add keyframes on significant camera translation
5. Triangulation: 3D point reconstruction from matched features
6. Loop Closure: Periodic global optimization (threshold: 10+ keyframes)
7. Map Building: Covisibility graph construction


