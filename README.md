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
5. Results & Performance
6. Installation & Setup
7. Demo Videos
8. Challenges & Solutions
9. Future Work
10. Acknowledgments
11. References

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

### Gazebo + ROS Noetic + MoveIt

- More realistic integration with ROS tools and MoveIt planning
- More fragile on newer Ubuntu versions

---

## 4. Technical Implementations

### 4.1 Ball Detection & Tracking

Algorithm Steps:

- Image Acquisition: Capture RGB frames from NAO's camera (320×240 resolution)
- Color Filtering: Apply HSV color space conversion and yellow mask
- Noise Reduction: Morphological operations (erosion + dilation)
- Contour Detection: Identify closed contours using OpenCV
- Circle Validation: Filter circular contours and compute center coordinates

#### Mathematical Model:

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

#### Visual Results

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

#### Code Snippet

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

#### Algorithm Overview

The A* implementation uses Manhattan distance heuristic for efficient pathfinding on a 2D grid:

```
h(n) = |x_n - x_goal| + |y_n - y_goal|
```

#### Key Features

- 8-way movement (diagonal movement allowed)
- Dynamic obstacle detection and avoidance
- Real-time path replanning (15-50ms per obstacle)
- Optimal path reconstruction via parent node tracking

#### Performance Metrics

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

#### Visual Results

<p align="center">
  <img width="370" height="288" alt="Screenshot 2025-12-12 094646" src="https://github.com/user-attachments/assets/70d9b4ae-cb33-4c92-92cb-c6ac1ab4a2a7" />
  <br>
  <em>2D Simulation of A* Algorithm </em>
</p>

#### Code Implementation

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

#### ORB-SLAM2 Inspired Pipeline

This visual SLAM system adapts ORB-SLAM2 architecture to Python 2.7 constraints:

Pipeline Stages:

1. Feature Extraction: ORB (Oriented FAST and Rotated BRIEF) feature detection (up to 3000 features)
2. Feature Matching: FLANN-based descriptor matching across frames
3. Motion Estimation: Essential matrix computation with RANSAC outlier rejection
4. Keyframe Selection: Add keyframes on significant camera translation
5. Triangulation: 3D point reconstruction from matched features
6. Loop Closure: Periodic global optimization (threshold: 10+ keyframes)
7. Map Building: Covisibility graph construction

#### Visual Results

<p align="center">
  <img width="580" height="263" alt="Screenshot 2025-12-11 225800" src="https://github.com/user-attachments/assets/4aa58c85-6538-4739-9c1d-38b08e64934b" />
  <br>
  <em>(1) Nao in Gazebo environment with ball (2) Covisibility graph of landmarks and robot camera trajectory</em>
</p>

#### Technical Details

ORB Feature Detection:

```python
orb = cv2.ORB_create(nfeatures=3000)
keypoints, descriptors = orb.detectAndCompute(image, None)
```

Feature Matching with FLANN:

```python
FLANN_INDEX_LSH = 6
index_params = dict(algorithm=FLANN_INDEX_LSH, table_number=6, 
                    key_size=12, multi_probe_level=1)
flann = cv2.FlannBasedMatcher(index_params, {})
matches = flann.knnMatch(desc1, desc2, k=2)
```

Essential Matrix & Camera Motion:

```python
E, mask = cv2.findEssentialMat(pts1, pts2, focal=focal, pp=(cx, cy), 
                                method=cv2.RANSAC, prob=0.999, threshold=1.0)
_, R, t, mask = cv2.recoverPose(E, pts1, pts2, focal=focal, pp=(cx, cy))
```

Particle Filter SLAM:

<p align="center">
  <img width="315" height="271" alt="Screenshot 2025-12-12 095629" src="https://github.com/user-attachments/assets/9f82ee20-e1a7-4a5b-95f1-62b049e9a602" />
  <br>
  <em>Particle filter-based SLAM showing belief map evolution and robot state estimation </em>
</p>

### 4.4 Kick Kinematics

#### Development Methodology

Development Steps:

1. Physical Teaching: Manually guide NAO's leg through desired kick motion
2. Joint Recording: Capture joint angles using Choregraphe timeline
3. Motion Refinement: Fine-tune keyframes for smooth trajectory
4. Balance Constraint: Weight shift to right leg + CoM recentering
5. Cartesian Control: End-effector position interpolation
6. Testing & Iteration: Validate stability and kick effectiveness

#### Joint Configuration

NAO Leg Degrees of Freedom (6 DOF per leg):

- Hip Yaw/Pitch: Position adjustment
- Hip Roll: Lateral movement
- Knee Pitch: Leg extension
- Ankle Pitch/Roll: Foot orientation

#### Visual Results

<p align="center">
  <img width="587" height="288" alt="Screenshot 2025-12-12 100240" src="https://github.com/user-attachments/assets/1722d13d-803d-475f-81d8-1997d4e6d9d6" />
  <br>
  <em>Kick motion visualization in Choregraphe showing successful execution </em>
</p>

#### Code Implementation

```python
# Balance and kick execution
motionProxy.wbFootState("Fixed", "RLeg")
motionProxy.wbEnableBalanceConstraint(True, "Legs")

# Cartesian interpolation for kick
effector = "LLeg"
space = motion.FRAME_ROBOT
path = [
    [0.0, 0.1, 0.05],   # Retract
    [0.15, 0.1, 0.05],  # Forward kick
    [0.0, 0.1, 0.0]     # Return
]
times = [1.0, 2.0, 3.0]
motionProxy.positionInterpolation(effector, space, path, 0x3f, times, True)
motionProxy.post.goToPosture("StandInit", 1.0)
```

Challenges:

- Center of gravity balance during single-leg support
- Preventing robot fall-over post-kick
- Timing coordination between leg and arm movements

### 4.5 Speech Recognition

#### Dual-Script Architecture

Due to Python 2.7 constraints on NAO, a novel dual-script system was implemented.

System Flow:

1. Script 1 (Python 3.12): Runs on laptop, captures microphone input
2. Speech Recognition: Processes audio using Google Speech API
3. File I/O: Writes transcription to shared .txt file
4. Script 2 (Python 2.7): Polls file, executes NAO commands via NAOqi
5. Cleanup: Clears file after command execution to manage memory

#### Code Implementation

```python
# Python 3.12 - Speech Recognition Script
import speech_recognition as sr

recognizer = sr.Recognizer()
with sr.Microphone() as source:
    audio = recognizer.listen(source)
    text = recognizer.recognize_google(audio)
    with open("command.txt", "w") as f:
        f.write(text)
```

```python
while True:
    if os.path.exists("command.txt"):
        with open("command.txt", "r") as f:
            command = f.read().strip()
        if command == "go get the ball":
            execute_ball_retrieval()
        open("command.txt", "w").close()  # Clear file
    time.sleep(0.5)
```

Limitations:

- Unable to run directly on NAO due to microphone compatibility issues
- Choregraphe simulation software incompatibility
- Workaround demonstrates concept but not fully integrated

---

## 5. Results & Performance

### Quantitative Results

<div align="center">
<table>

| Component | Metric | Performance | Notes |
|-----------|--------|-------------|-------|
| **Ball Detection** | Accuracy | 95%+ | Controlled lighting conditions |
| | Frame Rate | 15-20 FPS | 320×240 resolution |
| | Detection Range | 0.5m - 3m | Based on ball size |
| **Path Planning** | Success Rate | 92% | 50+ test runs |
| | Path Optimality | 12% better than Dijkstra | Length comparison |
| | Replanning Time | 15-50ms | Per obstacle update |
| **SLAM** | Feature Detection | Up to 3000 ORB features | Per frame |
| | Keyframe Threshold | 10+ frames | For global optimization |
| | Map Density | Sparse | Monocular constraints |
| **Kick Kinematics** | Success in Simulation | 100% | Choregraphe testing |
| | Real-world Stability | Unstable | Falls post-kick (needs tuning) |

</table>
</div>

### Qualitative Analysis

Strengths:

- Robust ball detection under varying ball positions
- Efficient path planning with obstacle avoidance
- Successful SLAM feature extraction and matching
- Modular, maintainable codebase
- Comprehensive documentation

Limitations:

- Legacy Python 2.7 constraints limit modern libraries
- Kick kinematics require fine-tuning for stability
- SLAM trajectory distortion due to incomplete loop closure
- Speech recognition not fully integrated with NAO
- Limited testing time with physical robot

---

## 6. Installation & Setup

### Prerequisites

#### Hardware:

- NAO V5 Humanoid Robot
- Computer running Ubuntu 14.04 (for ROS Indigo compatibility)
- Minimum 4GB RAM, 20GB storage

#### Software:

```bash
- Python 2.7.x (NAO compatibility)
- Python 3.12+ (Speech recognition)
- ROS Indigo
- NAOqi SDK 2.1.4.13
- OpenCV 3.x
- NumPy 1.x
- Gazebo 2.x
- MoveIt
- Choregraphe 2.1.4
```

#### Installation Steps:

1. **Clone Repository**

```bash
git clone https://github.com/Degas01/nao_robot.git
cd nao_robot
```

2. **Set Up Python 2.7 Environment (NAO)**

```bash
virtualenv -p python2.7 venv_nao
source venv_nao/bin/activate
pip install -r requirements.txt
```

3. **Set Up Python 3.12 Environment (Speech)**

```bash
python3.12 -m venv venv_speech
source venv_speech/bin/activate
pip install -r requirements_py312.txt
```

4. **Install ROS Indigo & Dependencies**

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-get update
sudo apt-get install ros-indigo-desktop-full
sudo apt-get install ros-indigo-naoqi-driver
sudo apt-get install ros-indigo-moveit
```

5. **Build ROS Workspace**

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
ln -s /path/to/nao-autonomous-ball-retrieval .
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

6. **Install Gazebo & NAO Models**

```bash
sudo apt-get install gazebo2
cd ~/catkin_ws/src
git clone https://github.com/ros-naoqi/nao_meshes.git
git clone https://github.com/ros-naoqi/nao_robot.git
catkin_make
```

### Autonomous Robot Stages

1. Initialize NAO robot connection
2. Start ball detection module
3. Wait for voice command "go get the ball"
4. Begin visual tracking and SLAM
5. Compute path using A*
6. Navigate to ball location
7. Execute kick when in range
8. Return to start position

---

## 7. Demo Videos

### 1. Ball Tracking Video

<p align="center">
  <video src="https://github.com/user-attachments/assets/78851a74-bb32-476a-bcdc-904b24dd233a" width="700" controls muted autoplay loop></video>
</p>

### 2. Kicking Movement Video

<p align="center">
  <video src="https://github.com/user-attachments/assets/12ef1f56-3e4c-48d9-9fec-f6cf9229e90d" width="600" width="700" controls muted autoplay loop></video>
</p>

### 3. 2D simulation of A* algorithm with obstacle avoidance

<p align="center">
  <video src="https://github.com/user-attachments/assets/45ac992e-4d0d-40f4-b758-635e9e911529" width="600" width="700" controls muted autoplay loop></video>
</p>

### 4. Choreograph Simulation

<p align="center">
  <video src="https://github.com/user-attachments/assets/665fcb40-f7b5-4906-9631-4b5793b1dcd5" width="600" width="700" controls muted autoplay loop></video>
</p>

### 5. Speech Recognition Showcase

<p align="center">
  <video src="https://github.com/user-attachments/assets/ffee81fd-8cf6-4344-9e7b-cfa20cec1210" width="600" width="700" controls muted autoplay loop></video>
</p>

---

## 8. Challenges & Solutions

### Challenge 1: Legacy Python 2.7 Constraints

#### Problem:

- NAO requires Python 2.7 and NAOqi SDK, incompatible with modern libraries (YOLO, TensorFlow)
- pip package ecosystem deprecated for Python 2.7

#### Solution:

- Use OpenCV 3.x (last version supporting Python 2.7) for ball detection
- Implement ORB-SLAM2 pipeline from scratch using available libraries
- Create dual-script architecture for speech recognition (Python 3.12 ↔ Python 2.7)

#### Impact: 

Increased development complexity but ensured NAO compatibility

### Challenge 2: Kick Stability

#### Problem:

- NAO falls over after executing kick motion in real world
- Center of gravity shifts excessively during single-leg balance

#### Solution Attempts:

- Implemented weight shift to supporting leg using wbFootState
- Added balance constraints with wbEnableBalanceConstraint
- Manual joint fine-tuning (ongoing)
- Future: Predictive balance model with IMU integration

#### Current Status:

Works in simulation, requires further real-robot tuning

### Challenge 3: SLAM Trajectory Distortion

#### Problem:

- Camera trajectory shows significant drift over time
- Loop closure mechanism incomplete, causing accumulated error

#### Solution:

- Implement bag-of-words (BoW) approach for better loop detection
- Integrate IMU data for motion prediction (ORB-SLAM3 approach)
- Add bundle adjustment optimization after loop closure

#### Workaround:

Sparse map still useful for local navigation (0-5m range)

### Challenge 4: Gazebo-MoveIt Integration

#### Problem:

- MoveIt unable to update NAO joint poses dynamically in Gazebo
- Planned trajectories execute in Rviz but not in simulated robot

#### Root Cause: 

ROS Indigo + Gazebo 2.x compatibility issues with NAO controller

#### Solution:

- Test kick planning separately in Rviz (visual validation)
- Execute pre-computed trajectories via Python scripts
- Use Choregraphe for kinematic validation

#### Recommendation:

Upgrade to ROS Noetic + Gazebo 11 (requires NAO SDK update)

### Challenge 5: Speech Recognition Integration

#### Problem:

- NAO's onboard microphone undetectable by speech recognition libraries
- Choregraphe audio modules incompatible with external Python scripts

#### Solution:

- Use laptop microphone for speech capture (Python 3.12)
- File-based communication between Python 3.12 and Python 2.7 scripts
- NAO executes commands from parsed text file

#### Limitation: 

Not fully autonomous (requires external laptop)

---

## 9. Future Work

### Short-Term Improvements (1-3 months)

1. **Kick Stability Enhancement**

 - Integrate Kalman filter for balance prediction
 - Add ZMP (Zero Moment Point) calculation for dynamic stability
 - Implement adaptive kick force based on ball distance
 - Test with various ball positions and weights

2. **SLAM Optimization**

 - Implement bag-of-words for robust loop closure
 - Add bundle adjustment after every N keyframes
 - Integrate IMU data for motion prior (ORB-SLAM3 style)
 - Dense reconstruction using patch-based stereo

3. **Path Planning Enhancements**

 - Add dynamic replanning for moving obstacles
 - Implement RRT* for complex environments
 - Integrate SLAM map directly into A* cost function
 - Test in outdoor tennis court environment

### Medium-Term Goals (3-6 months)

4. **Multi-Ball Tracking**

 - Extend detection to handle multiple balls simultaneously
 - Prioritize closest ball using depth estimation
 - Implement ball sorting strategy (e.g., nearest-first)

5. **Human Interaction**

 - Gesture recognition for commands (waving, pointing)
 - Ball handoff detection using pressure sensors
 - Natural language dialogue system

6. **Energy Efficiency**

 - Optimize gait for battery conservation
 - Sleep mode when idle
 - Periodic recharging behavior

### Long-Term Vision (6+ months)

7. **RoboCup Soccer Integration**

 - Multi-agent coordination with other NAO robots
 - Opponent detection and avoidance
 - Goal recognition and scoring strategy

8. **Deep Learning Integration**

 - Replace OpenCV with YOLO v8 ball detection (requires Python 3.x migration)
 - Deep reinforcement learning for kick optimization
 - Neural SLAM (e.g., Neural Recon)

9. **Full Autonomy**

 - Eliminate external laptop dependency for speech
 - Onboard edge computing module (e.g., Jetson Nano)
 - 5G connectivity for cloud offloading

### Research Directions

- Multi-modal Fusion: Combine vision, IMU, and pressure sensors for robust state estimation
- Sim-to-Real Transfer: Train policies in simulation, deploy on real robot
- Explainable AI: Visualize decision-making process for debugging and trust

---

## 10. Acknowledgments

### King's College London
    *Provided NAO robot and lab facilities (Quad Lab)*
    
### Aldebaran Robotics (SoftBank)
    *NAO robot platform and NAOqi SDK*

### ROS Community
    *ROS Indigo, Gazebo, MoveIt packages*

### Team Members
    *Harry Braganza, Hitesh Anavai, Mohammad Islam and Kriti Chauhan*
    
### OpenCV Community
    *Computer vision library*
    
### ORB-SLAM2 Authors
    *Raúl Mur-Artal and Juan D. Tardós for SLAM architecture inspiration*
    
### Course Instructors
    *Dr. Oya Celiktutan and teaching assistants for guidance*
    
### RoboCup SPL Community
    *Resources and documentation*

---

## 11. References

1. RoboCup Standard Platform League. https://spl.robocup.org/
2. Li, Q. & Zhao, Y. (2024). "Tennis Ball Recognition in Complex Scenes Based on Improved YOLOv5." ICAACE. DOI: 10.1109/icaace61206.2024.10548503
3. Leiva, L.A. et al. (2018). "Playing soccer without colors in the SPL: A convolutional neural network approach." arXiv:1811.12493.
4. Bradski, G. (2008). Learning OpenCV: Computer Vision with the OpenCV Library. O'Reilly.
5. Baevski, A. et al. (2020). "wav2vec 2.0: A Framework for Self-Supervised Learning of Speech Representations." arXiv:2006.11477.
6. Hart, P., Nilsson, N., & Raphael, B. (1968). "A Formal Basis for the Heuristic Determination of Minimum Cost Paths." IEEE Transactions on Systems Science and Cybernetics, 4(2), 100-107.
7. Kalman, R.E. (1960). "A New Approach to Linear Filtering and Prediction Problems." Journal of Basic Engineering, 82(1), 35-45.
