# ISRO IROC-U Rover — Manipulation Arm: VLA + Inverse Kinematics

Software stack for the ISRO Rover & Robotics Competition – University
(IROC-U) 2024. Focused on autonomous manipulation: detecting objects
and executing pick-and-place using a Vision-Language-Action (VLA) model
combined with a geometric inverse kinematics solver.

**Result:** Advanced through prototype and ideation rounds.

---

## Problem

Design an autonomous manipulation arm on a rover capable of detecting,
reaching, and picking objects in an unstructured environment — simulating
extraterrestrial surface operations where teleoperation latency makes
manual control impractical.

---

## System Architecture

```
Camera (RGB)
    │
    ▼
MediaPipe Object Detection
(on-device, Raspberry Pi)
    │
    ▼
3D Position Estimation
(pixel → robot base frame via camera intrinsics + depth)
    │
    ├──────────────────────────┐
    ▼                          ▼
Geometric IK Solver      VLA Policy
(4-DOF analytical)       (visuomotor end-to-end)
    │                          │
    └──────────┬───────────────┘
               ▼
    Joint Angle Commands
               │
               ▼
    ESP32 Servo Controller
               │
               ▼
    Physical Arm (4-DOF)
```

---

## Key Components

### Perception — MediaPipe
- Deployed MediaPipe object detection on Raspberry Pi for real-time
  on-device inference (target <100ms latency)
- Used landmark detection to estimate object centroid and orientation
- Calibrated pixel-to-3D mapping using camera intrinsics and known
  arm base reference frame transformation

### Manipulation — VLA (Vision-Language-Action)
- Trained a VLA model to map visual observations directly to arm
  joint commands for pick-and-place tasks
- Training data: teleoperated demonstrations of pick-and-place sequences
- End-to-end visuomotor policy — handles generalization to novel object
  positions without requiring explicit geometric re-specification
- Used as primary policy for generalization; IK solver as fallback
  for precise waypoint targets

### Inverse Kinematics — Geometric Solver
- Analytical geometric IK for a 4-DOF revolute arm
- Given target (x, y, z) in robot base frame, solves for joint
  angles θ1–θ4 using trigonometric decomposition
- Handles joint limit constraints and singularity avoidance
- Used for trajectory waypoint interpolation and as IK fallback

### Integration
- ROS2 nodes for perception pipeline, IK solver, arm controller
- ESP32 receives joint commands over UART, drives servo PWM signals
- Real-time feedback loop at ~10Hz

---

## Hardware

| Component | Role |
|-----------|------|
| Raspberry Pi 5 | Main compute — ROS2, perception, VLA inference |
| ESP32 | Servo PWM control, joint angle execution |
| RGB Camera | Object detection input for MediaPipe + VLA |
| 4-DOF Arm | Revolute joint manipulator |
| Servo motors | Joint actuation |

---

## Competition

**ISRO IROC-U 2024** — organized by the Indian Space Research Organisation.
Advanced through **prototype and ideation rounds**.
Certificate: available in portfolio.
