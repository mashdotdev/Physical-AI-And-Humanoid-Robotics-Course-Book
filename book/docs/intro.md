---
sidebar_position: 1
title: Welcome
---

# Physical AI and Humanoid Robotics

Welcome to **Physical AI and Humanoid Robotics**: a comprehensive, hands-on course for building intelligent humanoid robots from the ground up.

## Course Overview

This course bridges the gap between artificial intelligence research and physical robotics implementation, providing you with the skills to:

- Build and program humanoid robots using modern software stacks (ROS 2, NVIDIA Isaac Sim, PyTorch)
- Integrate vision-language-action (VLA) models with real-time robot controllers
- Design navigation, manipulation, and perception systems for real-world deployment
- Deploy AI models on edge compute platforms (NVIDIA Jetson Orin)
- Understand safety-critical robotics and human-robot interaction

## Target Audience

This course is designed for:
- **Software engineers** transitioning to robotics
- **ML engineers** wanting to deploy models on physical systems
- **Robotics students** seeking industry-relevant skills
- **Researchers** exploring embodied AI and humanoid robotics

### Prerequisites

- **Programming**: Proficient in Python (NumPy, PyTorch basics); C++ knowledge is a plus
- **Math**: Linear algebra (transforms, matrices), basic calculus, probability
- **Hardware**: Access to NVIDIA RTX GPU (recommended: RTX 4070 Ti or higher) or NVIDIA Jetson Orin Nano/NX for edge deployment
- **Operating System**: Ubuntu 22.04 LTS or WSL2 on Windows

## Course Structure

### Module 1: Foundations
**Chapter 1: The Robotic Nervous System (ROS 2)**
- Understanding robot middleware and communication paradigms
- Building ROS 2 nodes, topics, services, and actions
- Interfacing high-level AI with low-level controllers
- Visualizing robot structures with URDF

### Module 2: Perception (Coming Soon)
Vision systems, sensor fusion, and environment understanding

### Module 3: Motion and Control (Coming Soon)
Kinematics, dynamics, trajectory planning, and control theory

### Module 4: Navigation (Coming Soon)
Mapping, localization, path planning, and autonomous navigation

### Module 5: Manipulation (Coming Soon)
Grasping, object manipulation, and dexterous control

### Module 6: Vision-Language-Action Models (Coming Soon)
Integrating LLMs and VLAs with robot systems

### Module 7: Deployment and Safety (Coming Soon)
Edge compute, real-time constraints, and safety-critical systems

## Philosophy: Reality First, Simulation Second

This course adheres to a **"Simulation First, Reality Second"** approach:
1. **Develop in simulation** (NVIDIA Isaac Sim) for rapid iteration and safety
2. **Validate hardware alignment** (NVIDIA RTX GPUs, Jetson Orin)
3. **Deploy to physical systems** with minimal sim-to-real gap

We prioritize modern stacks (ROS 2, lifecycle nodes, behavior trees) over legacy approaches, ensuring your skills remain industry-relevant.

## Getting Started

Ready to begin? Proceed to **Chapter 1: The Robotic Nervous System (ROS 2)** to understand how robots communicate and coordinate software components.

### Development Environment Setup

Before starting, ensure you have:
- **ROS 2 Humble or Iron** installed on Ubuntu 22.04 LTS
- **NVIDIA Isaac Sim** (Omniverse) or Gazebo for simulation
- **Python 3.10+** with `rclpy`, NumPy, and PyTorch

Detailed setup instructions are provided in each chapter's quickstart guide.
