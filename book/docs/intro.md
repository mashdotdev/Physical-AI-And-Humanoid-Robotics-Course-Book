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

### Module 1: The Robotic Nervous System
**Chapter 1: ROS 2 Fundamentals (Weeks 1-5)**
- Understanding robot middleware and communication paradigms
- Building ROS 2 nodes, topics, services, and actions
- Interfacing high-level AI with low-level controllers
- Visualizing robot structures with URDF
- **Key Deliverable**: A simulated robot arm that responds to basic vector commands

### Module 2: The Digital Twin
**Chapter 2: Physics Simulation and Virtual Environments (Weeks 6-7)**
- Physics simulation in Gazebo (gravity, friction, collisions)
- High-fidelity rendering in Unity for stakeholder demos and HRI testing
- ROS 2 control integration (hardware-agnostic controllers)
- Sensor simulation (LiDAR, depth cameras, IMU) with realistic noise models
- Complete digital twin pipeline orchestration
- **Key Deliverable**: A robot falling, balancing, and seeing its environment in a virtual world

### Module 3: The AI-Robot Brain (Coming Soon)
**Scope**: NVIDIA Isaac Sim & Isaac ROS, VSLAM, Nav2 (Weeks 8-10)
- Visual Simultaneous Localization and Mapping (VSLAM)
- Nav2 for autonomous path planning and navigation
- GPU-accelerated perception with Isaac ROS
- **Key Deliverable**: A robot that maps a room and navigates it autonomously

### Module 4: Vision-Language-Action Models (Coming Soon)
**Scope**: VLA Models, Whisper, LLM Integration (Weeks 11-13)
- OpenAI Whisper for voice command processing
- Large Language Models (LLMs) generating ROS 2 action plans
- Vision-Language-Action model integration
- **Key Deliverable**: Capstone Project - The Autonomous Humanoid (hears command, plans task, executes it)

## Philosophy: Simulation First, Reality Second

This course adheres to the **"Digital Twin"** philosophy:
1. **Develop in simulation** (Gazebo, Unity, NVIDIA Isaac Sim) for rapid iteration and safety
2. **Validate hardware alignment** (NVIDIA RTX GPUs, Jetson Orin)
3. **Deploy to physical systems** with minimal sim-to-real gap

We prioritize modern stacks (ROS 2, lifecycle nodes, behavior trees) over legacy approaches, ensuring your skills remain industry-relevant.

## Getting Started

Ready to begin? Proceed to **Chapter 1: The Robotic Nervous System (ROS 2)** to understand how robots communicate and coordinate software components.

### Development Environment Setup

Before starting, ensure you have:
- **ROS 2 Humble or Iron** installed on Ubuntu 22.04 LTS
- **Gazebo Classic or NVIDIA Isaac Sim** for simulation
- **Python 3.10+** with `rclpy`, NumPy, and PyTorch

Detailed setup instructions are provided in each chapter's content.
