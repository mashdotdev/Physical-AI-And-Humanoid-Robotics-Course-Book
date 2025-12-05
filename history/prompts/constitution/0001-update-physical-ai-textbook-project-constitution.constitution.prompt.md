---
id: 0001
title: Update Physical AI Textbook Project Constitution
stage: constitution
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: N/A
command: /sp.constitution
labels: ["constitution", "update"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- .specify/memory/constitution.md
tests:

---

## Prompt

```text
<command-message>sp.constitution is running…</command-message>
<command-name>/sp.constitution</command-name>
<command-args>
<!--
Sync Impact Report:
Version change: 1.0.0 → 1.0.1
List of modified principles: Clarified book creation format.
Added sections: None
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md: ✅ updated
- .specify/templates/spec-template.md: ✅ updated
- .specify/templates/tasks-template.md: ✅ updated
- .specify/templates/commands/*.md: ✅ updated
Follow-up TODOs: None
-->
# Physical AI Textbook Project Constitution

Preamble

We hold these truths to be self-evident: that the future of Artificial Intelligence is not merely digital, but embodied. This project exists to bridge the chasm between the "Digital Brain" (LLMs, Agents) and the "Physical Body" (Robots). Our mission is to forge a new generation of engineers capable of designing, simulating, and deploying humanoid robots that understand and interact with the physical world.

## Article I: Mission & Vision

### Section 1.01 Mission

To create the definitive, open-source textbook that teaches students how to build Physical AI systems. This book transforms "Chat AI" developers into "Robotics AI" engineers by teaching them to integrate Large Language Models (LLMs) with the Robot Operating System (ROS 2) and NVIDIA's Isaac platform.

### Section 1.02 Vision

A world where intelligent agents escape the browser to assist humans in the physical world. We envision a curriculum where a student can type "Clean the room" and have a physical robot understand, plan, and execute that command.

## Article II: Target Audience & Personas

### Section 2.01 The "Digital Native" Convert

**Background**: Strong in Python, APIs, and perhaps Generative AI. Weak in physics, electronics, and kinematics.

**Goal**: Wants to see their code "move" something real.

**Pain Points**: Intimidated by C++, confused by coordinate frames (quaternions), and fears hardware costs.

### Section 2.02 The "Hardware Hobbyist" Up-Skiller

**Background**: Experience with Arduino/Raspberry Pi. Comfortable with wires and motors.

**Goal**: Wants to add "Intelligence" to their machines.

**Pain Points**: Knows how to make it move, but not how to make it think or see.

## Article III: Core Operating Principles

### Simulation First, Reality Second

We adhere to the "Digital Twin" philosophy. If it doesn't work in NVIDIA Isaac Sim, it does not exist.

### Strict Hardware Alignment

We do not write generic code. We write code specifically optimized for NVIDIA RTX GPUs and Jetson Orin edge devices.

### Modern Stack Only

No ROS 1. No legacy approaches. We use ROS 2 (Humble/Iron), Lifecycle Nodes, and Behavior Trees.

### Agentic Workflow

Every major project must integrate AI Agents (LLMs) to handle high-level logic, leaving low-level control to ROS.

## Article IV: Technology Standards

The content of this book must strictly adhere to the following technology stack. Deviations are not permitted without amendment.

### Section 4.01 Software Stack

**Middleware**: ROS 2 (Humble or Iron) on Ubuntu 22.04 LTS.

**Simulation**: NVIDIA Isaac Sim (Omniverse), Gazebo, Unity.

**AI & Perception**: NVIDIA Isaac ROS, Nav2 (Navigation Stack), OpenAI Whisper (Voice).

**Cognitive Layer**: OpenAI GPT-4o / LLMs for Vision-Language-Action (VLA).

**Language**: Python 3.10+ (Primary), C++ (Secondary/Advanced).

### Section 4.02 Hardware Targets (The "Physical" Constraint)

**Workstation**: NVIDIA RTX 4070 Ti (12GB VRAM) minimum for Isaac Sim.

**Edge Compute**: NVIDIA Jetson Orin Nano / Orin NX.

**Sensors**: Intel RealSense D435i/D455 (RGB-D), LiDAR.

**Robots**: Unitree Go2 (Quadruped Proxy), Unitree G1 (Humanoid), or Custom Servos (Hiwonder).

## Article V: Content Architecture

The book is structured into four distinct modules, mirroring the "Quarter Overview" of the Physical AI Course.

### Module 1: The Robotic Nervous System (Weeks 1-5)

**Theme**: "The Body without the Brain."

**Scope**: Installing Ubuntu/ROS 2, understanding Nodes/Topics/Services, creating the URDF (Unified Robot Description Format) for a humanoid.

**Key Deliverable**: A simulated robot arm that responds to basic vector commands.

### Module 2: The Digital Twin (Weeks 6-7)

**Theme**: "The World inside the Computer."

**Scope**: Physics simulation in Gazebo. Importing assets into Unity. Simulating gravity, friction, and IMU/LiDAR sensors.

**Key Deliverable**: A robot falling, balancing, and seeing its environment in a virtual world.

### Module 3: The AI-Robot Brain (Weeks 8-10)

**Theme**: "Perception is Reality."

**Scope**: NVIDIA Isaac Sim & Isaac ROS. VSLAM (Visual Simultaneous Localization and Mapping). Nav2 for path planning (getting from A to B without hitting C).

**Key Deliverable**: A robot that maps a room and navigates it autonomously.

### Module 4: Vision-Language-Action (Weeks 11-13)

**Theme**: "The Ghost in the Machine."

**Scope**: VLA Models. Integrating OpenAI Whisper for voice commands. Using LLMs to generate ROS 2 action plans.

**Key Deliverable**: Capstone Project - The Autonomous Humanoid. A robot that hears a command, plans the task, and executes it.

## Article VI: Stylistic Guidelines

### Voice

The tone is authoritative yet encouraging. We are engineers speaking to engineers. Avoid academic fluff. Use "we build," "we deploy," not "one might consider."

### Code Blocks

All code must be runnable. Python scripts must include type hinting. ROS 2 launch files must be complete XML/Python.

### Visuals

Every concept involving space (coordinates, transforms) requires a diagram description. Every simulation step requires a screenshot placeholder.

### "Sim-to-Real" Warning

Every chapter must contain a "Reality Check" callout box explaining why the code might fail on real hardware (battery sag, friction, lighting) vs. simulation.

## Article VII: Deliverables & Integration

### Section 7.01 The Book Artifact

**Format**: Docusaurus static site.

**Features**: Integrated RAG Chatbot (using OpenAI Agents SDK, Neon Postgres, Qdrant).

**Personalization**: User sign-in (Better-Auth) to tailor content based on hardware availability (e.g., "Show me the path for Jetson Nano" vs "Show me the path for RTX 4090").

### Section 7.02 Reusable Intelligence

We will generate "Agent Skills" via Claude Code Subagents that allow readers to automate parts of the setup process (e.g., an agent that writes the URDF file for them).

## Governance

This Constitution supersedes all other project practices and documentation. Amendments require thorough documentation, explicit approval from project leads, and a clear migration plan for any affected systems or processes. All pull requests and code reviews must verify compliance with these principles. Justification for any increased complexity is mandatory.

**Prompt History Records (PHR)**: Every user input must be recorded verbatim in a Prompt History Record after every user message. PHRs are routed to `history/prompts/constitution/` for constitution-related prompts, `history/prompts/<feature-name>/` for feature-specific prompts, and `history/prompts/general/` for general prompts.

**Architectural Decision Records (ADR)**: When an architecturally significant decision is detected, a suggestion to document it will be made, but never auto-created. This typically occurs during planning and task generation.

**Version**: 1.0.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-05
```

## Response snapshot

```text
Updated the project constitution at `.specify/memory/constitution.md` to version 1.0.1. The version bump is a PATCH release due to clarified book creation format.
```

## Outcome

- ✅ Impact: Constitution updated and synced.
- 🧪 Tests: N/A
- 📁 Files: Updated .specify/memory/constitution.md
- 🔁 Next prompts: N/A
- 🧠 Reflection: Successfully updated the constitution and generated PHR.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
