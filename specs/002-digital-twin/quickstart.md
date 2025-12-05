# Quickstart: Digital Twin Chapter Development

**Feature**: 002-digital-twin
**Date**: 2025-12-05
**Purpose**: Fast-track guide for implementing Chapter 2 content.

---

## Prerequisites

- Ubuntu 22.04 LTS with ROS 2 Humble installed
- Gazebo Classic 11.x installed (`sudo apt install gazebo`)
- Unity 2021.3 LTS or newer (for Section 2.3-2.4)
- NVIDIA GPU (recommended, RTX 4070 Ti or better for Unity)
- Chapter 1 completed (readers have URDF model)

---

## 30-Second Overview

**What We're Building**: A Docusaurus chapter teaching readers to create a synchronized digital twin pipeline: Gazebo physics + Unity rendering + ROS 2 control + sensor simulation.

**Key Deliverables**:
1. 7 Docusaurus `.mdx` files (sections 2.1-2.7)
2. 15-20 code examples (URDF, launch files, Python scripts, Unity C#)
3. 5-7 diagrams (architecture, data flow, TF trees)
4. 4 end-of-chapter exercises with validation criteria

---

## Development Workflow

### Phase 0: Research (Completed)

✅ Resolved all NEEDS CLARIFICATION items:
- Gazebo plugins: `gazebo_ros_pkgs` 3.7.x
- Unity: URDF Importer v0.7.0+, ROS TCP Connector
- Launch files: Hierarchical Python launch structure
- See `research.md` for details

### Phase 1: Design (Completed)

✅ Data model defined (`data-model.md`):
- 7 key entities: URDF Model, Physics Engine, ROS 2 Control, Unity Rendering, Sensors, Launch Config, TF Tree
- Entity relationships and validation rules documented

✅ Contracts defined (`contracts/`):
- Gazebo URDF schema with sensor plugins
- ros2_control YAML configuration structure
- Unity scene hierarchy and required scripts
- Launch file templates

### Phase 2: Implementation Tasks (Next: `/sp.tasks`)

The `/sp.tasks` command will generate a task list based on:
- 27 functional requirements from `spec.md`
- 6 user stories (prioritized P1, P2, P3)
- Entity definitions from `data-model.md`
- Contract schemas from `contracts/`

**Expected Task Structure** (preview):

```text
Section 2.1: Digital Twin Concepts
├── Task 1.1: Write introduction with real-world examples
├── Task 1.2: Create digital-twin-architecture.svg diagram
└── Task 1.3: Add "Looking Ahead" callout (Isaac Sim transition)

Section 2.2: Gazebo Physics Simulation
├── Task 2.1: Write installation instructions (Gazebo Classic)
├── Task 2.2: Create empty_world.world file
├── Task 2.3: Write URDF loading example with code walkthrough
├── Task 2.4: Explain physics fundamentals (gravity, friction, inertia)
├── Task 2.5: Create test_gravity.py script
├── Task 2.6: Create physics validation checklist
└── Task 2.7: Add "Reality Check" callout (sim vs real physics)

Section 2.3: ROS 2 Control Integration
├── Task 3.1: Write ros2_control overview
├── Task 3.2: Create ros2_control.urdf.xacro example
├── Task 3.3: Create controllers.yaml configuration
├── Task 3.4: Write launch file (gazebo_sim.launch.py)
├── Task 3.5: Create publish_joint_commands.py script
└── Task 3.6: Add troubleshooting section

... (continues for sections 2.4-2.7)
```

---

## File Organization

```text
book/docs/module-2-digital-twin/
├── chapter-2-intro.mdx              # Write first (foundation)
├── gazebo-physics.mdx               # Write second (core technical)
├── ros2-control-integration.mdx     # Write third (builds on 2.2)
├── unity-rendering.mdx              # Write fourth (parallel to Gazebo)
├── sensor-simulation.mdx            # Write fifth (uses 2.2 + 2.3)
├── full-pipeline.mdx                # Write sixth (capstone integration)
├── exercises.mdx                    # Write last (requires all sections)
└── assets/
    ├── diagrams/ (create as SVG, export from draw.io or Figma)
    └── code-examples/ (test ALL code before adding to chapter)
```

---

## Quick Development Commands

### Test URDF

```bash
# Validate syntax
check_urdf humanoid_robot.urdf

# Visualize in RViz
ros2 launch robot_state_publisher view_robot.launch.py
```

### Test Gazebo Spawn

```bash
# Terminal 1
gazebo --verbose

# Terminal 2
ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity robot
```

### Test ros2_control

```bash
# Launch simulation with control
ros2 launch digital_twin_chapter2 gazebo_sim.launch.py

# Load controllers
ros2 control load_controller joint_state_broadcaster
ros2 control load_controller position_controller
ros2 control set_controller_state joint_state_broadcaster start
ros2 control set_controller_state position_controller start

# Test command
ros2 topic pub /position_controller/commands std_msgs/Float64MultiArray "data: [0.5, 0.0, -1.0]"
```

### Test Unity Bridge

```bash
# Terminal 1: Start ROS TCP Endpoint
ros2 launch digital_twin_chapter2 unity_bridge.launch.py

# Terminal 2: Start Gazebo
ros2 launch digital_twin_chapter2 gazebo_sim.launch.py

# Unity: Press Play (robot should sync)
```

---

## Validation Checklist

Before marking a section complete:

- [ ] All code examples run without errors on Ubuntu 22.04 + ROS 2 Humble
- [ ] Code is heavily commented (explain WHY, not just WHAT)
- [ ] "Reality Check" callout included (sim vs hardware differences)
- [ ] Diagrams are clear and labeled (test with non-expert reviewer)
- [ ] Cross-references work (links to Chapter 1, forward to Module 3)
- [ ] Screenshots show expected output (add red arrows/annotations)
- [ ] Troubleshooting section covers ≥3 common errors
- [ ] Docusaurus sidebar shows correct section numbering

---

## Common Pitfalls to Avoid

❌ **Don't**: Assume readers remember Chapter 1 details
✅ **Do**: Briefly recap URDF structure at start of Section 2.2

❌ **Don't**: Use uncommented code snippets
✅ **Do**: Add inline comments for every parameter/function

❌ **Don't**: Show only final working code
✅ **Do**: Show incremental progression (empty world → obstacles → sensors)

❌ **Don't**: Ignore performance constraints
✅ **Do**: Mention "Expect 0.5x-1.0x real-time factor on mid-range GPU"

❌ **Don't**: Skip validation steps
✅ **Do**: End each section with "How to verify this works"

❌ **Don't**: Write for experts
✅ **Do**: Assume reader is "Digital Native Convert" persona (strong Python, weak physics)

---

## Timeline Estimate

*Note: These are complexity estimates, not calendar time.*

| Section | Complexity | Prerequisites |
|---------|------------|---------------|
| 2.1 Concepts | Low | None |
| 2.2 Gazebo | Medium | 2.1 |
| 2.3 ros2_control | Medium | 2.2 |
| 2.4 Unity | High | 2.2 (URDF) |
| 2.5 Sensors | Medium | 2.2, 2.3 |
| 2.6 Full Pipeline | High | 2.2-2.5 |
| 2.7 Exercises | Low | All sections |

**Recommended Order**: 2.1 → 2.2 → 2.3 → 2.5 (sensors) → 2.4 (Unity) → 2.6 (integration) → 2.7 (exercises)

*Rationale*: Unity (2.4) is independent of sensors, so can be written in parallel after 2.3.

---

## Resources

### Documentation Links

- ROS 2 Humble: https://docs.ros.org/en/humble/
- Gazebo Classic: http://gazebosim.org/tutorials
- ros2_control: https://control.ros.org/humble/
- Unity Robotics Hub: https://github.com/Unity-Technologies/Unity-Robotics-Hub

### Example Repositories

- Universal Robots ROS 2: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver
- Clearpath Robotics: https://github.com/clearpathrobotics
- Unity URDF Importer Examples: https://github.com/Unity-Technologies/URDF-Importer/tree/main/Runtime/UrdfComponents

### Tools

- URDF Validation: `check_urdf` (ros2 package: `liburdfdom-tools`)
- TF Visualization: `ros2 run tf2_tools view_frames`
- Topic Monitor: `ros2 topic hz /joint_states`
- Bag Recording: `ros2 bag record -a` (record all topics for debugging)

---

## Next Steps

1. **Run `/sp.tasks`** to generate detailed task list
2. **Create branch protection**: Ensure code examples are tested before merge
3. **Set up Docusaurus dev environment**: `cd book && npm install && npm start`
4. **Create diagram templates**: Set up draw.io or Figma files for consistency
5. **Schedule peer review**: Have another engineer test all code examples

---

## Questions or Blockers?

If you encounter issues during implementation:

1. **Technical blockers**: Check `research.md` for resolved NEEDS CLARIFICATION items
2. **Architectural decisions**: Check `plan.md` Constitution Check section
3. **Contract ambiguities**: Check `contracts/` for schema definitions
4. **Entity relationships**: Check `data-model.md` for entity definitions

All design decisions have been documented in this planning phase. If new unknowns arise, update `research.md` before proceeding.