---
sidebar_position: 4
title: 1.4 Robot Description Format (URDF)
---

# 1.4 Visualizing Robot Structure with URDF

Before you can control a humanoid robot, you need to **define its physical structure**: how many joints it has, where they're located, how they move, and what the robot looks like. This is where **URDF (Unified Robot Description Format)** comes in.

## What is URDF?

URDF is an **XML-based format** for describing the kinematic and dynamic properties of a robot. Think of it as the "blueprint" or "skeleton" of your robot.

### The Biological Analogy (Revisited)

In our nervous system analogy:
- **Nodes** = neurons (processors)
- **Topics** = nerves (communication channels)
- **URDF** = **the skeleton** (physical structure)

Just as your nervous system needs to know where your bones and joints are located to coordinate movement, ROS 2 needs URDF to understand your robot's physical structure.

## Core URDF Concepts

### 1. Links (Rigid Bodies)

A **link** represents a rigid body in the robot—a part that doesn't deform. Examples:
- The robot's torso
- An upper arm segment
- A thigh bone
- A gripper finger

**Link properties:**
- **Visual**: How it looks (geometry, color) for visualization
- **Collision**: Simplified geometry for collision detection
- **Inertial**: Mass and inertia properties for physics simulation

### 2. Joints (Connections)

A **joint** connects two links and defines how they move relative to each other.

**Joint types:**
- **Fixed**: No movement (e.g., camera mounted on head)
- **Revolute**: Rotational movement with limits (e.g., elbow, knee)
- **Continuous**: Unlimited rotation (e.g., a wheel)
- **Prismatic**: Linear sliding movement (e.g., elevator platform)
- **Floating**: 6 degrees of freedom (e.g., free-flying drone)
- **Planar**: Movement in a 2D plane

**Joint properties:**
- **Parent link**: The link this joint is attached to
- **Child link**: The link that moves when the joint actuates
- **Axis**: Direction of rotation or translation
- **Limits**: Min/max position, max velocity, max torque
- **Dynamics**: Damping and friction

### 3. Visual and Collision Geometry

**Visual geometry**: What you see in the visualizer (RViz, Isaac Sim)
- Can be complex meshes (`.stl`, `.dae`, `.obj` files)
- Prioritizes appearance

**Collision geometry**: Simplified shapes for physics calculations
- Typically boxes, cylinders, spheres
- Prioritizes computational efficiency

## Example: Simple 2-Joint Robot Arm

Let's build a minimal robot arm to understand URDF structure:

```text
        ┌─────────────┐
        │ End Effector│ (sphere, yellow)
        └──────┬──────┘
               │ fixed joint
        ┌──────┴──────┐
        │ Elbow Link  │ (cylinder, red)
        └──────┬──────┘
               │ elbow_joint (revolute)
        ┌──────┴──────┐
        │Shoulder Link│ (cylinder, green)
        └──────┬──────┘
               │ shoulder_joint (revolute)
        ┌──────┴──────┐
        │  Base Link  │ (box, blue)
        └─────────────┘
```

### URDF Implementation

```xml title="src/ros2_chapter1/urdf/simple_2_joint_robot.urdf"
<?xml version="1.0"?>
<robot name="simple_2_joint_robot">

  <!-- Base Link (Fixed to World) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1.0"/>
      </material>
    </visual>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0"
               iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Shoulder Joint (Revolute - Pitch Motion) -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>  <!-- Rotates around Y-axis -->
    <limit effort="10.0" lower="-1.57" upper="1.57" velocity="1.0"/>
  </joint>

  <!-- Shoulder Link (Upper Arm) -->
  <link name="shoulder_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <material name="green">
        <color rgba="0.2 0.8 0.2 1.0"/>
      </material>
    </visual>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0"
               iyy="0.005" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Elbow Joint (Revolute - Pitch Motion) -->
  <joint name="elbow_joint" type="revolute">
    <parent link="shoulder_link"/>
    <child link="elbow_link"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="5.0" lower="-2.0" upper="0.0" velocity="1.0"/>
  </joint>

  <!-- Elbow Link (Forearm) -->
  <link name="elbow_link">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <material name="red">
        <color rgba="0.8 0.2 0.2 1.0"/>
      </material>
    </visual>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0"
               iyy="0.002" iyz="0.0" izz="0.0005"/>
    </inertial>
  </link>

</robot>
```

### Understanding the URDF Elements

**`<origin>` Tag:**
- `xyz`: Position offset (meters) in X, Y, Z
- `rpy`: Orientation (Roll, Pitch, Yaw) in radians

**`<axis>` Tag:**
- Defines the rotation or translation axis
- `xyz="0 1 0"` = rotates around Y-axis (pitch)
- `xyz="1 0 0"` = rotates around X-axis (roll)
- `xyz="0 0 1"` = rotates around Z-axis (yaw)

**`<limit>` Tag (for revolute/prismatic joints):**
- `lower`, `upper`: Joint position limits (radians for revolute, meters for prismatic)
- `effort`: Maximum torque (N·m) or force (N)
- `velocity`: Maximum joint velocity (rad/s or m/s)

**`<inertial>` Tag:**
- `mass`: Link mass in kg
- `inertia`: 3x3 inertia tensor (required for physics simulation)

## Visualizing Your Robot

### Launch File Setup

To visualize the URDF, we need three ROS 2 nodes:

1. **robot_state_publisher**: Publishes TF transforms based on URDF
2. **joint_state_publisher**: Publishes joint positions (allows interactive control with GUI)
3. **rviz2**: 3D visualization tool

```python title="src/ros2_chapter1/launch/simple_robot_launch.py"
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Load URDF file
    urdf_file = os.path.join(
        os.path.dirname(__file__), '..', 'urdf', 'simple_2_joint_robot.urdf'
    )
    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    return LaunchDescription([
        # Publish TF transforms from URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),

        # Interactive joint control GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),

        # 3D visualization
        Node(
            package='rviz2',
            executable='rviz2',
        ),
    ])
```

### Running the Visualization

**Step 1: Install required packages**
```bash
sudo apt install ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui
```

**Step 2: Launch the visualization**
```bash
chmod +x src/ros2_chapter1/launch/simple_robot_launch.py
ros2 launch src/ros2_chapter1/launch/simple_robot_launch.py
```

**Step 3: Configure RViz**
1. In RViz, click **Add** → **RobotModel**
2. Set **Fixed Frame** to `base_link`
3. You should see your robot arm appear!
4. Use the **Joint State Publisher GUI** sliders to move the joints

### Expected Visualization

You should see:
- **Blue box** (base)
- **Green cylinder** (shoulder link) rotating with shoulder joint
- **Red cylinder** (elbow link) rotating with elbow joint
- **Yellow sphere** (end effector)

Drag the sliders in the Joint State Publisher GUI to see the arm move in real-time!

## Real-World Humanoid URDF

Production humanoid robots have **much more complex** URDF files:

### Example: Humanoid Torso Structure

```text
Typical humanoid has 30-50+ joints:
├─ torso (6 DOF floating base)
├─ head (2 DOF: pan, tilt)
├─ left_arm (7 DOF: shoulder x3, elbow, wrist x3)
├─ right_arm (7 DOF)
├─ left_leg (6 DOF: hip x3, knee, ankle x2)
└─ right_leg (6 DOF)

Total: ~40 degrees of freedom
```

### URDF Best Practices

**1. Use descriptive link/joint names:**
```xml
<!-- Good -->
<link name="left_shoulder_roll_link"/>
<joint name="left_shoulder_roll_joint" type="revolute"/>

<!-- Bad -->
<link name="link1"/>
<joint name="j1" type="revolute"/>
```

**2. Set realistic joint limits:**
```xml
<!-- Human-like shoulder limits -->
<limit lower="-3.14" upper="3.14" effort="50" velocity="2.0"/>
```

**3. Use mesh files for complex geometry:**
```xml
<visual>
  <geometry>
    <mesh filename="package://my_robot/meshes/torso.stl" scale="0.001 0.001 0.001"/>
  </geometry>
</visual>
```

**4. Separate visual and collision geometry:**
```xml
<visual>
  <geometry>
    <mesh filename="package://my_robot/meshes/hand_detailed.stl"/>
  </geometry>
</visual>
<collision>
  <geometry>
    <box size="0.1 0.05 0.15"/>  <!-- Simplified for collision -->
  </geometry>
</collision>
```

## URDF Tools and Utilities

### Validating URDF

Check your URDF for errors:
```bash
check_urdf simple_2_joint_robot.urdf
```

### Visualizing URDF Structure

Generate a PDF graph of links and joints:
```bash
urdf_to_graphiz simple_2_joint_robot.urdf
```

### Converting URDF to Other Formats

```bash
# URDF to SDF (Gazebo Simulation)
gz sdf -p simple_2_joint_robot.urdf > robot.sdf

# URDF to MJCF (MuJoCo Physics Engine)
# Requires mujoco_urdf_converter
```

## Reality Check: URDF Limitations

While URDF is the standard for ROS, it has limitations:

| Limitation | Impact | Solution |
|------------|--------|----------|
| **No closed kinematic chains** | Can't model parallel robots (e.g., Stewart platform) | Use SDF or custom plugins |
| **Static parameter values** | Can't model adaptive structures | Use Xacro for parameterization |
| **Limited contact modeling** | Physics simulation inaccuracies | Use Gazebo/Isaac Sim for detailed contact |
| **XML verbosity** | Large files for complex robots | Use Xacro macros |

### Xacro: Programmable URDF

**Xacro** (XML Macros) extends URDF with:
- Variables and expressions
- Macros for reusable components
- Math operations

**Example:**
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">
  <xacro:property name="link_length" value="0.3"/>
  <xacro:property name="link_radius" value="0.05"/>

  <xacro:macro name="arm_link" params="name length">
    <link name="${name}">
      <visual>
        <geometry>
          <cylinder radius="${link_radius}" length="${length}"/>
        </geometry>
      </visual>
    </link>
  </xacro:macro>

  <!-- Use macro -->
  <xacro:arm_link name="upper_arm" length="${link_length}"/>
  <xacro:arm_link name="forearm" length="${link_length * 0.8}"/>
</robot>
```

## Summary

In this section, we learned:
- ✅ URDF is the standard format for describing robot physical structure in ROS 2
- ✅ **Links** represent rigid bodies, **joints** define how they move
- ✅ URDF includes visual, collision, and inertial properties for simulation
- ✅ `robot_state_publisher` and `joint_state_publisher` enable visualization
- ✅ RViz provides interactive 3D visualization of robots
- ✅ Production humanoid robots have 30-50+ joints defined in URDF

**Key Takeaways:**
- URDF is the "skeleton" of your robot in ROS 2
- Accurate URDF is critical for motion planning, collision avoidance, and physics simulation
- Use Xacro for complex robots to reduce repetition
- Always validate URDF before deploying to hardware

Next, we'll put everything together with hands-on exercises!

## Further Reading

- [URDF Official Specification](http://wiki.ros.org/urdf/XML)
- [Xacro Documentation](http://wiki.ros.org/xacro)
- [ROS 2 URDF Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- [Isaac Sim URDF Import](https://docs.omniverse.nvidia.com/isaacsim/latest/features/environment_setup/ext_omni_isaac_urdf.html)
