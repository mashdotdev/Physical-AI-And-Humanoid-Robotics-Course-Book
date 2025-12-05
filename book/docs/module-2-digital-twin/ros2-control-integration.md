---
title: 2.3 ROS 2 Control Integration
sidebar_position: 3
---

# 2.3 ROS 2 Control Integration

## Introduction

**ros2_control** is the framework that bridges simulation and real hardware with **identical controller interfaces**. The same position controller that commands a simulated robot in Gazebo will control the real robot's actuators—no code changes required.

This section teaches you to:
1. Configure ros2_control hardware interfaces for Gazebo
2. Define joint controllers (position, velocity, effort)
3. Send commands to simulated joints
4. Validate controller behavior with /joint_states topic

## ros2_control Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                    CONTROLLER MANAGER                         │
│  (Orchestrates all controllers and hardware interfaces)       │
└────────────────────┬─────────────────────────────────────────┘
                     │
        ┌────────────┼────────────┐
        │                         │
        ▼                         ▼
┌──────────────────┐      ┌──────────────────┐
│ JOINT STATE      │      │ POSITION         │
│ BROADCASTER      │      │ CONTROLLER       │
│ (Read only)      │      │ (Read + Write)   │
└────────┬─────────┘      └────────┬─────────┘
         │ /joint_states            │ /position_controller/commands
         │                          │
         ▼                          ▼
┌─────────────────────────────────────────────────────────────┐
│           GAZEBO HARDWARE INTERFACE                          │
│  (Reads joint states from physics, writes joint commands)    │
└─────────────────────────────────────────────────────────────┘
                     │
                     ▼
           GAZEBO PHYSICS ENGINE
```

**Key Components**:
- **Controller Manager**: Loads/unloads/starts/stops controllers
- **Hardware Interface**: Gazebo plugin that bridges ros2_control ↔ physics engine
- **Controllers**: Position, velocity, effort, trajectory (modular, swappable)

## Step 1: Add ros2_control to URDF

Create a ros2_control block that defines controlled joints:

```xml title="book/docs/module-2-digital-twin/assets/code-examples/urdf/ros2_control.urdf.xacro"
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- ros2_control Hardware Interface -->
  <ros2_control name="GazeboSystem" type="system">
    <hardware>
      <!-- Gazebo hardware interface plugin -->
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>

    <!-- Left Arm Joints -->
    <joint name="left_shoulder_pitch">
      <command_interface name="position">
        <param name="min">-3.14</param>
        <param name="max">3.14</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>

    <joint name="left_shoulder_roll">
      <command_interface name="position">
        <param name="min">-1.57</param>
        <param name="max">1.57</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>

    <joint name="left_elbow">
      <command_interface name="position">
        <param name="min">0.0</param>
        <param name="max">2.356</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>

    <!-- Right Arm Joints (mirror configuration) -->
    <joint name="right_shoulder_pitch">
      <command_interface name="position">
        <param name="min">-3.14</param>
        <param name="max">3.14</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>

    <joint name="right_shoulder_roll">
      <command_interface name="position">
        <param name="min">-1.57</param>
        <param name="max">1.57</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>

    <joint name="right_elbow">
      <command_interface name="position">
        <param name="min">0.0</param>
        <param name="max">2.356</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>

  </ros2_control>

  <!-- Gazebo Plugin: Load ros2_control controllers -->
  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>$(find digital_twin_chapter2)/config/joint_controllers.yaml</parameters>
    </plugin>
  </gazebo>

</robot>
```

## Step 2: Configure Controllers (YAML)

Define controller parameters:

```yaml title="book/docs/module-2-digital-twin/assets/code-examples/config/joint_controllers.yaml"
controller_manager:
  ros__parameters:
    update_rate: 100  # 100 Hz control loop

    # List of controllers to load
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    arm_position_controller:
      type: position_controllers/JointGroupPositionController

# Joint State Broadcaster (publishes /joint_states)
joint_state_broadcaster:
  ros__parameters:
    # No additional params needed (auto-detects all joints)

# Position Controller for Arms
arm_position_controller:
  ros__parameters:
    joints:
      - left_shoulder_pitch
      - left_shoulder_roll
      - left_elbow
      - right_shoulder_pitch
      - right_shoulder_roll
      - right_elbow

    # Interface type
    interface_name: position

    # Command topic
    command_interfaces:
      - position

    # State publishing
    state_interfaces:
      - position
      - velocity

    # State publisher settings
    state_publish_rate: 50.0  # Publish at 50 Hz
```

## Step 3: Load Controllers

Start controllers after Gazebo launches:

```bash
# List available controllers
ros2 control list_controllers

# Load joint state broadcaster
ros2 control load_controller joint_state_broadcaster
ros2 control set_controller_state joint_state_broadcaster start

# Load position controller
ros2 control load_controller arm_position_controller
ros2 control set_controller_state arm_position_controller start

# Verify
ros2 control list_controllers
# Expected:
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
# arm_position_controller[position_controllers/JointGroupPositionController] active
```

## Step 4: Send Joint Commands

Test by publishing commands:

```python title="book/docs/module-2-digital-twin/assets/code-examples/scripts/publish_joint_commands.py"
#!/usr/bin/env python3
"""
Publish Joint Commands Script

Sends position commands to ros2_control position controller.

Usage:
    python3 publish_joint_commands.py

Expected: Robot arms move to commanded positions.

Author: Physical AI Textbook - Chapter 2
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')

        # Publisher for position commands
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/arm_position_controller/commands',
            10
        )

        self.get_logger().info('Joint command publisher ready!')

    def publish_wave_motion(self):
        """Publish sinusoidal wave motion to arms."""
        self.get_logger().info('Starting wave motion...')

        rate = self.create_rate(10)  # 10 Hz
        t = 0.0

        for _ in range(100):  # Run for 10 seconds (100 steps * 0.1s)
            msg = Float64MultiArray()

            # Sinusoidal motion: arms wave up and down
            left_shoulder = 1.0 * math.sin(t)
            right_shoulder = 1.0 * math.sin(t + math.pi)  # Out of phase

            # Command: [left_shoulder_pitch, left_shoulder_roll, left_elbow,
            #           right_shoulder_pitch, right_shoulder_roll, right_elbow]
            msg.data = [
                left_shoulder,    # Left shoulder pitch
                0.0,              # Left shoulder roll (neutral)
                1.0,              # Left elbow (bent)
                right_shoulder,   # Right shoulder pitch
                0.0,              # Right shoulder roll (neutral)
                1.0,              # Right elbow (bent)
            ]

            self.cmd_pub.publish(msg)
            t += 0.1 * 2 * math.pi  # Increment time
            time.sleep(0.1)

        self.get_logger().info('Wave motion complete!')

def main(args=None):
    rclpy.init(args=args)
    node = JointCommandPublisher()
    node.publish_wave_motion()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Run**:
```bash
python3 publish_joint_commands.py
```

**Expected**: Robot arms wave up and down in opposite phase.

## Troubleshooting

### Error: "Controller failed to load"

**Cause**: Missing controller plugin or incorrect YAML path

**Solution**:
```bash
# Check installed controller plugins
ros2 pkg list | grep controllers

# Verify YAML path in URDF
# Must be absolute or use $(find package_name)
```

### Error: "Joint limits violated"

**Cause**: Command exceeds URDF `<limit>` tags

**Solution**: Clamp commands in your node:
```python
def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))

msg.data = [clamp(cmd, -3.14, 3.14) for cmd in commands]
```

## Section Summary

**Key Concepts**:
1. **ros2_control**: Hardware-agnostic controller framework
2. **Gazebo hardware interface**: Bridges ros2_control ↔ physics engine
3. **Controllers**: Position, velocity, effort (modular, swappable)
4. **/joint_states topic**: Real-time joint position/velocity/effort feedback

**Next Section**: [2.4 Unity High-Fidelity Rendering](./unity-rendering.md)
