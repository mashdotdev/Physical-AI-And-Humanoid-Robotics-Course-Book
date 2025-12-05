---
title: 2.6 Full Digital Twin Pipeline Integration
sidebar_position: 6
---

# 2.6 Full Digital Twin Pipeline Integration

## Introduction

This section brings everything together: **Gazebo physics + ROS 2 control + Unity rendering + sensors** orchestrated with a single launch command.

## Complete Launch File

```python title="book/docs/module-2-digital-twin/assets/code-examples/launch/digital_twin_complete.launch.py"
"""
Complete Digital Twin Launch File

Starts entire simulation pipeline with one command.

Usage:
    ros2 launch digital_twin_chapter2 digital_twin_complete.launch.py

Author: Physical AI Textbook - Chapter 2
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        # 1. Start Gazebo with robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('digital_twin_chapter2'),
                    'launch',
                    'gazebo_sim.launch.py'
                ])
            ])
        ),

        # 2. Start ROS TCP Endpoint (for Unity bridge)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros_tcp_endpoint'),
                    'launch',
                    'endpoint.launch.py'
                ])
            ]),
            launch_arguments={'ROS_IP': '0.0.0.0'}.items()
        ),
    ])
```

**Run Complete Pipeline**:
```bash
ros2 launch digital_twin_chapter2 digital_twin_complete.launch.py
```

Then press Play in Unity → Everything syncs!

## Best Practices Validation

- ✅ Real inertia values (from CAD or analytical formulas)
- ✅ Collision meshes &lt;1000 triangles
- ✅ Sensor noise matches hardware specs
- ✅ TF tree synchronized (check `ros2 run tf2_tools view_frames`)
- ✅ Real-time factor ≥ 0.8

## Section Summary

**Complete Pipeline**:
1. Gazebo handles physics (authoritative)
2. ros2_control manages joint commands
3. Unity mirrors visual state
4. Sensors publish realistic data
5. Single launch file orchestrates everything

**Next Section**: [2.7 Exercises](./exercises.md)
