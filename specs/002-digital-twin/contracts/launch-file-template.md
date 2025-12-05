# Contract: Launch File Template Structure

**Purpose**: Define standard launch file structure for digital twin pipeline.

**Version**: 1.0
**Date**: 2025-12-05

---

## Template Structure

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world', default='empty_world.world')

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='empty_world.world'),

        # Gazebo
        Node(
            package='gazebo_ros',
            executable='gzserver',
            arguments=[world_file],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': '$(cat path/to/robot.urdf)'}]
        ),

        # Spawn Entity
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'robot']
        ),

        # ROS TCP Endpoint (Unity Bridge)
        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            parameters=[{'ROS_IP': '0.0.0.0', 'ROS_TCP_PORT': 10000}]
        ),
    ])
```

---

## Contract Requirements

✅ ALL nodes MUST set `use_sim_time` parameter
✅ Launch files MUST be testable standalone (not just as includes)
✅ File paths MUST use `FindPackageShare` (not hardcoded)
✅ Default arguments provided for all required parameters

---

## Usage

```bash
ros2 launch digital_twin_chapter2 digital_twin_complete.launch.py
ros2 launch digital_twin_chapter2 digital_twin_complete.launch.py world:=test_environment.world
```