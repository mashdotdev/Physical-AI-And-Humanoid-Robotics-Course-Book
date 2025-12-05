"""
Gazebo Simulation Launch File

Starts Gazebo and spawns the humanoid robot from URDF.

Usage:
    ros2 launch digital_twin_chapter2 gazebo_sim.launch.py

Author: Physical AI Textbook - Chapter 2
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Paths
    pkg_share = get_package_share_directory('digital_twin_chapter2')
    urdf_file = os.path.join(pkg_share, 'urdf', 'humanoid_robot.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'empty_world.world')

    # Process xacro to URDF
    robot_description = xacro.process_file(urdf_file).toxml()

    # Robot State Publisher (publishes TF tree from URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True  # CRITICAL: Use Gazebo's /clock topic
        }]
    )

    # Gazebo Server (physics engine, runs headless)
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Gazebo Client (GUI, optional for debugging)
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # Spawn Robot Entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',  # Read from robot_state_publisher
            '-entity', 'humanoid_robot',      # Name in Gazebo
            '-x', '0', '-y', '0', '-z', '1.0', # Spawn 1m above ground
        ],
        output='screen'
    )

    # Event handler: Spawn robot after Gazebo starts
    spawn_after_gazebo = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_server,
            on_exit=[spawn_entity]
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo_server,
        gazebo_client,
        spawn_entity,
    ])
