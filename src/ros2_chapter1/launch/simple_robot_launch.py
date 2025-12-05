#!/usr/bin/env python3
"""
Launch file for visualizing the simple 2-joint robot in RViz.

This launch file:
1. Loads the URDF robot description
2. Publishes robot state (joint positions)
3. Launches RViz with a preconfigured view

Usage:
    ros2 launch ros2_chapter1 simple_robot_launch.py

Note: Requires robot_state_publisher and joint_state_publisher packages.
Install with:
    sudo apt install ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for robot visualization."""

    # Get the URDF file path
    # Note: In a real package, this would use get_package_share_directory
    # For now, we'll use an absolute path relative to the workspace
    urdf_file = os.path.join(
        os.path.dirname(__file__),
        '..',
        'urdf',
        'simple_2_joint_robot.urdf'
    )

    # Read the URDF file
    try:
        with open(urdf_file, 'r') as file:
            robot_description = file.read()
    except FileNotFoundError:
        print(f"ERROR: URDF file not found at {urdf_file}")
        robot_description = ""

    # Declare launch arguments
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Use joint_state_publisher GUI to control joints'
    )

    # Robot State Publisher - publishes TF transforms based on URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Joint State Publisher - publishes joint positions
    # Use GUI version for interactive control
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=LaunchConfiguration('use_gui'),
        output='screen'
    )

    # Alternative: Non-GUI joint state publisher (uncomment if GUI not available)
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     output='screen'
    # )

    # RViz - 3D visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(os.path.dirname(__file__), 'robot_view.rviz')]
        if os.path.exists(os.path.join(os.path.dirname(__file__), 'robot_view.rviz'))
        else []
    )

    return LaunchDescription([
        use_gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ])
