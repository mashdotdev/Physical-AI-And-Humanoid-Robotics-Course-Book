"""
Complete Digital Twin Launch File

Starts entire simulation pipeline with one command.

Usage:
    ros2 launch digital_twin_chapter2 digital_twin_complete.launch.py

Launches:
    - Gazebo physics simulation with robot
    - ROS 2 control managers and controllers
    - ROS TCP Endpoint (for Unity bridge)
    - Sensor publishers

Author: Physical AI Textbook - Chapter 2
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Package directories
    pkg_share = FindPackageShare('digital_twin_chapter2')

    return LaunchDescription([
        # ===== 1. Start Gazebo with Robot =====
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    pkg_share,
                    'launch',
                    'gazebo_sim.launch.py'
                ])
            ])
        ),

        # ===== 2. Start ROS TCP Endpoint (for Unity bridge) =====
        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name='ros_tcp_endpoint',
            parameters=[{
                'ROS_IP': '0.0.0.0',
                'ROS_TCP_PORT': 10000
            }],
            output='screen'
        ),

        # ===== 3. Load ros2_control Controllers =====
        ExecuteProcess(
            cmd=[
                'ros2', 'control', 'load_controller',
                '--set-state', 'active',
                'joint_state_broadcaster'
            ],
            output='screen'
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'control', 'load_controller',
                '--set-state', 'active',
                'arm_position_controller'
            ],
            output='screen'
        ),

        # ===== 4. Launch RViz (optional, for visualization) =====
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                pkg_share,
                'config',
                'digital_twin.rviz'
            ])],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
    ])
