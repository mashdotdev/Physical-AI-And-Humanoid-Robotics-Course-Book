#!/usr/bin/env python3
"""
Isaac ROS Visual SLAM Launch File
Contract Template for Section 3.3

Purpose: Launch Isaac ROS VSLAM with Isaac Sim camera topics
Requirements: FR-015 (VSLAM launch file with topic configuration)
Success Criteria: SC-004 (30Hz pose updates, RViz visualization)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
import os


def generate_launch_description():
    """
    Generate launch description for Isaac ROS Visual SLAM.

    Preconditions:
        - Isaac Sim running and publishing camera topics
        - ROS 2 Humble or Iron installed
        - isaac_ros_visual_slam package available

    Postconditions:
        - VSLAM node launched and configured
        - Topics remapped to Isaac Sim conventions
        - VSLAM publishes /vslam/pose at 30Hz
        - Map→odom transform published

    Performance:
        - Pose estimation latency < 33ms (30Hz)
        - GPU memory usage < 2GB
    """

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time from Isaac Sim",
    )

    config_file_arg = DeclareLaunchArgument(
        "vslam_params_file",
        default_value=os.path.join(
            # Path relative to examples/module-3-ai-robot-brain/isaac-ros-vslam/
            os.path.dirname(__file__),
            "..",
            "config",
            "vslam_params.yaml",
        ),
        description="Path to VSLAM parameters YAML file",
    )

    enable_imu_arg = DeclareLaunchArgument(
        "enable_imu_fusion",
        default_value="true",
        description="Enable IMU fusion for better tracking",
    )

    # Visual SLAM node (Lifecycle node for managed startup)
    visual_slam_node = LifecycleNode(
        package="isaac_ros_visual_slam",
        executable="isaac_ros_visual_slam",
        name="visual_slam_node",
        namespace="",
        output="screen",
        parameters=[
            LaunchConfiguration("vslam_params_file"),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "enable_imu_fusion": LaunchConfiguration("enable_imu_fusion"),
                "enable_loop_closure": True,
                "enable_localization_n_mapping": True,
                "rectified_images": True,  # Isaac Sim provides rectified images
                "enable_slam_visualization": True,
                "enable_landmarks_view": True,
                "enable_observations_view": True,
                "min_num_features": 150,
                "max_num_features": 500,
                # Frame IDs (must match URDF/TF tree)
                "map_frame": "map",
                "odom_frame": "odom",
                "base_frame": "base_link",
                "camera_frame": "camera_link",
                # Performance tuning
                "num_cameras": 2,  # Stereo configuration
                "use_gpu": True,  # NVIDIA GPU acceleration
            },
        ],
        remappings=[
            # Isaac Sim topics → VSLAM expected topics
            ("/stereo_camera/left/image", "/camera/left/image_raw"),
            ("/stereo_camera/left/camera_info", "/camera/left/camera_info"),
            ("/stereo_camera/right/image", "/camera/right/image_raw"),
            ("/stereo_camera/right/camera_info", "/camera/right/camera_info"),
            ("/visual_slam/imu", "/camera/imu"),
            # VSLAM outputs
            ("/visual_slam/tracking/odometry", "/vslam/pose"),
            ("/visual_slam/tracking/slam_path", "/vslam/path"),
            ("/visual_slam/status", "/vslam/status"),
            ("/visual_slam/vis/landmarks_cloud", "/vslam/map"),
        ],
    )

    # Static transform publishers (camera→base_link)
    # NOTE: These would typically come from URDF, but explicitly provided for clarity
    camera_left_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_left_tf_publisher",
        output="screen",
        arguments=[
            "0.0",
            "0.1",
            "0.0",  # x, y, z (camera offset from base)
            "0.0",
            "0.0",
            "0.0",
            "1.0",  # qx, qy, qz, qw (no rotation)
            "base_link",
            "camera_left_link",
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    camera_right_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_right_tf_publisher",
        output="screen",
        arguments=[
            "0.0",
            "-0.1",
            "0.0",  # Stereo baseline: 0.2m
            "0.0",
            "0.0",
            "0.0",
            "1.0",
            "base_link",
            "camera_right_link",
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_tf_publisher",
        output="screen",
        arguments=[
            "0.0",
            "0.0",
            "0.05",  # IMU slightly above cameras
            "0.0",
            "0.0",
            "0.0",
            "1.0",
            "base_link",
            "imu_link",
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    # Monitor node (checks VSLAM status and reports)
    monitor_node = Node(
        package="isaac_ros_visual_slam",
        executable="vslam_status_monitor",  # Custom monitoring node
        name="vslam_monitor",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription(
        [
            # Arguments
            use_sim_time_arg,
            config_file_arg,
            enable_imu_arg,
            # Nodes
            visual_slam_node,
            camera_left_tf,
            camera_right_tf,
            imu_tf,
            monitor_node,
        ]
    )


"""
Launch File Contract Validation:

Input Topics (expected from Isaac Sim):
  - /camera/left/image_raw (sensor_msgs/Image, 640x480, 30Hz)
  - /camera/right/image_raw (sensor_msgs/Image, 640x480, 30Hz)
  - /camera/imu (sensor_msgs/Imu, 200Hz)
  - /camera/left/camera_info (sensor_msgs/CameraInfo)
  - /camera/right/camera_info (sensor_msgs/CameraInfo)

Output Topics (provided by VSLAM):
  - /vslam/pose (geometry_msgs/PoseStamped, 30Hz) - Robot pose in map frame
  - /vslam/path (nav_msgs/Path) - Full trajectory history
  - /vslam/status (isaac_ros_visual_slam_interfaces/VisualSlamStatus, 10Hz)
  - /vslam/map (sensor_msgs/PointCloud2, 1Hz) - 3D landmark cloud

TF Transforms (published by VSLAM):
  - map → odom (dynamic, from VSLAM tracking)

TF Transforms (required as input):
  - odom → base_link (from robot odometry)
  - base_link → camera_left/right/imu_link (static, from this launch file)

Lifecycle States:
  1. Unconfigured → Configured (on launch)
  2. Configured → Activated (when camera topics available)
  3. Activated → Tracking (when first keyframe initialized)

Failure Modes:
  - Tracking lost: /vslam/status = TRACKING_LOST (triggers recovery in Nav2)
  - No camera input: Node stays in Configured state
  - IMU fusion failure: Degrades to vision-only tracking (warning logged)

Performance Metrics:
  - Pose estimation rate: 30Hz (measured via ros2 topic hz /vslam/pose)
  - Pose latency: <33ms (measured via timestamp difference)
  - Feature tracking: ≥150 features per frame (logged in /vslam/status)
  - Loop closure frequency: ~1 per 100 keyframes (depends on trajectory)

Usage:
  # Launch with defaults
  ros2 launch vslam_isaac_sim.launch.py

  # Launch with custom config
  ros2 launch vslam_isaac_sim.launch.py vslam_params_file:=/path/to/custom.yaml

  # Disable IMU fusion (vision-only)
  ros2 launch vslam_isaac_sim.launch.py enable_imu_fusion:=false

Validation Commands:
  # Check topics are publishing
  ros2 topic list | grep vslam

  # Monitor pose output rate
  ros2 topic hz /vslam/pose

  # Check VSLAM status
  ros2 topic echo /vslam/status

  # Verify TF tree
  ros2 run tf2_tools view_frames
"""
