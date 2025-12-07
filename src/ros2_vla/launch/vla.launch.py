from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_vla',
            executable='audio_capture_node',
            name='audio_capture_node',
            output='screen',
        ),
        Node(
            package='ros2_vla',
            executable='whisper_node',
            name='whisper_node',
            output='screen',
        ),
        Node(
            package='ros2_vla',
            executable='llm_node',
            name='llm_node',
            output='screen',
        ),
        Node(
            package='ros2_vla',
            executable='task_dispatcher_node',
            name='task_dispatcher_node',
            output='screen',
            parameters=[{'robot_command': 'none'}] # Initialize with no command
        ),
    ])
