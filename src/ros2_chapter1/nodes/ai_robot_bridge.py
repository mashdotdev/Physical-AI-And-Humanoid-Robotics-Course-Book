#!/usr/bin/env python3
"""
AI-Robot Bridge Node

This node demonstrates how to integrate high-level AI decision-making systems
(like LLMs or vision-language-action models) with low-level robot controllers
via ROS 2.

Architecture:
    AI Agent (LLM) --> Bridge Node --> Robot Controller --> Hardware

The bridge translates high-level commands (e.g., "pick up the red cup") into
low-level control signals (joint velocities, positions).

Usage:
    ros2 run ros2_chapter1 ai_robot_bridge

Test with:
    ros2 topic pub /ai/command std_msgs/msg/String "{data: 'move_forward'}"
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import re


class AIRobotBridge(Node):
    """
    Bridge between high-level AI commands and low-level robot control.

    This node:
    1. Subscribes to AI commands (natural language or structured)
    2. Parses and validates commands
    3. Translates to robot control messages
    4. Publishes to robot controller topics
    5. Provides feedback and error handling
    """

    def __init__(self):
        super().__init__('ai_robot_bridge')

        # Subscribe to AI commands
        self.ai_subscription = self.create_subscription(
            String,
            '/ai/command',
            self.ai_command_callback,
            10
        )

        # Publish to robot controller
        self.robot_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Publish status feedback
        self.status_publisher = self.create_publisher(
            String,
            '/ai/status',
            10
        )

        # Safety limits (m/s and rad/s)
        self.max_linear_velocity = 0.5
        self.max_angular_velocity = 1.0

        # Command mapping (high-level command -> velocity)
        self.command_map = {
            'move_forward': (0.3, 0.0),
            'move_backward': (-0.3, 0.0),
            'turn_left': (0.0, 0.5),
            'turn_right': (0.0, -0.5),
            'stop': (0.0, 0.0),
        }

        self.get_logger().info('AI-Robot Bridge started. Listening to /ai/command')

    def ai_command_callback(self, msg):
        """
        Process AI commands and translate to robot controls.

        Args:
            msg (String): High-level command from AI system
        """
        command = msg.data.strip().lower()
        self.get_logger().info(f'Received AI command: "{command}"')

        # Parse and validate command
        try:
            cmd_vel = self.parse_command(command)

            # Safety check
            if self.is_safe(cmd_vel):
                self.execute_command(cmd_vel)
                self.publish_status(f'EXECUTING: {command}')
            else:
                self.get_logger().warn(f'Command rejected (safety limits): {command}')
                self.publish_status(f'REJECTED: {command} - exceeds safety limits')

        except ValueError as e:
            self.get_logger().error(f'Invalid command: {command} - {e}')
            self.publish_status(f'ERROR: {command} - {e}')

    def parse_command(self, command):
        """
        Parse high-level command into velocity control.

        Supports:
        - Predefined commands: 'move_forward', 'turn_left', etc.
        - Parameterized commands: 'move forward 0.2', 'turn left 0.5'

        Args:
            command (str): High-level command string

        Returns:
            Twist: Velocity command message

        Raises:
            ValueError: If command is invalid or unsafe
        """
        cmd_vel = Twist()

        # Try predefined commands first
        if command in self.command_map:
            linear, angular = self.command_map[command]
            cmd_vel.linear.x = linear
            cmd_vel.angular.z = angular
            return cmd_vel

        # Parse parameterized commands (e.g., "move forward 0.3")
        patterns = [
            (r'move forward ([\d.]+)', lambda m: (float(m.group(1)), 0.0)),
            (r'move backward ([\d.]+)', lambda m: (-float(m.group(1)), 0.0)),
            (r'turn left ([\d.]+)', lambda m: (0.0, float(m.group(1)))),
            (r'turn right ([\d.]+)', lambda m: (0.0, -float(m.group(1)))),
        ]

        for pattern, handler in patterns:
            match = re.match(pattern, command)
            if match:
                linear, angular = handler(match)
                cmd_vel.linear.x = linear
                cmd_vel.angular.z = angular
                return cmd_vel

        raise ValueError(f'Unrecognized command: {command}')

    def is_safe(self, cmd_vel):
        """
        Validate command against safety limits.

        Args:
            cmd_vel (Twist): Proposed velocity command

        Returns:
            bool: True if command is safe, False otherwise
        """
        linear_safe = abs(cmd_vel.linear.x) <= self.max_linear_velocity
        angular_safe = abs(cmd_vel.angular.z) <= self.max_angular_velocity

        return linear_safe and angular_safe

    def execute_command(self, cmd_vel):
        """
        Execute validated command by publishing to robot controller.

        Args:
            cmd_vel (Twist): Velocity command to execute
        """
        self.robot_publisher.publish(cmd_vel)
        self.get_logger().info(
            f'Executing: linear={cmd_vel.linear.x:.2f}, angular={cmd_vel.angular.z:.2f}'
        )

    def publish_status(self, status):
        """
        Publish status feedback to AI system.

        Args:
            status (str): Status message
        """
        msg = String()
        msg.data = status
        self.status_publisher.publish(msg)


def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)
    node = AIRobotBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Emergency stop on shutdown
        stop_cmd = Twist()
        node.robot_publisher.publish(stop_cmd)
        node.get_logger().info('Emergency stop issued on shutdown')

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
