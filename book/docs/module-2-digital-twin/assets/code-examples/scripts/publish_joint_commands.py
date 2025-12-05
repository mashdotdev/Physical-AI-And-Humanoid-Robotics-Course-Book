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
