#!/usr/bin/env python3
"""
Simple ROS 2 Publisher Node

This node demonstrates the basics of publishing messages to a ROS 2 topic.
It publishes a String message at 1 Hz (once per second).

Usage:
    ros2 run ros2_chapter1 simple_publisher

Monitor output:
    ros2 topic echo /robot/message
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):
    """
    A simple ROS 2 node that publishes String messages periodically.

    This demonstrates the basic publisher pattern:
    1. Create a publisher object attached to a topic
    2. Use a timer to trigger periodic publishing
    3. Publish messages to the topic
    """

    def __init__(self):
        super().__init__('simple_publisher')

        # Create a publisher that publishes String messages to '/robot/message'
        # Queue size of 10 means we buffer up to 10 messages if subscribers are slow
        self.publisher_ = self.create_publisher(
            String,              # Message type
            '/robot/message',    # Topic name
            10                   # Queue size
        )

        # Create a timer that calls timer_callback() every 1.0 second
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter to track message number
        self.count = 0

        self.get_logger().info('Simple Publisher node started. Publishing to /robot/message')

    def timer_callback(self):
        """
        Callback function triggered by the timer.

        This function is called every timer_period seconds and publishes
        a message to the topic.
        """
        # Create a new String message
        msg = String()
        msg.data = f'Hello from ROS 2! Message count: {self.count}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log the published message
        self.get_logger().info(f'Publishing: "{msg.data}"')

        self.count += 1


def main(args=None):
    """Main entry point for the node."""
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the SimplePublisher node
    node = SimplePublisher()

    try:
        # Spin the node (process callbacks) until shutdown
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and shutdown
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
