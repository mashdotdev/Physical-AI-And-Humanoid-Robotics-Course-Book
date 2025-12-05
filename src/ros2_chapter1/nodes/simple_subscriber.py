#!/usr/bin/env python3
"""
Simple ROS 2 Subscriber Node

This node demonstrates the basics of subscribing to messages from a ROS 2 topic.
It listens to String messages published on the '/robot/message' topic.

Usage:
    ros2 run ros2_chapter1 simple_subscriber

Test with:
    ros2 run ros2_chapter1 simple_publisher
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSubscriber(Node):
    """
    A simple ROS 2 node that subscribes to String messages.

    This demonstrates the basic subscriber pattern:
    1. Create a subscription object attached to a topic
    2. Define a callback function to process incoming messages
    3. Process messages as they arrive
    """

    def __init__(self):
        super().__init__('simple_subscriber')

        # Create a subscription to the '/robot/message' topic
        # Queue size of 10 means we buffer up to 10 unprocessed messages
        self.subscription = self.create_subscription(
            String,              # Message type
            '/robot/message',    # Topic name
            self.listener_callback,  # Callback function
            10                   # Queue size
        )

        # Prevent unused variable warning
        self.subscription

        # Counter to track received messages
        self.count = 0

        self.get_logger().info('Simple Subscriber node started. Listening to /robot/message')

    def listener_callback(self, msg):
        """
        Callback function triggered when a message is received.

        Args:
            msg (std_msgs.msg.String): The received message
        """
        self.count += 1
        self.get_logger().info(f'Received message #{self.count}: "{msg.data}"')


def main(args=None):
    """Main entry point for the node."""
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the SimpleSubscriber node
    node = SimpleSubscriber()

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
