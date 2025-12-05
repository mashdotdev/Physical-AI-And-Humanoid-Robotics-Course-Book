#!/usr/bin/env python3
"""
ROS 2 Service Client and Server Node

This node demonstrates request-response communication using ROS 2 services.
It implements both a service server (to handle requests) and a service client
(to send requests).

The example service adds two integers together (AddTwoInts).

Usage:
    # Run the server:
    ros2 run ros2_chapter1 service_client_server --ros-args -p mode:=server

    # Run the client (in another terminal):
    ros2 run ros2_chapter1 service_client_server --ros-args -p mode:=client -p a:=5 -p b:=7

Test server manually:
    ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 10, b: 15}"
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServer(Node):
    """
    Service server that provides addition of two integers.

    This demonstrates the basic service server pattern:
    1. Create a service server object
    2. Define a callback function to handle requests
    3. Return responses with computation results
    """

    def __init__(self):
        super().__init__('add_two_ints_server')

        # Create a service server that handles AddTwoInts requests
        self.srv = self.create_service(
            AddTwoInts,              # Service type
            '/add_two_ints',         # Service name
            self.add_two_ints_callback  # Callback function
        )

        self.get_logger().info('AddTwoInts service server started at /add_two_ints')

    def add_two_ints_callback(self, request, response):
        """
        Service callback function to handle addition requests.

        Args:
            request (AddTwoInts.Request): Contains 'a' and 'b' integers
            response (AddTwoInts.Response): Contains 'sum' result

        Returns:
            AddTwoInts.Response: Response with the sum
        """
        # Perform the computation
        response.sum = request.a + request.b

        # Log the operation
        self.get_logger().info(
            f'Incoming request: a={request.a}, b={request.b} -> sum={response.sum}'
        )

        return response


class AddTwoIntsClient(Node):
    """
    Service client that sends addition requests to the server.

    This demonstrates the basic service client pattern:
    1. Create a service client object
    2. Wait for the service to become available
    3. Send a request and wait for the response
    """

    def __init__(self, a, b):
        super().__init__('add_two_ints_client')

        # Create a service client for AddTwoInts
        self.cli = self.create_client(AddTwoInts, '/add_two_ints')

        # Wait for the service to become available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /add_two_ints to become available...')

        # Create a request
        self.req = AddTwoInts.Request()
        self.req.a = a
        self.req.b = b

        self.get_logger().info(f'Service client ready. Sending request: a={a}, b={b}')

    def send_request(self):
        """
        Send the request to the server and return a future for the response.

        Returns:
            rclpy.task.Future: A future object representing the pending response
        """
        self.get_logger().info(f'Calling service with a={self.req.a}, b={self.req.b}')
        self.future = self.cli.call_async(self.req)
        return self.future


def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)

    # Create a temporary node to read parameters
    temp_node = rclpy.create_node('param_reader')
    temp_node.declare_parameter('mode', 'server')
    temp_node.declare_parameter('a', 0)
    temp_node.declare_parameter('b', 0)

    mode = temp_node.get_parameter('mode').value
    a = temp_node.get_parameter('a').value
    b = temp_node.get_parameter('b').value

    temp_node.destroy_node()

    if mode == 'server':
        # Run as service server
        node = AddTwoIntsServer()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()

    elif mode == 'client':
        # Run as service client
        node = AddTwoIntsClient(a, b)
        future = node.send_request()

        # Wait for the service call to complete
        rclpy.spin_until_future_complete(node, future)

        if future.result() is not None:
            response = future.result()
            node.get_logger().info(
                f'Service response received: {a} + {b} = {response.sum}'
            )
        else:
            node.get_logger().error('Service call failed')

        node.destroy_node()

    else:
        print(f"Error: Invalid mode '{mode}'. Use 'server' or 'client'.")
        return

    rclpy.shutdown()


if __name__ == '__main__':
    main()
