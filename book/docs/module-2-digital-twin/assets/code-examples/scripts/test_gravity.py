#!/usr/bin/env python3
"""
Test Gravity Toggle Script

Demonstrates Gazebo physics by enabling/disabling gravity and observing robot behavior.

Usage:
    python3 test_gravity.py

Expected Behavior:
    - Gravity OFF: Robot floats in place (no forces)
    - Gravity ON: Robot falls at 9.81 m/s² acceleration

Author: Physical AI Textbook - Chapter 2
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetPhysicsProperties, GetPhysicsProperties
from geometry_msgs.msg import Vector3
import time

class GravityToggleNode(Node):
    def __init__(self):
        super().__init__('gravity_toggle_node')

        # Service clients
        self.get_physics_client = self.create_client(
            GetPhysicsProperties, '/get_physics_properties'
        )
        self.set_physics_client = self.create_client(
            SetPhysicsProperties, '/set_physics_properties'
        )

        # Wait for services
        while not self.get_physics_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /get_physics_properties service...')
        while not self.set_physics_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_physics_properties service...')

        self.get_logger().info('Gravity toggle ready!')

    def get_current_gravity(self):
        """Retrieve current gravity vector from Gazebo."""
        request = GetPhysicsProperties.Request()
        future = self.get_physics_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return response.gravity

    def set_gravity(self, gravity_vector):
        """Set gravity vector in Gazebo."""
        # Get current physics properties
        get_request = GetPhysicsProperties.Request()
        future = self.get_physics_client.call_async(get_request)
        rclpy.spin_until_future_complete(self, future)
        current_props = future.result()

        # Modify only gravity, keep other properties
        set_request = SetPhysicsProperties.Request()
        set_request.time_step = current_props.time_step
        set_request.max_update_rate = current_props.max_update_rate
        set_request.gravity = gravity_vector
        set_request.ode_config = current_props.ode_config

        # Send update
        future = self.set_physics_client.call_async(set_request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    def run_test(self):
        """Run gravity toggle experiment."""
        self.get_logger().info('=== Gravity Toggle Test ===')

        # Test 1: Disable gravity
        self.get_logger().info('Setting gravity to ZERO...')
        zero_gravity = Vector3(x=0.0, y=0.0, z=0.0)
        success = self.set_gravity(zero_gravity)
        if success:
            self.get_logger().info('✓ Gravity disabled. Robot should float.')
        time.sleep(5)

        # Test 2: Enable standard gravity
        self.get_logger().info('Setting gravity to Earth normal (9.81 m/s²)...')
        earth_gravity = Vector3(x=0.0, y=0.0, z=-9.81)
        success = self.set_gravity(earth_gravity)
        if success:
            self.get_logger().info('✓ Gravity enabled. Robot should fall.')
        time.sleep(5)

        # Test 3: Low gravity (Moon: 1.62 m/s²)
        self.get_logger().info('Setting gravity to Moon level (1.62 m/s²)...')
        moon_gravity = Vector3(x=0.0, y=0.0, z=-1.62)
        success = self.set_gravity(moon_gravity)
        if success:
            self.get_logger().info('✓ Low gravity set. Robot falls slower.')
        time.sleep(5)

        # Restore Earth gravity
        self.set_gravity(earth_gravity)
        self.get_logger().info('=== Test Complete ===')

def main(args=None):
    rclpy.init(args=args)
    node = GravityToggleNode()
    node.run_test()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
