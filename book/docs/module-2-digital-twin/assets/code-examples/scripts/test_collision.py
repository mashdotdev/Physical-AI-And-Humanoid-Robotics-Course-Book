#!/usr/bin/env python3
"""
Test Collision Detection Script

Spawns a box obstacle in front of robot and verifies collision detection.

Usage:
    python3 test_collision.py

Expected: Robot collides with box, collision forces prevent penetration.

Author: Physical AI Textbook - Chapter 2
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose
import time

class CollisionTestNode(Node):
    def __init__(self):
        super().__init__('collision_test_node')

        # Service clients
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')

        # Wait for services
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity...')

        self.get_logger().info('Collision test ready!')

    def spawn_box(self, name, x, y, z, size=0.5):
        """Spawn a box obstacle in Gazebo."""
        box_sdf = f"""
        <?xml version="1.0"?>
        <sdf version="1.6">
          <model name="{name}">
            <static>true</static>
            <link name="link">
              <collision name="collision">
                <geometry>
                  <box>
                    <size>{size} {size} {size}</size>
                  </box>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <box>
                    <size>{size} {size} {size}</size>
                  </box>
                </geometry>
                <material>
                  <script>
                    <uri>file://media/materials/scripts/gazebo.material</uri>
                    <name>Gazebo/Red</name>
                  </script>
                </material>
              </visual>
            </link>
          </model>
        </sdf>
        """

        request = SpawnEntity.Request()
        request.name = name
        request.xml = box_sdf
        request.robot_namespace = ''
        request.initial_pose = Pose()
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = z

        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    def delete_entity(self, name):
        """Delete an entity from Gazebo."""
        request = DeleteEntity.Request()
        request.name = name
        future = self.delete_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    def run_test(self):
        """Run collision detection test."""
        self.get_logger().info('=== Collision Detection Test ===')

        # Spawn box 2m in front of robot
        self.get_logger().info('Spawning box obstacle at (2, 0, 0.25)...')
        success = self.spawn_box('test_box', x=2.0, y=0.0, z=0.25)
        if success:
            self.get_logger().info('✓ Box spawned successfully')

        # Wait for observation
        self.get_logger().info('Push the robot toward the box in Gazebo GUI to test collision.')
        time.sleep(10)

        # Clean up
        self.get_logger().info('Deleting box...')
        self.delete_entity('test_box')
        self.get_logger().info('=== Test Complete ===')

def main(args=None):
    rclpy.init(args=args)
    node = CollisionTestNode()
    node.run_test()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
