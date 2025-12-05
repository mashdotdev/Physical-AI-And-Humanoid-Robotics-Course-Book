#!/usr/bin/env python3
"""
Validate Sensor Data Script

Checks that all sensors publish at expected rates.

Usage:
    python3 validate_sensors.py

Author: Physical AI Textbook - Chapter 2
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu

class SensorValidator(Node):
    def __init__(self):
        super().__init__('sensor_validator')

        self.lidar_count = 0
        self.camera_count = 0
        self.imu_count = 0

        # Subscribers
        self.create_subscription(LaserScan, '/scan', lambda msg: self.count('lidar'), 10)
        self.create_subscription(Image, '/camera/image_raw', lambda msg: self.count('camera'), 10)
        self.create_subscription(Imu, '/imu/data', lambda msg: self.count('imu'), 10)

        # Timer to report rates
        self.create_timer(1.0, self.report_rates)

        self.get_logger().info('Sensor validator started!')

    def count(self, sensor):
        if sensor == 'lidar':
            self.lidar_count += 1
        elif sensor == 'camera':
            self.camera_count += 1
        elif sensor == 'imu':
            self.imu_count += 1

    def report_rates(self):
        self.get_logger().info(
            f'LiDAR: {self.lidar_count} Hz, '
            f'Camera: {self.camera_count} Hz, '
            f'IMU: {self.imu_count} Hz'
        )
        self.lidar_count = 0
        self.camera_count = 0
        self.imu_count = 0

def main(args=None):
    rclpy.init(args=args)
    node = SensorValidator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
