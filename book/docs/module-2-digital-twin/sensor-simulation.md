---
title: 2.5 Sensor Simulation
sidebar_position: 5
---

# 2.5 Sensor Simulation

## Introduction

Realistic sensor simulation is **critical** for developing perception algorithms. Without accurate sensor noise, range limitations, and update rates, your navigation/perception code will fail catastrophically when deployed on real hardware.

This section covers simulating:
1. **LiDAR 2D**: 360° laser range finder (for obstacle detection, SLAM)
2. **Depth Camera**: RGB-D sensor (for object recognition, manipulation)
3. **IMU**: Inertial Measurement Unit (for orientation estimation, odometry)

## LiDAR 2D Sensor

Add to URDF:

```xml title="book/docs/module-2-digital-twin/assets/code-examples/urdf/gazebo_sensors.urdf.xacro"
<gazebo reference="head">
  <sensor name="lidar_sensor" type="ray">
    <pose>0 0 0.2 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>30</update_rate>

    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.10</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </ray>

    <plugin name="laser_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/</namespace>
        <argument>~/out:=scan</argument>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>head</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**Verify**:
```bash
ros2 topic echo /scan
ros2 topic hz /scan  # Should show ~30 Hz
```

## Depth Camera

Add RGB-D camera:

```xml
<gazebo reference="head">
  <sensor name="depth_camera" type="depth">
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>

    <plugin name="depth_camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/camera</namespace>
      </ros>
      <camera_name>depth_camera</camera_name>
      <frame_name>head</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**Topics Published**:
- `/camera/image_raw` (RGB image)
- `/camera/depth/image_raw` (depth image)
- `/camera/camera_info` (calibration)

## IMU Sensor

Add IMU:

```xml
<gazebo reference="torso">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x><noise type="gaussian"><mean>0.0</mean><stddev>0.01</stddev></noise></x>
        <y><noise type="gaussian"><mean>0.0</mean><stddev>0.01</stddev></noise></y>
        <z><noise type="gaussian"><mean>0.0</mean><stddev>0.01</stddev></noise></z>
      </angular_velocity>
      <linear_acceleration>
        <x><noise type="gaussian"><mean>0.0</mean><stddev>0.01</stddev></noise></x>
        <y><noise type="gaussian"><mean>0.0</mean><stddev>0.01</stddev></noise></y>
        <z><noise type="gaussian"><mean>0.0</mean><stddev>0.01</stddev></noise></z>
      </linear_acceleration>
    </imu>

    <plugin name="imu_controller" filename="libgazebo_ros_imu_sensor.so">
      <ros><namespace>/imu</namespace></ros>
      <frame_name>torso</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## Validate Sensors

```python title="book/docs/module-2-digital-twin/assets/code-examples/scripts/validate_sensors.py"
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
        self.get_logger().info(f'LiDAR: {self.lidar_count} Hz, Camera: {self.camera_count} Hz, IMU: {self.imu_count} Hz')
        self.lidar_count = 0
        self.camera_count = 0
        self.imu_count = 0

def main(args=None):
    rclpy.init(args=args)
    node = SensorValidator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Section Summary

**Key Sensors**:
1. **LiDAR**: 360° obstacle detection
2. **Depth Camera**: RGB-D for manipulation
3. **IMU**: Orientation and acceleration

**Next Section**: [2.4 Unity High-Fidelity Rendering](./unity-rendering.md)
