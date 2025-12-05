---
sidebar_position: 5
title: Exercises
---

# Chapter 1: Hands-On Exercises

Test your understanding of ROS 2 concepts with these practical exercises. Solutions are provided at the end, but try to solve them independently first!

## Exercise 1: Topic Communication - Temperature Monitor

**Difficulty**: ⭐ Beginner

**Goal**: Create a publisher that simulates temperature sensor readings and a subscriber that monitors for dangerous temperatures.

**Requirements**:
1. Create a **publisher node** that:
   - Publishes temperature readings (in Celsius) to `/robot/temperature`
   - Simulates temperature fluctuations (use `random.uniform(15.0, 35.0)`)
   - Publishes at 2 Hz

2. Create a **subscriber node** that:
   - Subscribes to `/robot/temperature`
   - Logs "WARNING: High temperature!" if temp > 30°C
   - Logs "DANGER: Overheating!" if temp > 33°C
   - Otherwise logs "Temperature normal: X°C"

**Hints**:
- Use `std_msgs/msg/Float32` for temperature messages
- Review `simple_publisher.py` and `simple_subscriber.py` from Section 1.2

<details>
<summary>Solution</summary>

```python title="temperature_publisher.py"
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random


class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')
        self.publisher_ = self.create_publisher(Float32, '/robot/temperature', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)  # 2 Hz
        self.get_logger().info('Temperature sensor started')

    def timer_callback(self):
        msg = Float32()
        msg.data = random.uniform(15.0, 35.0)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data:.1f}°C')


def main(args=None):
    rclpy.init(args=args)
    node = TemperaturePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

```python title="temperature_monitor.py"
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class TemperatureMonitor(Node):
    def __init__(self):
        super().__init__('temperature_monitor')
        self.subscription = self.create_subscription(
            Float32, '/robot/temperature', self.temperature_callback, 10
        )
        self.get_logger().info('Temperature monitor started')

    def temperature_callback(self, msg):
        temp = msg.data
        if temp > 33.0:
            self.get_logger().error(f'DANGER: Overheating! {temp:.1f}°C')
        elif temp > 30.0:
            self.get_logger().warn(f'WARNING: High temperature! {temp:.1f}°C')
        else:
            self.get_logger().info(f'Temperature normal: {temp:.1f}°C')


def main(args=None):
    rclpy.init(args=args)
    node = TemperatureMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

</details>

---

## Exercise 2: Service - Distance Calculator

**Difficulty**: ⭐⭐ Intermediate

**Goal**: Create a service that calculates the Euclidean distance between two 2D points.

**Requirements**:
1. Define a **service** at `/calculate_distance`
2. **Request**: Two points (x1, y1) and (x2, y2)
3. **Response**: Distance between the points
4. **Formula**: distance = √((x2-x1)² + (y2-y1)²)

**Hints**:
- Use `example_interfaces/srv/AddTwoInts` as a template (but you'll need 4 float inputs instead of 2 ints)
- For simplicity, use two separate service calls with 2 floats each, OR
- Create a custom message type (advanced)

<details>
<summary>Solution (Simplified with String Parsing)</summary>

```python title="distance_calculator.py"
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import math
import json


class DistanceCalculator(Node):
    def __init__(self):
        super().__init__('distance_calculator')
        self.srv = self.create_service(
            Trigger, '/calculate_distance', self.calculate_callback
        )
        self.get_logger().info('Distance calculator service started')

    def calculate_callback(self, request, response):
        """
        Expects request.data = '{"x1": 0.0, "y1": 0.0, "x2": 3.0, "y2": 4.0}'
        """
        try:
            data = json.loads(request.data) if hasattr(request, 'data') else {}
            x1, y1 = 0.0, 0.0  # Default point1
            x2, y2 = data.get('x2', 3.0), data.get('y2', 4.0)

            distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            response.success = True
            response.message = f'Distance: {distance:.2f}'
            self.get_logger().info(f'Calculated distance: {distance:.2f}')
        except Exception as e:
            response.success = False
            response.message = f'Error: {str(e)}'

        return response


def main(args=None):
    rclpy.init(args=args)
    node = DistanceCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Test**:
```bash
ros2 service call /calculate_distance std_srvs/srv/Trigger
```

</details>

---

## Exercise 3: AI Bridge Extension - Multi-Robot Control

**Difficulty**: ⭐⭐⭐ Advanced

**Goal**: Extend the AI-Robot bridge to control **two robots simultaneously** with namespaced topics.

**Requirements**:
1. Modify `ai_robot_bridge.py` to accept a **robot namespace** parameter (e.g., `robot1`, `robot2`)
2. Subscribe to `/ai/command`
3. Publish to `/{namespace}/cmd_vel` (e.g., `/robot1/cmd_vel`, `/robot2/cmd_vel`)
4. Support commands like: `"robot1 move_forward"`, `"robot2 turn_left"`, `"all stop"`

**Hints**:
- Use `self.declare_parameter('namespace', 'robot1')` to set namespace
- Parse the command to extract robot name
- Use a dictionary to manage multiple publishers

<details>
<summary>Solution</summary>

```python title="multi_robot_bridge.py"
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class MultiRobotBridge(Node):
    def __init__(self):
        super().__init__('multi_robot_bridge')

        # Subscribe to AI commands
        self.ai_subscription = self.create_subscription(
            String, '/ai/command', self.ai_command_callback, 10
        )

        # Create publishers for multiple robots
        self.robot_publishers = {
            'robot1': self.create_publisher(Twist, '/robot1/cmd_vel', 10),
            'robot2': self.create_publisher(Twist, '/robot2/cmd_vel', 10),
        }

        self.command_map = {
            'move_forward': (0.3, 0.0),
            'move_backward': (-0.3, 0.0),
            'turn_left': (0.0, 0.5),
            'turn_right': (0.0, -0.5),
            'stop': (0.0, 0.0),
        }

        self.get_logger().info('Multi-robot bridge started')

    def ai_command_callback(self, msg):
        command = msg.data.strip().lower()
        self.get_logger().info(f'Received: {command}')

        # Parse command: "robot1 move_forward" or "all stop"
        parts = command.split(maxsplit=1)
        if len(parts) < 2:
            self.get_logger().error('Invalid command format')
            return

        robot_name, action = parts[0], parts[1]

        # Handle "all" keyword
        if robot_name == 'all':
            for pub in self.robot_publishers.values():
                self.publish_command(pub, action)
        elif robot_name in self.robot_publishers:
            self.publish_command(self.robot_publishers[robot_name], action)
        else:
            self.get_logger().error(f'Unknown robot: {robot_name}')

    def publish_command(self, publisher, action):
        if action in self.command_map:
            cmd_vel = Twist()
            linear, angular = self.command_map[action]
            cmd_vel.linear.x = linear
            cmd_vel.angular.z = angular
            publisher.publish(cmd_vel)
        else:
            self.get_logger().error(f'Unknown action: {action}')


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Test**:
```bash
ros2 topic pub /ai/command std_msgs/msg/String "{data: 'robot1 move_forward'}"
ros2 topic pub /ai/command std_msgs/msg/String "{data: 'robot2 turn_left'}"
ros2 topic pub /ai/command std_msgs/msg/String "{data: 'all stop'}"
```

</details>

---

## Exercise 4: URDF - Add a Third Joint

**Difficulty**: ⭐⭐ Intermediate

**Goal**: Extend `simple_2_joint_robot.urdf` to add a **wrist joint** after the elbow.

**Requirements**:
1. Add a new **revolute joint** named `wrist_joint` after the elbow
2. Add a new **link** named `wrist_link` (small cylinder)
3. Joint should rotate around the Z-axis (yaw motion)
4. Limits: -1.57 to 1.57 radians

**Hints**:
- Copy the elbow joint/link structure
- Change the `axis` to `xyz="0 0 1"` for yaw rotation
- Update the parent link to `elbow_link`

<details>
<summary>Solution</summary>

Add this to your URDF after the `elbow_link`:

```xml
  <!-- Wrist Joint (Revolute - Yaw Motion) -->
  <joint name="wrist_joint" type="revolute">
    <parent link="elbow_link"/>
    <child link="wrist_link"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>  <!-- Rotates around Z-axis (yaw) -->
    <limit effort="3.0" lower="-1.57" upper="1.57" velocity="1.0"/>
  </joint>

  <!-- Wrist Link -->
  <link name="wrist_link">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.1"/>
      </geometry>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <material name="purple">
        <color rgba="0.6 0.2 0.8 1.0"/>
      </material>
    </visual>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0"
               iyy="0.0001" iyz="0.0" izz="0.00005"/>
    </inertial>
  </link>

  <!-- Update End Effector Joint Parent -->
  <joint name="end_effector_joint" type="fixed">
    <parent link="wrist_link"/>  <!-- Changed from elbow_link -->
    <child link="end_effector_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>
```

</details>

---

## Challenge Exercise: Mini Humanoid Arm

**Difficulty**: ⭐⭐⭐⭐ Expert

**Goal**: Create a complete humanoid arm with shoulder (3 DOF), elbow (1 DOF), and wrist (2 DOF).

**Requirements**:
1. **Shoulder**: 3 revolute joints (roll, pitch, yaw)
2. **Elbow**: 1 revolute joint (pitch only)
3. **Wrist**: 2 revolute joints (pitch, yaw)
4. Realistic joint limits based on human anatomy
5. Launch file that opens in RViz with configured view

**Reference: Human Arm Joint Ranges**
- Shoulder roll: -180° to 180°
- Shoulder pitch: -90° to 180°
- Shoulder yaw: -90° to 90°
- Elbow pitch: 0° to 150°
- Wrist pitch: -90° to 90°
- Wrist yaw: -90° to 90°

This is a challenging exercise! Use the concepts from all sections of this chapter.

---

## Reflection Questions

After completing the exercises, reflect on these questions:

1. **When would you choose a topic vs. a service** for robot communication? Give 3 specific examples.

2. **What safety mechanisms** would you add to the AI-Robot bridge for a production humanoid robot?

3. **Why is URDF important** for motion planning and collision avoidance? What would happen if your URDF didn't match the physical robot?

4. **Describe the flow** from a high-level AI command like "pick up the cup" to the robot's motors actually moving. What ROS 2 components are involved at each stage?

5. **How would you debug** a situation where your subscriber isn't receiving messages from a publisher? List 5 troubleshooting steps.

---

## Next Steps

Congratulations on completing Chapter 1! You now understand:
- ✅ The fundamentals of ROS 2 communication (topics, services, actions)
- ✅ How to integrate AI systems with robot controllers
- ✅ How to define and visualize robot structures with URDF

**Continue your journey:**
- **Chapter 2**: Perception and Vision Systems (Coming Soon)
- **Chapter 3**: Motion Planning and Control (Coming Soon)
- **Chapter 4**: Navigation and Localization (Coming Soon)

## Additional Resources

- [ROS 2 Examples Repository](https://github.com/ros2/examples)
- [Official ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS 2 Design Documentation](https://design.ros2.org/)
- [Robotics Stack Exchange](https://robotics.stackexchange.com/)
