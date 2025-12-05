---
sidebar_position: 3
title: 1.3 Integrating AI with Robot Controllers
---

# 1.3 Integrating High-Level AI with Robot Controllers

Modern humanoid robots combine **high-level AI decision-making** (vision-language-action models, LLMs) with **low-level robot controllers** (motor drivers, trajectory planners). This section explores how ROS 2 bridges these two worlds.

## The AI-Robot Integration Challenge

Imagine you want your humanoid robot to respond to natural language commands like:
- "Pick up the red cup on the table"
- "Walk to the kitchen and open the fridge"
- "Wave hello to the person in front of you"

These high-level instructions must be translated into **thousands of low-level control signals** sent to motors, actuators, and sensors at high frequencies (50-1000 Hz).

### The Architecture

```text
┌─────────────────────────────────────────────────────────────────┐
│                     AI Decision-Making Layer                     │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐  │
│  │   Vision     │  │   Language   │  │  Action Planning     │  │
│  │   (VLA)      │  │   (LLM)      │  │  (Behavior Trees)    │  │
│  └──────┬───────┘  └──────┬───────┘  └──────────┬───────────┘  │
└─────────┼──────────────────┼─────────────────────┼──────────────┘
          │                  │                     │
          └──────────────────┼─────────────────────┘
                             │ High-level commands (ROS 2 Topics)
                             ▼
          ┌────────────────────────────────────────────┐
          │        AI-Robot Bridge Node                │
          │  • Command parsing                         │
          │  • Validation & safety checks              │
          │  • Translation to control messages         │
          └─────────────────┬──────────────────────────┘
                            │ Control commands (Twist, JointState)
                            ▼
          ┌────────────────────────────────────────────┐
          │       Robot Controller Layer               │
          │  ┌──────────────┐  ┌──────────────────┐   │
          │  │ Motion       │  │  Motor           │   │
          │  │ Planner      │  │  Controller      │   │
          │  └──────┬───────┘  └────────┬─────────┘   │
          └─────────┼──────────────────────┼───────────┘
                    │                      │
                    ▼                      ▼
          ┌─────────────────────────────────────────┐
          │         Hardware Layer                  │
          │  • Motors  • Sensors  • Actuators       │
          └─────────────────────────────────────────┘
```

### Why a Bridge Node?

The **bridge node** serves several critical functions:

1. **Command Translation**: Converts semantic commands ("move forward") to control signals (linear velocity = 0.3 m/s)
2. **Safety Validation**: Ensures AI commands don't exceed hardware limits or violate safety constraints
3. **Error Handling**: Gracefully handles invalid or ambiguous commands
4. **Feedback Loop**: Reports execution status back to the AI system for closed-loop control

## Code Example: AI-Robot Bridge

Let's implement a bridge that translates AI commands to robot velocity controls.

### Bridge Node Implementation

```python title="src/ros2_chapter1/nodes/ai_robot_bridge.py"
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class AIRobotBridge(Node):
    def __init__(self):
        super().__init__('ai_robot_bridge')

        # Subscribe to AI commands
        self.ai_subscription = self.create_subscription(
            String, '/ai/command', self.ai_command_callback, 10
        )

        # Publish to robot controller
        self.robot_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Safety limits
        self.max_linear_velocity = 0.5  # m/s
        self.max_angular_velocity = 1.0  # rad/s

        # Command mapping
        self.command_map = {
            'move_forward': (0.3, 0.0),
            'move_backward': (-0.3, 0.0),
            'turn_left': (0.0, 0.5),
            'turn_right': (0.0, -0.5),
            'stop': (0.0, 0.0),
        }

    def ai_command_callback(self, msg):
        command = msg.data.strip().lower()

        try:
            cmd_vel = self.parse_command(command)

            if self.is_safe(cmd_vel):
                self.execute_command(cmd_vel)
            else:
                self.get_logger().warn(f'Command rejected: {command}')
        except ValueError as e:
            self.get_logger().error(f'Invalid command: {e}')

    def parse_command(self, command):
        """Translate high-level command to velocity."""
        cmd_vel = Twist()

        if command in self.command_map:
            linear, angular = self.command_map[command]
            cmd_vel.linear.x = linear
            cmd_vel.angular.z = angular
            return cmd_vel

        raise ValueError(f'Unrecognized command: {command}')

    def is_safe(self, cmd_vel):
        """Validate against safety limits."""
        return (abs(cmd_vel.linear.x) <= self.max_linear_velocity and
                abs(cmd_vel.angular.z) <= self.max_angular_velocity)

    def execute_command(self, cmd_vel):
        """Publish validated command to robot."""
        self.robot_publisher.publish(cmd_vel)
```

### Running the Bridge

**Terminal 1 (Bridge Node):**
```bash
python3 src/ros2_chapter1/nodes/ai_robot_bridge.py
```

**Terminal 2 (Simulate AI Commands):**
```bash
# Send high-level commands
ros2 topic pub /ai/command std_msgs/msg/String "{data: 'move_forward'}"
ros2 topic pub /ai/command std_msgs/msg/String "{data: 'turn_left'}"
ros2 topic pub /ai/command std_msgs/msg/String "{data: 'stop'}"
```

**Terminal 3 (Monitor Robot Commands):**
```bash
ros2 topic echo /cmd_vel
```

## Safety Considerations

Integrating AI with physical robots introduces **safety-critical challenges**:

### 1. Command Validation

**Problem**: AI models can generate invalid or dangerous commands.

**Solution**: Implement strict validation and sanity checks:

```python
def is_safe(self, cmd_vel):
    """Multi-layer safety validation."""

    # Check velocity limits
    if abs(cmd_vel.linear.x) > self.max_linear_velocity:
        return False

    # Check collision zones (requires sensor data)
    if self.detect_obstacle_ahead() and cmd_vel.linear.x > 0:
        return False

    # Check robot state (e.g., emergency stop engaged)
    if self.is_emergency_stopped():
        return False

    return True
```

### 2. Timeout and Watchdog

**Problem**: AI system crashes or stops sending commands.

**Solution**: Implement command timeout with automatic emergency stop:

```python
class AIRobotBridge(Node):
    def __init__(self):
        # ...
        self.last_command_time = self.get_clock().now()
        self.command_timeout = 1.0  # seconds

        # Watchdog timer
        self.timer = self.create_timer(0.1, self.watchdog_callback)

    def watchdog_callback(self):
        """Emergency stop if no commands received within timeout."""
        time_since_last_cmd = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9

        if time_since_last_cmd > self.command_timeout:
            self.get_logger().warn('Command timeout - issuing emergency stop')
            stop_cmd = Twist()
            self.robot_publisher.publish(stop_cmd)

    def ai_command_callback(self, msg):
        self.last_command_time = self.get_clock().now()
        # ... process command
```

### 3. Graceful Degradation

**Problem**: AI system produces low-confidence or ambiguous outputs.

**Solution**: Implement confidence thresholds and fallback behaviors:

```python
def ai_command_callback(self, msg):
    command, confidence = self.parse_ai_output(msg.data)

    if confidence < 0.8:
        self.get_logger().warn(f'Low confidence ({confidence:.2f}) - using safe fallback')
        self.execute_safe_fallback()
    else:
        self.execute_command(command)

def execute_safe_fallback(self):
    """Conservative behavior when AI is uncertain."""
    # Option 1: Stop and request clarification
    self.stop_robot()
    self.request_user_input()

    # Option 2: Execute only high-confidence sub-commands
    self.execute_partial_plan()
```

### 4. Emergency Stop Interface

**Always** provide a hardware or software emergency stop:

```python
class AIRobotBridge(Node):
    def __init__(self):
        # ...
        self.emergency_stop_subscription = self.create_subscription(
            Bool, '/emergency_stop', self.emergency_stop_callback, 10
        )
        self.is_emergency_stopped = False

    def emergency_stop_callback(self, msg):
        if msg.data:
            self.is_emergency_stopped = True
            self.get_logger().error('EMERGENCY STOP ACTIVATED')

            # Send zero velocity
            stop_cmd = Twist()
            self.robot_publisher.publish(stop_cmd)
```

## Error Handling Patterns

### Pattern 1: Try-Catch with Logging

```python
def ai_command_callback(self, msg):
    try:
        cmd_vel = self.parse_command(msg.data)
        self.execute_command(cmd_vel)
    except ValueError as e:
        self.get_logger().error(f'Parse error: {e}')
    except RuntimeError as e:
        self.get_logger().error(f'Execution error: {e}')
        self.execute_safe_fallback()
```

### Pattern 2: State Machine for Recovery

```python
from enum import Enum

class BridgeState(Enum):
    NORMAL = 1
    ERROR = 2
    RECOVERY = 3
    EMERGENCY = 4

class AIRobotBridge(Node):
    def __init__(self):
        # ...
        self.state = BridgeState.NORMAL

    def ai_command_callback(self, msg):
        if self.state == BridgeState.EMERGENCY:
            self.get_logger().warn('In emergency state - ignoring commands')
            return

        if self.state == BridgeState.RECOVERY:
            self.attempt_recovery()
            return

        # Normal processing...
```

### Pattern 3: Feedback Loop

Provide status updates back to the AI system:

```python
def execute_command(self, cmd_vel):
    self.robot_publisher.publish(cmd_vel)

    # Send feedback to AI
    status_msg = String()
    status_msg.data = f'EXECUTING: linear={cmd_vel.linear.x}, angular={cmd_vel.angular.z}'
    self.status_publisher.publish(status_msg)
```

## Real-World Integration Example

Here's how this pattern is used in production humanoid robots:

```python
"""
Production-grade AI-Robot bridge for humanoid manipulation.

AI Input: "Pick up the red cup on the table"
Processing:
  1. Vision model identifies cup location (x, y, z)
  2. Grasp planner computes hand approach trajectory
  3. Inverse kinematics calculates joint angles
  4. Motion controller executes arm movement
  5. Gripper actuates to grasp
"""

class HumanoidManipulationBridge(Node):
    def ai_command_callback(self, msg):
        # msg.data = "pick up red cup"

        # Step 1: Parse high-level command
        action, object_name = self.parse_task(msg.data)

        # Step 2: Perception (get object pose from vision)
        object_pose = self.query_vision_service(object_name)

        # Step 3: Motion planning (call MoveIt2 service)
        trajectory = self.plan_grasp_trajectory(object_pose)

        # Step 4: Safety check
        if not self.is_trajectory_safe(trajectory):
            self.report_failure("Unsafe trajectory")
            return

        # Step 5: Execute action (send to robot controller)
        self.execute_trajectory_action(trajectory)
```

## Reality Check: Points of Failure

When integrating AI with robots, expect failures at:

| Component | Common Failures | Mitigation |
|-----------|----------------|------------|
| **AI Model** | Hallucinations, low confidence, invalid outputs | Confidence thresholds, validation layers |
| **Perception** | Object detection fails, occlusions, lighting changes | Fallback to manual control, multi-sensor fusion |
| **Network** | Latency spikes, dropped messages | Local processing, QoS policies, timeouts |
| **Hardware** | Motor faults, sensor drift, battery depletion | Health monitoring, graceful degradation |
| **Bridge Logic** | Parsing errors, state inconsistencies | Extensive logging, state machines |

:::danger Critical Safety Rule
**Never trust AI output blindly.** Always validate, sanitize, and limit commands before execution on physical hardware.
:::

## Summary

In this section, we learned:
- ✅ How to bridge high-level AI decision-making with low-level robot control via ROS 2
- ✅ The architecture of an AI-robot integration system
- ✅ Critical safety considerations: validation, timeouts, emergency stops
- ✅ Error handling patterns for robust operation
- ✅ Real-world failure modes and mitigation strategies

**Key Takeaways:**
- The **bridge node** is the safety-critical interface between AI and hardware
- **Command validation** prevents AI hallucinations from causing damage
- **Watchdog timers** protect against AI system crashes
- **Graceful degradation** ensures safe behavior under uncertainty
- **Feedback loops** enable closed-loop AI control

Next, we'll visualize robot structures using URDF (Unified Robot Description Format) to define the physical layout of humanoid robots.

## Further Reading

- [ROS 2 Control: Hardware Interface](https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/hardware_interface_types_userdoc.html)
- [MoveIt 2: Motion Planning for Manipulation](https://moveit.picknik.ai/main/index.html)
- [Safety in Human-Robot Interaction](https://ieeexplore.ieee.org/document/9387087)
- [Real-Time Systems in ROS 2](https://design.ros2.org/articles/realtime_background.html)
