# Contract: ROS 2 Control Configuration Schema

**Purpose**: Define the configuration structure for ros2_control controllers in YAML format.

**Version**: 1.0
**Date**: 2025-12-05

---

## Controller Manager Configuration (`controllers.yaml`)

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    # List of controllers to load
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    position_controller:
      type: position_controllers/JointGroupPositionController

## Joint State Broadcaster (Required)

```yaml
joint_state_broadcaster:
  ros__parameters:
    # No additional config needed
    # Automatically publishes /joint_states
```

**Contract**:
- Publishes `sensor_msgs/JointState` to `/joint_states` (100 Hz)
- MUST be loaded before any other controllers

---

## Position Controller

```yaml
position_controller:
  ros__parameters:
    joints:
      - shoulder_pitch_joint
      - shoulder_roll_joint
      - elbow_joint
      - wrist_joint

    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity
```

**Contract**:
- Subscribes to: `/position_controller/commands` (std_msgs/Float64MultiArray)
- Command array length MUST equal number of joints
- Commands MUST respect URDF joint limits

---

## Validation

```bash
ros2 control load_controller joint_state_broadcaster
ros2 control load_controller position_controller
ros2 control set_controller_state joint_state_broadcaster start
ros2 control set_controller_state position_controller start

# Test
ros2 topic pub /position_controller/commands std_msgs/Float64MultiArray "data: [0.5, 0.0, -1.0, 0.3]"
```