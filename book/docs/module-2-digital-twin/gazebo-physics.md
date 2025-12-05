---
title: 2.2 Gazebo Physics Simulation
sidebar_position: 2
---

# 2.2 Gazebo Physics Simulation

## Introduction

Physics simulation is the **foundation** of any digital twin. Without accurate gravity, friction, collision detection, and joint dynamics, your virtual robot won't behave like the real one—and any control algorithms you develop in simulation will fail catastrophically on hardware.

**Gazebo Classic** is the industry-standard physics simulator for ROS robotics. It provides:
- **Real-time physics**: ODE (Open Dynamics Engine) solver running at 1000 Hz
- **Realistic contact dynamics**: Friction, bouncing, joint limits, actuator constraints
- **ROS 2 integration**: Seamless communication with controllers, sensors, and visualization tools

In this section, you'll learn to:
1. Install and configure Gazebo for ROS 2 Humble
2. Load your humanoid robot URDF into Gazebo
3. Test physics fundamentals (gravity, friction, collisions, inertia)
4. Validate simulation accuracy with automated scripts

## Installation

### Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble installed (from Chapter 1)
- Workspace set up with `colcon` build system

### Install Gazebo Classic 11

```bash
# Install Gazebo Classic 11.x
sudo apt update
sudo apt install gazebo

# Install ROS 2 Gazebo bridge packages
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control

# Verify installation
gazebo --version
# Expected output: Gazebo multi-robot simulator, version 11.x.x
```

:::info Gazebo Classic vs. Ignition Gazebo

This chapter uses **Gazebo Classic 11** because:
- Mature, stable, well-documented
- Lower system requirements (runs on CPU)
- Direct ROS 2 plugin integration

**Ignition Gazebo** (now called "Gazebo") is the next-generation simulator with better performance, but requires Ogre2 rendering and more complex setup. We'll introduce it in advanced topics.

:::

## Loading a URDF Robot into Gazebo

### Step 1: Understand the URDF → Gazebo Pipeline

**Data Flow**:
```
URDF File (.urdf or .urdf.xacro)
    ↓
robot_state_publisher (ROS 2 node)
    ↓
/robot_description parameter (ROS 2 parameter server)
    ↓
spawn_entity.py script (Gazebo ROS bridge)
    ↓
Gazebo Physics Engine (simulation running)
```

### Step 2: Create a Launch File

The launch file orchestrates starting Gazebo and spawning the robot. Create `gazebo_sim.launch.py`:

```python title="book/docs/module-2-digital-twin/assets/code-examples/launch/gazebo_sim.launch.py"
"""
Gazebo Simulation Launch File

Starts Gazebo and spawns the humanoid robot from URDF.

Usage:
    ros2 launch digital_twin_chapter2 gazebo_sim.launch.py

Author: Physical AI Textbook - Chapter 2
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Paths
    pkg_share = get_package_share_directory('digital_twin_chapter2')
    urdf_file = os.path.join(pkg_share, 'urdf', 'humanoid_robot.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'empty_world.world')

    # Process xacro to URDF
    robot_description = xacro.process_file(urdf_file).toxml()

    # Robot State Publisher (publishes TF tree from URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True  # CRITICAL: Use Gazebo's /clock topic
        }]
    )

    # Gazebo Server (physics engine, runs headless)
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Gazebo Client (GUI, optional for debugging)
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # Spawn Robot Entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',  # Read from robot_state_publisher
            '-entity', 'humanoid_robot',      # Name in Gazebo
            '-x', '0', '-y', '0', '-z', '1.0', # Spawn 1m above ground
        ],
        output='screen'
    )

    # Event handler: Spawn robot after Gazebo starts
    spawn_after_gazebo = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_server,
            on_exit=[spawn_entity]
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo_server,
        gazebo_client,
        spawn_entity,
    ])
```

**Key Configuration**:
- `use_sim_time: True` → All ROS nodes use Gazebo's `/clock` instead of system time (ensures perfect synchronization)
- `-z 1.0` → Spawn robot 1m above ground to test gravity
- `gzserver` → Physics engine (headless)
- `gzclient` → GUI (optional, disable for headless servers)

### Step 3: Launch Gazebo

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash
source ~/your_workspace/install/setup.bash

# Launch Gazebo with robot
ros2 launch digital_twin_chapter2 gazebo_sim.launch.py
```

**Expected Output**:
- Gazebo GUI opens showing empty world with ground plane
- Humanoid robot appears 1m above ground
- Robot falls due to gravity and lands on feet (if balanced) or tips over (if unbalanced)

## Physics Fundamentals

### 1. Gravity

**What it is**: Constant downward acceleration (Earth: 9.81 m/s²)

**How Gazebo models it**: Applies force `F = m * g` to every link's center of mass every physics timestep (1ms)

**Test**: Toggle gravity on/off and observe robot behavior

```python title="book/docs/module-2-digital-twin/assets/code-examples/scripts/test_gravity.py"
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
```

**Run Test**:
```bash
python3 test_gravity.py
```

**Expected Observations**:
- Gravity OFF → Robot hovers at spawn height
- Gravity ON → Robot accelerates downward at 9.81 m/s²
- Moon gravity → Robot falls 6x slower than Earth

### 2. Friction

**What it is**: Resistance to sliding motion between two surfaces in contact

**How Gazebo models it**: Coulomb friction model with two coefficients:
- `mu1` (friction direction 1): Along surface tangent
- `mu2` (friction direction 2): Perpendicular to direction 1

**Typical Values**:
- Metal on metal: 0.3-0.6
- Rubber on concrete: 0.7-1.0
- Ice on ice: 0.02-0.05
- Teflon: 0.04

**Configuration** (in URDF `<gazebo>` tags):
```xml
<gazebo reference="left_foot">
  <mu1>1.0</mu1>  <!-- High friction for standing stability -->
  <mu2>1.0</mu2>
  <material>Gazebo/Grey</material>
</gazebo>
```

### 3. Mass and Inertia

**What it is**:
- **Mass**: Resistance to linear acceleration (kg)
- **Inertia tensor**: Resistance to rotational acceleration (kg·m²)

**Why it matters**: Incorrect inertia → unrealistic dynamics
- Too low: Robot spins like a top with tiny torques
- Too high: Robot moves sluggishly, can't achieve target velocities

**How to calculate**:
- Use CAD software (SolidWorks, Fusion 360) to export mass properties
- Analytical formulas for simple shapes (box, cylinder, sphere)

**Example: Box Inertia**:
```
I_xx = (1/12) * m * (h² + d²)
I_yy = (1/12) * m * (h² + w²)
I_zz = (1/12) * m * (w² + d²)
```

Where: m = mass, w = width, h = height, d = depth

### 4. Collision Detection

**What it is**: Detecting when two geometric shapes intersect

**How Gazebo does it**:
- Checks collision geometry (NOT visual geometry) every physics step
- Computes contact points, normals, penetration depth
- Applies contact forces to prevent inter-penetration

**Performance Tip**: Simplify collision meshes!
- Visual mesh: Can be 10,000+ triangles (high detail)
- Collision mesh: Should be &lt;1,000 triangles (convex hulls preferred)

**Test Collision Detection**:

```python title="book/docs/module-2-digital-twin/assets/code-examples/scripts/test_collision.py"
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
```

### 5. Joint Limits and Damping

**Joint Limits**: Prevent joints from exceeding physical range of motion

**Example** (from URDF):
```xml
<joint name="left_elbow" type="revolute">
  <limit lower="0.0" upper="2.356" effort="10.0" velocity="1.5"/>
  <!-- Elbow can bend 0° to 135° (2.356 rad) -->
</joint>
```

**Damping**: Simulates internal friction in joints (prevents oscillation)

```xml
<joint name="left_shoulder_pitch" type="revolute">
  <dynamics damping="0.1" friction="0.05"/>
  <!-- damping: Opposes velocity (N·m·s/rad) -->
  <!-- friction: Constant resistive torque (N·m) -->
</joint>
```

## Physics Validation Checklist

Before trusting your simulation, verify these properties:

- [ ] **Gravity works**: Robot falls when spawned above ground
- [ ] **Friction prevents sliding**: Robot feet don't slip on flat ground (mu ≥ 0.8)
- [ ] **Collisions are detected**: Robot can't pass through walls/obstacles
- [ ] **Inertia is realistic**: Robot doesn't spin unrealistically when torques applied
- [ ] **Joint limits enforced**: Joints stop at URDF limit values
- [ ] **Real-time factor ≥ 0.8**: Check Gazebo status bar (bottom-right corner)

**How to check real-time factor**:
```bash
gz stats
# Output: real time factor: 0.95 (95% of wall-clock speed)
```

If real-time factor &lt; 0.5 → Simulation is too slow:
- Reduce physics update rate (0.001s → 0.002s)
- Simplify collision meshes
- Reduce number of contact points

## Troubleshooting Common Errors

### Error: "No inertial data for link"

**Cause**: Missing `<inertial>` tag in URDF

**Solution**: Add inertia to every non-fixed link:
```xml
<inertial>
  <mass value="1.0"/>
  <inertia ixx="0.01" ixy="0.0" ixz="0.0"
           iyy="0.01" iyz="0.0" izz="0.01"/>
</inertial>
```

### Error: "Collision mesh is too complex"

**Cause**: Collision geometry has >10,000 triangles

**Solution**: Use simplified collision shapes:
```xml
<!-- GOOD: Simple box collision -->
<collision>
  <geometry>
    <box size="0.5 0.3 0.1"/>
  </geometry>
</collision>

<!-- BAD: High-poly mesh collision -->
<collision>
  <geometry>
    <mesh filename="detailed_model.stl"/>  <!-- 50,000 triangles -->
  </geometry>
</collision>
```

### Error: "Robot vibrates/explodes at startup"

**Cause**: Inter-penetrating collision geometry or stiff contacts

**Solution**:
1. Check links don't overlap in initial pose
2. Reduce contact stiffness (`<kp>`) in Gazebo tags:
   ```xml
   <gazebo reference="link_name">
     <kp>100000.0</kp>  <!-- Lower = softer contacts -->
     <kd>1.0</kd>
   </gazebo>
   ```

## Reality Check: Simulation Limitations

:::caution Sim-to-Real Gap

**What Gazebo gets right**:
- ✅ Gravity, friction (coefficient-based)
- ✅ Rigid body dynamics (Newton-Euler)
- ✅ Joint limits, simple damping

**What Gazebo simplifies**:
- ⚠️ **Material deformation**: Real objects bend/compress; simulation assumes rigid
- ⚠️ **Contact dynamics**: Real contacts have vibration, stick-slip; simulation is smooth
- ⚠️ **Actuator limits**: Real motors have thermal limits, voltage sag; simulation has infinite power

**Example**: A simulated robot can balance on one foot indefinitely if the controller is perfect. A real robot will drift due to:
- Sensor noise (IMU drift: ~1°/minute)
- Actuator backlash (gear play: ~0.1°)
- Floor irregularities (not modeled in flat plane)

**Best Practice**: Add realistic sensor noise (see Section 2.5) and test on hardware with conservative safety margins.

:::

## Section Summary

**Key Concepts**:
1. **Gazebo Classic** = industry-standard physics simulator for ROS 2
2. **Physics fundamentals**: Gravity, friction, mass, inertia, collision detection
3. **URDF → Gazebo pipeline**: robot_state_publisher → /robot_description → spawn_entity.py
4. **Validation**: Use automated scripts to verify gravity, collisions, joint limits

**This Section's Deliverable**: A working Gazebo simulation where:
- Humanoid robot spawns and falls realistically
- Gravity/friction/collisions are validated
- Real-time factor ≥ 0.8 (acceptable performance)

**Next Section**: [2.3 ROS 2 Control Integration](./ros2-control-integration.md) - Learn to control simulated joints using the same interfaces as real hardware.
