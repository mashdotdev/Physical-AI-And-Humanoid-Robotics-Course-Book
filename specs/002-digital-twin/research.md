# Research: Digital Twin Chapter Technology Decisions

**Feature**: 002-digital-twin
**Date**: 2025-12-05
**Purpose**: Resolve NEEDS CLARIFICATION items from Technical Context and establish best practices for Gazebo, Unity, and ROS 2 integration.

---

## Research Questions

### 1. Gazebo Sensor Plugin Versions (gazebo_ros_pkgs)

**Question**: Which version of gazebo_ros_pkgs should we use for sensor simulation (LiDAR, depth camera, IMU)?

**Research Summary**:

For ROS 2 Humble on Ubuntu 22.04 LTS:
- **Gazebo Classic 11.x**: Uses `gazebo_ros_pkgs` from ros-humble-gazebo-ros-pkgs (version 3.7.x)
- **Ignition Gazebo (Fortress/Garden)**: Uses `ros_gz` bridge packages (version 0.244.x for Fortress)

**Decision**: Use **gazebo_ros_pkgs 3.7.x** for Gazebo Classic examples (primary path for beginners)

**Rationale**:
1. Gazebo Classic 11 is stable, well-documented, and has mature sensor plugins
2. Lower system requirements than Ignition (no Ogre2 rendering dependencies)
3. Direct ROS 2 plugin integration without bridge complexity
4. Aligns with Module 2 teaching goal: physics fundamentals before advanced rendering

**Sensor Plugins Selected**:
- LiDAR 2D: `libgazebo_ros_ray_sensor.so` (publishes sensor_msgs/LaserScan)
- LiDAR 3D: `libgazebo_ros_velodyne_laser.so` (publishes sensor_msgs/PointCloud2)
- Depth Camera: `libgazebo_ros_camera.so` with depth=true (publishes sensor_msgs/Image + CameraInfo)
- IMU: `libgazebo_ros_imu_sensor.so` (publishes sensor_msgs/Imu)

**Alternatives Considered**:
- Ignition Gazebo: Rejected for Chapter 2 (too complex, requires ros_gz bridge understanding)
- Isaac Sim sensors: Out of scope (Module 3)

**Code Reference**: URDF sensor blocks will use gazebo_ros_pkgs 3.7.x plugin syntax from http://wiki.ros.org/gazebo_plugins

---

### 2. Unity Physics Engine Configuration for ROS Sync

**Question**: Should Unity use PhysX (default) or custom physics sync for time-critical ROS integration?

**Research Summary**:

Unity offers three physics timing modes:
1. **Fixed Timestep** (default 0.02s = 50 Hz)
2. **Variable Timestep** (matches render frame rate)
3. **Manual Simulation** (advance physics via script)

ROS 2 + Gazebo simulations typically run at:
- Physics: 1000 Hz (1ms timestep)
- Control: 100 Hz (10ms)
- Visualization: 30-60 Hz

**Decision**: Use **Fixed Timestep with Manual /clock subscription**

**Rationale**:
1. Fixed timestep (0.01s = 100 Hz) matches ROS control rate
2. Unity subscribes to Gazebo's `/clock` topic and advances physics in sync
3. PhysX remains default engine (no custom physics solver needed)
4. Rendering runs at native FPS but physics steps are controlled by ROS time

**Implementation Approach**:
```csharp
// Pseudo-code for Unity TimeSync.cs
void OnClockMessage(rosgraph_msgs.Clock msg) {
    float targetTime = msg.clock.sec + msg.clock.nanosec / 1e9f;
    while (Time.fixedTime < targetTime) {
        Physics.Simulate(Time.fixedDeltaTime);
    }
}
```

**Alternatives Considered**:
- Custom physics engine: Rejected (unnecessary complexity, PhysX is production-grade)
- No time sync: Rejected (would cause Gazebo-Unity drift within seconds)
- Unity as primary simulator: Rejected (PhysX not designed for robotics contact dynamics)

**Best Practice**: Unity is visualization layer only. Gazebo is source of truth for physics. Unity mirrors state.

---

### 3. URDF Importer Version and Mesh Conversion Pipeline

**Question**: Which Unity URDF Importer version should we use, and how should meshes be converted for compatibility?

**Research Summary**:

Unity URDF Importer options:
1. **Unity Robotics Hub URDF Importer** (v0.7.0+): Official Unity + ROS integration package
2. **ROS# URDF Importer**: Legacy, unmaintained
3. **Manual import**: Requires custom C# scripting

Mesh conversion workflow:
- ROS meshes: Typically `.stl` or `.dae` (Collada)
- Unity meshes: Requires `.fbx`, `.obj`, or built-in mesh format
- Collision meshes: Unity requires convex decomposition for non-convex shapes

**Decision**: Use **Unity Robotics Hub URDF Importer v0.7.0+** with **simplified collision meshes**

**Rationale**:
1. Officially maintained by Unity (active development, ROS 2 support)
2. Automatic mesh conversion: Converts .stl/.dae → Unity mesh on import
3. Articulation body support: Uses Unity's ArticulationBody (not legacy joints) for accurate kinematics
4. GitHub: https://github.com/Unity-Technologies/URDF-Importer

**Mesh Pipeline Best Practices**:
```text
URDF Authoring (Chapter 1 output)
├── Visual meshes: .stl or .dae (can be high-poly for Unity rendering)
├── Collision meshes: .stl (MUST be <1000 triangles, convex hulls preferred)
└── inertia values: Must be defined (Unity ArticulationBody requires mass/inertia)

Unity Import Process
├── 1. Install com.unity.robotics.urdf-importer via Package Manager
├── 2. Import URDF via Assets > Import Robot from URDF
├── 3. Auto-conversion: .stl/.dae → Unity Mesh
├── 4. Collision simplification: Unity MeshCollider with Convex=true
└── 5. Material assignment: Default materials → Custom PBR materials

Validation
├── Check ArticulationBody mass matches URDF <inertial> tags
├── Verify collision hulls are convex (non-convex causes physics instability)
└── Test joint limits match URDF <limit> tags
```

**Alternatives Considered**:
- Manual Blender export: Rejected (adds tooling dependency, error-prone)
- ROS#: Rejected (unmaintained since 2020)
- Isaac Sim USD converter: Out of scope for Chapter 2 (Module 3 topic)

**Tooling for Mesh Simplification** (optional, for advanced readers):
- Blender + Decimate modifier
- MeshLab (open-source, cross-platform)
- Mention in "Reality Check" callout: "If collision meshes are too complex, use Blender's Decimate modifier to reduce to <1000 triangles"

---

### 4. ROS-Unity Bridge: ROS TCP Connector Configuration

**Question**: What is the correct setup for ROS TCP Connector to ensure low-latency joint state updates?

**Research Summary**:

ROS-Unity communication options:
1. **ROS TCP Connector** (Unity Robotics Hub, official)
2. **ros2-web-bridge** (WebSocket-based, higher latency)
3. **ros1_bridge + rosbridge_suite** (ROS 1 legacy)

ROS TCP Connector architecture:
- Unity side: Runs TCP client, connects to ROS TCP Endpoint server
- ROS side: Runs `ros_tcp_endpoint` node (Python), bridges TCP ↔ ROS topics
- Protocol: Custom binary format, optimized for low latency (~5-10ms typical)

**Decision**: Use **ROS TCP Connector (Unity Robotics Hub)** with optimized topic configuration

**Rationale**:
1. Official Unity + ROS 2 solution (maintained, documented)
2. Lower latency than WebSocket-based bridges (~5ms vs ~50ms)
3. Supports both publishers and subscribers in Unity
4. Automatically handles message serialization (geometry_msgs, sensor_msgs, etc.)

**Configuration Best Practices**:

ROS side (Ubuntu 22.04):
```bash
# Install ROS TCP Endpoint
sudo apt install ros-humble-ros-tcp-endpoint

# Launch endpoint node
ros2 launch ros_tcp_endpoint endpoint.launch.py tcp_ip:=0.0.0.0 tcp_port:=10000
```

Unity side (2021.3 LTS+):
```text
1. Install packages:
   - com.unity.robotics.ros-tcp-connector
   - com.unity.robotics.urdf-importer

2. ROSConnectionSettings.asset:
   - ROS IP Address: 192.168.1.100 (or localhost if same machine)
   - ROS Port: 10000
   - Protocol: ROS 2

3. Topic subscriptions (Unity C#):
   - /joint_states (sensor_msgs/JointState) → Update Unity ArticulationBody positions
   - /clock (rosgraph_msgs/Clock) → Sync simulation time

4. Topic publishers (Unity C#):
   - /joint_commands (std_msgs/Float64MultiArray) → Test manual joint control
   - /camera/image_raw (sensor_msgs/Image) → Unity camera → ROS visualization
```

**Performance Targets**:
- Joint state update rate: 100 Hz (10ms period)
- ROS → Unity latency: <10ms (measured via timestamp comparison)
- Unity → ROS latency: <10ms
- Total round-trip: <20ms (acceptable for visualization; Gazebo is control authority)

**Alternatives Considered**:
- ros2-web-bridge: Rejected (50ms latency too high for smooth visualization)
- Direct DDS binding: Rejected (complex setup, no Unity support out-of-box)
- Custom TCP protocol: Rejected (reinventing wheel)

**Debugging Tips for Chapter**:
- Use `ros2 topic hz /joint_states` to verify 100 Hz publication
- Use Unity's Timeline window to visualize frame drops
- Monitor network traffic with `iftop` if using separate machines

---

### 5. Gazebo World File Best Practices

**Question**: What should be included in the example world files for Chapter 2?

**Research Summary**:

Gazebo world files (`.world`) define:
- Physics engine parameters (gravity, solver iterations, time step)
- Scene elements (ground plane, lighting, obstacles)
- Plugin configurations (sensors, controllers)

For educational purposes, world files should:
1. Be minimal (avoid clutter that obscures learning goals)
2. Include comments explaining each parameter
3. Provide incremental complexity (empty → obstacles → full environment)

**Decision**: Provide **3 world files** with increasing complexity

**World File Structure**:

**File 1: `empty_world.world`** (Section 2.2: Physics Fundamentals)
```xml
<!-- Minimal world: gravity, ground plane, basic lighting -->
<!-- Use case: Test robot spawning, gravity toggle, basic movement -->
<!-- Physics: ODE solver, 1ms timestep, gravity=-9.81 -->
```

**File 2: `test_environment.world`** (Section 2.5: Sensor Simulation)
```xml
<!-- Add: walls, boxes, cylinders for obstacle detection -->
<!-- Use case: LiDAR scanning, collision testing, navigation experiments -->
<!-- Includes: Static obstacles at known positions for sensor validation -->
```

**File 3: `human_interaction_scene.world`** (Section 2.6: Full Pipeline)
```xml
<!-- Add: Human actor model, furniture, realistic lighting -->
<!-- Use case: Human-robot interaction, reachability testing, safety zones -->
<!-- Gazebo actors: Walking human with scripted path -->
```

**Physics Parameters** (consistent across all files):
```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>        <!-- 1ms = 1000 Hz -->
  <real_time_factor>1.0</real_time_factor>    <!-- Target real-time -->
  <real_time_update_rate>1000</real_time_update_rate>
  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>                       <!-- Balance speed/accuracy -->
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

**Rationale**:
- 1ms timestep: Industry standard for humanoid robots (fast enough for 100 Hz control)
- ODE solver: Default in Gazebo Classic, well-tested
- 50 iterations: Good balance (lower = faster but less stable, higher = slower but more accurate)

**Alternatives Considered**:
- Single complex world: Rejected (overwhelming for Section 2.2 beginners)
- Isaac Sim world files: Out of scope (Module 3)

---

### 6. Launch File Architecture

**Question**: How should launch files be structured for the "single-command digital twin pipeline"?

**Research Summary**:

ROS 2 launch files can:
- Start multiple nodes (Python)
- Include other launch files (composition)
- Set parameters from YAML config
- Conditionally launch nodes based on arguments

For Chapter 2 capstone (Section 2.6), readers need a single command that starts:
1. Gazebo physics simulation
2. ROS 2 control nodes (joint state publisher, controllers)
3. ROS TCP Endpoint (for Unity bridge)
4. Sensor publishers (LiDAR, depth camera, IMU)
5. RViz visualization (optional)

**Decision**: Use **hierarchical launch file architecture** with `digital_twin_complete.launch.py` as entry point

**Launch File Hierarchy**:

```text
digital_twin_complete.launch.py (ENTRY POINT)
├── Include: gazebo_sim.launch.py
│   ├── Start: gzserver (Gazebo physics server)
│   ├── Start: gzclient (Gazebo GUI client)
│   └── Spawn: robot_state_publisher + spawn_entity.py
│
├── Include: ros2_control.launch.py
│   ├── Load: controller_manager
│   ├── Load: joint_state_broadcaster
│   └── Load: position_controller (for all joints)
│
├── Include: unity_bridge.launch.py
│   └── Start: ros_tcp_endpoint (port 10000)
│
├── Include: sensor_publishers.launch.py
│   ├── Start: lidar_republisher (if needed)
│   ├── Start: depth_camera_processor
│   └── Start: imu_filter (optional, for noise reduction)
│
└── Include: rviz.launch.py (OPTIONAL)
    └── Start: rviz2 with config file
```

**Entry Point Example** (`digital_twin_complete.launch.py`):
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # 1. Start Gazebo simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('digital_twin_chapter2'),
                    'launch',
                    'gazebo_sim.launch.py'
                ])
            ])
        ),

        # 2. Start ROS 2 control
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('digital_twin_chapter2'),
                    'launch',
                    'ros2_control.launch.py'
                ])
            ])
        ),

        # 3. Start Unity bridge (readers launch Unity manually)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('digital_twin_chapter2'),
                    'launch',
                    'unity_bridge.launch.py'
                ])
            ])
        ),

        # 4. Start sensor processing
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('digital_twin_chapter2'),
                    'launch',
                    'sensor_publishers.launch.py'
                ])
            ])
        ),
    ])
```

**Rationale**:
- Modular: Each subsystem in separate file (readers can test incrementally)
- Composable: Sections 2.2-2.5 use individual launch files; Section 2.6 combines all
- Standard ROS 2 Python launch: No XML roslaunch legacy
- Reader-friendly: Single command `ros2 launch digital_twin_chapter2 digital_twin_complete.launch.py`

**Alternatives Considered**:
- Monolithic launch file: Rejected (hard to debug, not educational)
- Shell script wrapper: Rejected (not idiomatic ROS 2)

---

## Technology Stack Summary

**Finalized Dependencies** (no more NEEDS CLARIFICATION):

| Component | Version/Package | Purpose |
|-----------|----------------|---------|
| ROS 2 | Humble (Ubuntu 22.04) | Middleware, control framework |
| Gazebo | Classic 11.x | Physics simulation |
| gazebo_ros_pkgs | 3.7.x (ros-humble-gazebo-ros-pkgs) | Sensor plugins, ros2_control integration |
| Unity | 2021.3 LTS or newer | High-fidelity rendering |
| Unity URDF Importer | v0.7.0+ (com.unity.robotics.urdf-importer) | URDF → Unity conversion |
| ROS TCP Connector | Latest (com.unity.robotics.ros-tcp-connector) | Unity ↔ ROS communication |
| ros_tcp_endpoint | ros-humble-ros-tcp-endpoint | ROS side of Unity bridge |
| ros2_control | Included in Humble | Joint controllers |
| RViz | Included in Humble | Visualization |

---

## Best Practices Captured

### For Chapter Content

1. **Incremental Complexity**: Start with empty world (Section 2.2), add obstacles (Section 2.5), add humans (Section 2.6)

2. **Reality Check Callouts**: Every section must include:
   - Simulation vs hardware differences (e.g., "Real IMUs drift ~10°/hour, simulated IMUs are perfect unless noise is added")
   - Performance expectations (e.g., "Gazebo with 20-DOF humanoid: expect 0.5x-1.0x real-time factor on mid-range GPU")

3. **Code Comments**: All code examples include:
   - Purpose statement at top
   - Parameter explanations inline
   - Expected output/behavior
   - Troubleshooting tips (e.g., "If joints don't move, check ros2 topic list /joint_states")

4. **Validation Steps**: Each section ends with:
   - "How to verify this works" checklist
   - Common failure modes and solutions
   - Screenshot showing expected result

### For Code Examples

1. **URDF Best Practices**:
   - Always define <inertial> tags (Gazebo defaults to 1kg cube if missing)
   - Collision meshes: convex hulls, <1000 triangles
   - Visual meshes: can be high-poly (Unity handles this)
   - Use xacro for parameterization (avoid copy-paste)

2. **Launch File Best Practices**:
   - Use Python launch (not XML)
   - Parameterize paths with FindPackageShare
   - Include optional nodes with DeclareLaunchArgument

3. **Unity Script Best Practices**:
   - Subscribe to /clock for time sync
   - Use ArticulationBody (not legacy joints)
   - Update physics in FixedUpdate(), rendering in Update()

---

## Integration with Isaac Sim (Forward Compatibility)

**Bridge to Module 3**: Section 2.1 will include explicit statement:

> **Looking Ahead**: This chapter teaches digital twin fundamentals using Gazebo and Unity. In Module 3 (Chapters 8-10), you will transition to NVIDIA Isaac Sim, which combines:
> - GPU-accelerated physics (PhysX 5)
> - Photorealistic rendering (RTX ray tracing)
> - Integrated sensor simulation (Isaac Sensor suite)
> - URDF/USD compatibility
>
> The concepts you learn here (URDF structure, physics parameters, ROS control, sensor configuration) **transfer directly** to Isaac Sim. The primary difference: Isaac Sim uses USD (Universal Scene Description) format instead of Gazebo .world files, and sensors are higher-fidelity (e.g., Isaac LiDAR simulates ray-tracing for reflections).

**Migration Path** (mentioned in Section 2.6 Summary):
- Gazebo world → Isaac Sim USD stage
- Gazebo sensor plugins → Isaac Sensor API
- ROS TCP Connector → Isaac ROS bridge (native DDS)
- Unity rendering → Isaac Sim Omniverse rendering

---

## Diagram Requirements

Based on research, the following diagrams are required:

1. **digital-twin-architecture.svg** (Section 2.1):
   - Shows: Real Robot ↔ Digital Twin ↔ Control Logic
   - Annotations: Safety, iteration speed, cost savings

2. **gazebo-ros-dataflow.svg** (Section 2.2):
   - Shows: Gazebo Physics → ros2_control → ROS Topics → RViz
   - Includes: /joint_states, /joint_commands, /clock

3. **unity-sync-diagram.svg** (Section 2.3):
   - Shows: Gazebo → ROS TCP Endpoint → Unity
   - Timeline: /clock sync, joint state updates at 100 Hz

4. **sensor-tf-tree.svg** (Section 2.5):
   - Shows: base_link → lidar_link, camera_link, imu_link
   - Transformations: Translation + rotation for each sensor

5. **full-pipeline-flowchart.svg** (Section 2.6):
   - Shows: URDF → Gazebo Physics → ROS Control → Unity Visualization → Sensors → (future) Agent
   - End-to-end data flow from robot definition to behavior

---

## Validation Criteria

All research decisions must pass:

✅ **Testability**: Can readers verify this works in <10 minutes?
✅ **Hardware Access**: Does this require only Ubuntu 22.04 + mid-range GPU?
✅ **Forward Compatibility**: Does this knowledge transfer to Isaac Sim?
✅ **Industry Standard**: Do production robotics companies use this approach?
✅ **ROS 2 Native**: No ROS 1 legacy, no workarounds

All research items: PASSED