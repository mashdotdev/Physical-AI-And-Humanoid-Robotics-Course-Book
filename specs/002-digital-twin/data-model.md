# Data Model: Digital Twin Chapter Entities

**Feature**: 002-digital-twin
**Date**: 2025-12-05
**Purpose**: Define the structure and relationships of key entities in the digital twin system for educational content.

---

## Overview

This is an **educational/documentation project**, not a traditional software application. The "data model" describes the **conceptual entities** that readers will work with when building digital twins. These entities are represented as:

- **Configuration files** (URDF XML, YAML, JSON)
- **ROS 2 topics/messages** (runtime data streams)
- **Unity scene objects** (GameObjects with components)
- **Coordinate frames** (TF tree transformations)

---

## Entity Definitions

### 1. URDF Model (Robot Description)

**Type**: Configuration File (XML)
**File Format**: `.urdf` or `.urdf.xacro`
**Scope**: Single robot definition
**Lifespan**: Design-time artifact, loaded at simulation startup

**Structure**:

```text
URDFModel
├── robot (root element)
│   ├── name: string (e.g., "humanoid_robot")
│   ├── links: Link[] (1 to many)
│   ├── joints: Joint[] (0 to many)
│   └── gazebo_plugins: Plugin[] (0 to many, Gazebo-specific)
```

**Link Entity**:
```text
Link
├── name: string (e.g., "torso", "right_arm")
├── visual: Visual (optional)
│   ├── geometry: Mesh | Box | Cylinder | Sphere
│   │   └── mesh.filename: string (e.g., "package://robot_description/meshes/torso.stl")
│   └── material: Material (color or texture)
├── collision: Collision (required for physics)
│   └── geometry: Mesh | Primitive (MUST be simplified: <1000 triangles)
└── inertial: Inertial (required for physics)
    ├── mass: float (kg)
    ├── inertia: Matrix3x3 (ixx, ixy, ixz, iyy, iyz, izz)
    └── origin: xyz, rpy (center of mass offset)
```

**Joint Entity**:
```text
Joint
├── name: string (e.g., "shoulder_pitch_joint")
├── type: "revolute" | "prismatic" | "continuous" | "fixed"
├── parent: Link.name (reference)
├── child: Link.name (reference)
├── origin: xyz, rpy (joint frame relative to parent)
├── axis: xyz (rotation/translation axis)
└── limit: (for revolute/prismatic only)
    ├── lower: float (radians or meters)
    ├── upper: float (radians or meters)
    ├── effort: float (max torque/force)
    └── velocity: float (max rad/s or m/s)
```

**Gazebo Plugin Entity** (extends URDF for simulation):
```text
GazeboPlugin
├── plugin_name: string (e.g., "gazebo_ros2_control")
├── plugin_filename: string (e.g., "libgazebo_ros2_control.so")
└── parameters: key-value map (e.g., robot_param: "robot_description")
```

**Relationships**:
- Link → Joint: parent/child relationship defines kinematic tree
- URDF Model → ROS 2: Published to `/robot_description` parameter
- URDF Model → Gazebo: Loaded via `spawn_entity.py` script
- URDF Model → Unity: Imported via URDF Importer package

**Validation Rules**:
- Every Link (except base_link) must have exactly 1 parent Joint
- base_link has no parent (root of kinematic tree)
- Joint limits: lower < upper
- Inertial mass > 0
- Collision geometry must be convex or decomposable

**State Transitions**: N/A (static configuration)

---

### 2. Physics Engine State (Gazebo)

**Type**: Runtime Simulation State
**Scope**: Per-simulation instance
**Lifespan**: Simulation runtime (ephemeral)

**Structure**:

```text
PhysicsEngineState
├── world: World
│   ├── name: string (e.g., "empty_world")
│   ├── physics: PhysicsConfig
│   │   ├── type: "ode" | "bullet" | "dart"
│   │   ├── max_step_size: float (seconds, e.g., 0.001)
│   │   ├── real_time_factor: float (target, e.g., 1.0)
│   │   └── gravity: Vector3 (m/s², e.g., [0, 0, -9.81])
│   ├── models: Model[] (spawned robots, obstacles)
│   └── time: SimulationTime
│       ├── sim_time: float (seconds since simulation start)
│       └── real_time: float (wall-clock time)
│
└── per_model_state: ModelState[]
    ├── model_name: string
    ├── links: LinkState[]
    │   ├── link_name: string
    │   ├── pose: Pose (position + orientation)
    │   ├── linear_velocity: Vector3
    │   ├── angular_velocity: Vector3
    │   └── contact_forces: Force[] (if in collision)
    └── joints: JointState[]
        ├── joint_name: string
        ├── position: float (radians or meters)
        ├── velocity: float (rad/s or m/s)
        └── effort: float (N·m or N)
```

**Relationships**:
- PhysicsEngineState → ROS 2: Publishes to `/clock`, `/joint_states`, `/tf`
- PhysicsEngineState → Unity: Indirectly via ROS topics (Unity mirrors state)

**Validation Rules**:
- max_step_size ≤ 0.001s (1ms) for humanoid robots (stability)
- real_time_factor: 0.5-1.5 acceptable range (below 0.5 = too slow to be useful)
- Joint positions must respect URDF limits

**State Transitions**:
1. **Paused → Running**: Simulation starts advancing time
2. **Running → Paused**: User press Gazebo pause button
3. **Running → Reset**: Reload world file, reset all state to t=0

---

### 3. ROS 2 Control Framework State

**Type**: Runtime Control State
**Scope**: Per-robot instance
**Lifespan**: While ros2_control nodes are running

**Structure**:

```text
ROS2ControlFramework
├── controller_manager: ControllerManager
│   ├── robot_description: URDF (loaded from parameter server)
│   ├── hardware_interfaces: HardwareInterface[]
│   │   └── gazebo_system: GazeboSystemInterface (bridges Gazebo ↔ ros2_control)
│   └── controllers: Controller[]
│       ├── joint_state_broadcaster: Controller
│       │   ├── type: "joint_state_broadcaster/JointStateBroadcaster"
│       │   └── publishes_to: /joint_states (sensor_msgs/JointState)
│       ├── position_controller: Controller
│       │   ├── type: "position_controllers/JointGroupPositionController"
│       │   ├── joints: string[] (list of joint names)
│       │   ├── subscribes_to: /position_controller/commands (std_msgs/Float64MultiArray)
│       │   └── control_mode: "position"
│       └── [additional controllers: velocity, effort, trajectory]
│
└── control_loop: ControlLoop
    ├── update_rate: float (Hz, e.g., 100)
    ├── current_command: Float64MultiArray (target positions/velocities/efforts)
    └── feedback: JointState (current positions/velocities/efforts from Gazebo)
```

**Relationships**:
- ROS2ControlFramework → URDF Model: Loads joint names and limits from URDF
- ROS2ControlFramework → PhysicsEngineState: Reads joint state, writes joint commands
- ROS2ControlFramework → User Code: User publishes to `/position_controller/commands`

**Validation Rules**:
- update_rate ≥ 50 Hz (minimum for smooth control)
- Command array length = number of controlled joints
- Commands must respect joint limits (enforced by controller)

**State Transitions**:
1. **Inactive → Active**: `ros2 control load_controller` + `ros2 control set_controller_state <name> start`
2. **Active → Inactive**: `ros2 control set_controller_state <name> stop`

---

### 4. Rendering Engine State (Unity)

**Type**: Visualization Runtime State
**Scope**: Unity scene instance
**Lifespan**: Unity application runtime

**Structure**:

```text
UnityRenderingEngine
├── scene: Scene
│   ├── name: string (e.g., "HumanoidSimulation")
│   ├── game_objects: GameObject[]
│   │   ├── RobotModel: GameObject (root)
│   │   │   ├── transform: Transform (position, rotation, scale)
│   │   │   ├── articulation_body: ArticulationBody (Unity's advanced physics)
│   │   │   │   ├── mass: float (from URDF inertial)
│   │   │   │   ├── inertia_tensor: Vector3 (from URDF inertial)
│   │   │   │   └── joints: ArticulationJoint[] (from URDF joints)
│   │   │   └── children: GameObject[] (links as nested objects)
│   │   ├── HumanCharacter: GameObject (for HRI testing)
│   │   ├── Environment: GameObject (walls, floor, furniture)
│   │   └── Lighting: GameObject (directional light, point lights)
│   └── cameras: Camera[]
│       ├── MainCamera: Camera (player view)
│       └── RobotCamera: Camera (simulated depth camera, optional)
│
├── ros_connection: ROSConnection
│   ├── ros_ip: string (e.g., "192.168.1.100")
│   ├── ros_port: int (e.g., 10000)
│   ├── subscriptions: Subscription[]
│   │   ├── /joint_states → OnJointStateReceived(JointState msg)
│   │   └── /clock → OnClockReceived(Clock msg)
│   └── publishers: Publisher[]
│       └── /unity/camera/image → Publish(Image msg)
│
└── sync_state: SyncState
    ├── last_clock_time: float (from /clock topic)
    ├── physics_step_count: int
    └── render_fps: float (Unity framerate, 30-60 Hz)
```

**Relationships**:
- UnityRenderingEngine → ROS 2: Subscribes to `/joint_states`, `/clock`
- UnityRenderingEngine → URDF Model: Imported via URDF Importer (design-time)
- UnityRenderingEngine → PhysicsEngineState: Mirrors Gazebo state (not authoritative)

**Validation Rules**:
- ArticulationBody joint count = URDF joint count
- ArticulationBody mass/inertia match URDF inertial tags
- ROS connection established before simulation start

**State Transitions**:
1. **Disconnected → Connected**: ROSConnection establishes TCP connection to ros_tcp_endpoint
2. **Connected → Synchronized**: Receiving `/clock` and `/joint_states` at expected rate
3. **Synchronized → Desynchronized**: No `/clock` messages for >1 second (timeout)

---

### 5. Sensor Models (Simulation)

**Type**: Configuration + Runtime Data Stream
**Scope**: Per-sensor instance
**Lifespan**: Simulation runtime

**Structure**:

```text
SensorModel (Abstract Base)
├── sensor_name: string (e.g., "front_lidar")
├── sensor_type: "lidar_2d" | "lidar_3d" | "depth_camera" | "imu"
├── parent_link: string (URDF link where sensor is attached)
├── sensor_frame: string (TF frame, e.g., "lidar_link")
├── update_rate: float (Hz)
├── ros_topic: string (where data is published)
└── noise_model: NoiseModel (optional)
    ├── type: "gaussian" | "uniform"
    ├── mean: float
    └── stddev: float

LiDAR2D extends SensorModel
├── min_angle: float (radians, e.g., -π)
├── max_angle: float (radians, e.g., +π)
├── angle_increment: float (radians, e.g., 0.01)
├── range_min: float (meters, e.g., 0.1)
├── range_max: float (meters, e.g., 30.0)
└── publishes: sensor_msgs/LaserScan
    ├── ranges: float[] (distance measurements)
    └── intensities: float[] (optional)

LiDAR3D extends SensorModel
├── channels: int (vertical slices, e.g., 16, 32, 64)
├── horizontal_resolution: float (degrees, e.g., 0.1)
├── vertical_fov: float (degrees, e.g., 30)
├── range_min: float (meters)
├── range_max: float (meters)
└── publishes: sensor_msgs/PointCloud2
    ├── points: Point[] (x, y, z in sensor frame)
    └── intensities: float[] (optional)

DepthCamera extends SensorModel
├── image_width: int (pixels, e.g., 640)
├── image_height: int (pixels, e.g., 480)
├── horizontal_fov: float (radians, e.g., 1.047 = 60°)
├── near_clip: float (meters, e.g., 0.1)
├── far_clip: float (meters, e.g., 10.0)
└── publishes: sensor_msgs/Image + sensor_msgs/CameraInfo
    ├── image/rgb: Image (RGB channels)
    ├── image/depth: Image (32FC1, depth in meters)
    └── camera_info: CameraInfo (intrinsic parameters)

IMU extends SensorModel
├── linear_acceleration_noise: NoiseModel
├── angular_velocity_noise: NoiseModel
├── orientation_noise: NoiseModel
└── publishes: sensor_msgs/Imu
    ├── orientation: Quaternion (x, y, z, w)
    ├── angular_velocity: Vector3 (rad/s)
    └── linear_acceleration: Vector3 (m/s²)
```

**Relationships**:
- SensorModel → URDF Model: Sensor defined in `<gazebo>` tags within URDF
- SensorModel → TF Tree: Sensor frame published to `/tf` (transform from base_link)
- SensorModel → ROS 2: Publishes sensor data to topics
- SensorModel → PhysicsEngineState: Ray-tracing queries Gazebo collision geometry

**Validation Rules**:
- update_rate: 10-100 Hz typical (higher = more CPU load)
- LiDAR range_max ≤ 100m (realistic limit for most hardware)
- Depth camera resolution: 640x480 or 1280x720 (higher = slower)
- IMU noise: stddev should match real hardware specs (e.g., ±0.01 m/s² for accelerometer)

**State Transitions**:
1. **Inactive → Active**: Gazebo plugin initializes, starts publishing
2. **Active → Paused**: Simulation paused, sensor stops publishing
3. **Active → Failed**: Sensor plugin crashes (rare, usually URDF misconfiguration)

---

### 6. Launch Configuration (System Orchestration)

**Type**: Configuration File (Python Launch Description)
**File Format**: `.launch.py`
**Scope**: System-level (coordinates multiple nodes/processes)
**Lifespan**: Design-time artifact, executed at startup

**Structure**:

```text
LaunchConfiguration
├── name: string (e.g., "digital_twin_complete")
├── arguments: LaunchArgument[]
│   ├── use_sim_time: bool (default: true)
│   ├── world_file: string (default: "empty_world.world")
│   └── rviz_config: string (default: "default.rviz")
│
├── nodes: Node[]
│   ├── gzserver: Node (Gazebo physics server)
│   │   ├── package: "gazebo_ros"
│   │   ├── executable: "gzserver"
│   │   └── arguments: [world_file]
│   ├── gzclient: Node (Gazebo GUI)
│   ├── robot_state_publisher: Node
│   │   └── parameters: [robot_description]
│   ├── spawn_entity: Node
│   │   └── arguments: ["-topic", "robot_description", "-entity", "robot"]
│   ├── controller_manager: Node (ros2_control)
│   ├── ros_tcp_endpoint: Node (Unity bridge)
│   └── rviz2: Node (visualization)
│
└── included_launches: LaunchDescription[]
    ├── gazebo_sim.launch.py
    ├── ros2_control.launch.py
    ├── unity_bridge.launch.py
    └── sensor_publishers.launch.py
```

**Relationships**:
- LaunchConfiguration → All Entities: Starts and configures entire system
- LaunchConfiguration → ROS 2 Parameter Server: Sets `/use_sim_time`, `/robot_description`

**Validation Rules**:
- All file paths must be absolute or use `FindPackageShare` substitution
- Nodes must have unique names within namespace
- Launch arguments must have default values (for ease of use)

**State Transitions**:
1. **Idle → Launching**: `ros2 launch` command executed
2. **Launching → Running**: All nodes successfully started
3. **Running → Shutdown**: User Ctrl+C or launch file completes

---

### 7. TF Tree (Coordinate Frame Graph)

**Type**: Runtime Data Structure (Graph)
**Scope**: System-wide (all nodes can query)
**Lifespan**: Simulation runtime

**Structure**:

```text
TFTree (Directed Acyclic Graph)
├── frames: Frame[]
│   ├── base_link (ROOT)
│   │   ├── transform_from_parent: N/A (root has no parent)
│   │   └── children: [torso, left_leg, right_leg]
│   ├── torso
│   │   ├── transform_from_parent: Transform (x, y, z, qx, qy, qz, qw)
│   │   └── children: [lidar_link, camera_link, imu_link]
│   ├── lidar_link
│   │   ├── transform_from_parent: Transform
│   │   └── children: []
│   ├── camera_link
│   ├── imu_link
│   └── [additional links from URDF]
│
└── transforms: Transform[]
    ├── parent_frame: string
    ├── child_frame: string
    ├── translation: Vector3 (x, y, z in meters)
    ├── rotation: Quaternion (qx, qy, qz, qw)
    └── timestamp: Time (when transform was published)
```

**Relationships**:
- TFTree → URDF Model: Frame structure mirrors URDF link hierarchy
- TFTree → PhysicsEngineState: Gazebo publishes joint transforms to `/tf`
- TFTree → SensorModels: Sensor frames attached to robot links
- TFTree → ROS 2: Published to `/tf` and `/tf_static` topics

**Validation Rules**:
- No cycles allowed (DAG property)
- All frames must have path to root (base_link)
- Timestamps must be monotonically increasing (or using sim time)
- Quaternions must be unit quaternions (norm = 1)

**State Transitions**: N/A (continuously updated as robot moves)

---

## Entity Relationship Diagram (Conceptual)

```text
┌─────────────────┐
│  URDF Model     │ (design-time)
└────────┬────────┘
         │ loaded by
         ├──────────────────────────────────┐
         │                                  │
         ▼                                  ▼
┌────────────────────┐            ┌─────────────────┐
│ Physics Engine     │◄───sync────┤ Unity Rendering │
│ (Gazebo)           │            │ Engine          │
└────────┬───────────┘            └─────────────────┘
         │ publishes
         ▼
┌────────────────────┐
│ ROS 2 Topics       │
│ (/joint_states,    │
│  /clock, /tf)      │
└────────┬───────────┘
         │ consumed by
         ├──────────────────────┬─────────────────┐
         ▼                      ▼                 ▼
┌────────────────┐   ┌────────────────┐  ┌──────────────┐
│ ROS 2 Control  │   │ Sensor Models  │  │   RViz       │
│ Framework      │   │ (LiDAR, IMU)   │  │ (Visualization)
└────────────────┘   └────────────────┘  └──────────────┘
         │                      │
         │ controlled by        │ visualized in
         ▼                      ▼
┌────────────────────────────────────────┐
│       User Code / Launch Files         │
│  (orchestrates entire digital twin)    │
└────────────────────────────────────────┘
```

---

## Data Flow Example: Joint Command Execution

**Scenario**: User wants to move robot's arm to a specific position.

**Flow**:

1. **User Code** → publishes to `/position_controller/commands` (std_msgs/Float64MultiArray)
   - Data: `[shoulder_pitch: 0.5, elbow: -1.0, wrist: 0.3]` (radians)

2. **ROS 2 Control Framework** → receives command, applies joint limits, sends to Gazebo
   - Validates: All values within URDF `<limit>` tags
   - Sends: JointEffort commands to Gazebo via hardware interface

3. **Physics Engine (Gazebo)** → applies torques to joints, simulates motion
   - Physics: PID controller drives joints toward target positions
   - Collision: Detects if arm hits obstacles, applies contact forces

4. **Gazebo** → publishes updated state to ROS topics
   - `/joint_states`: New positions, velocities, efforts (100 Hz)
   - `/tf`: Updated transformations for all links (100 Hz)
   - `/clock`: Simulation time (1000 Hz)

5. **Unity** → subscribes to `/joint_states` and `/clock`
   - Updates: ArticulationBody joint positions to match Gazebo
   - Syncs: Physics simulation step to match `/clock` timestamp
   - Renders: Arm moving smoothly in 3D scene (30-60 FPS)

6. **RViz** → subscribes to `/tf` and `/joint_states`
   - Displays: Robot model with updated link positions
   - Overlays: Sensor data (LiDAR points, camera image)

**Round-trip latency**: ~20-50ms (Gazebo → ROS → Unity → rendered frame)

---

## Data Persistence

**What is saved to disk**:
- URDF models (`.urdf`, `.urdf.xacro`)
- Gazebo world files (`.world`)
- Launch files (`.launch.py`)
- Configuration files (`.yaml`)
- Unity scenes (`.unity`)
- Unity scripts (`.cs`)

**What is ephemeral** (lost after shutdown):
- Physics engine state (joint positions, velocities)
- ROS 2 topic data (unless recorded with `ros2 bag record`)
- TF tree (reconstructed from URDF + joint states each run)

**Data Recording** (optional, for advanced readers):
- `ros2 bag record /joint_states /tf /lidar/scan` → saves to `.db3` file
- Can replay: `ros2 bag play <file>.db3`
- Use case: Record real robot data, replay in simulation for debugging

---

## Validation Checklist (for Chapter Content)

When writing chapter sections, ensure readers can:

✅ **URDF Model**: Verify with `check_urdf` command-line tool
✅ **Physics Engine**: Observe realistic gravity, collision detection in Gazebo GUI
✅ **ROS 2 Control**: Check `ros2 topic list`, `ros2 topic hz /joint_states`
✅ **Unity Rendering**: See robot moving in Unity, confirm sync with Gazebo
✅ **Sensor Models**: Visualize sensor data in RViz (`ros2 run rviz2 rviz2`)
✅ **Launch Configuration**: Single command starts entire pipeline without errors
✅ **TF Tree**: Run `ros2 run tf2_tools view_frames` → generates PDF graph

---

## Notes for Implementation (Task Generation)

When `/sp.tasks` generates tasks from this data model:

1. **Section 2.2 Tasks**: Focus on URDF Model + Physics Engine entities
   - Task: "Create URDF with 1 link + 1 joint, verify in Gazebo"
   - Task: "Add inertial properties, test gravity response"

2. **Section 2.3 Tasks**: Focus on ROS 2 Control Framework entity
   - Task: "Load position controller, publish command, verify joint moves"

3. **Section 2.4 Tasks**: Focus on Unity Rendering Engine entity
   - Task: "Import URDF to Unity, configure ROS connection, sync joint states"

4. **Section 2.5 Tasks**: Focus on Sensor Models entities
   - Task: "Add LiDAR sensor to URDF, visualize /scan in RViz"

5. **Section 2.6 Tasks**: Focus on Launch Configuration entity
   - Task: "Create complete launch file that starts all entities simultaneously"

Each task should reference specific entity properties and validation rules from this document.