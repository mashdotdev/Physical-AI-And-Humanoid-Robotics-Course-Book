# Data Model: The AI-Robot Brain (NVIDIA Isaac™)

**Date**: 2025-12-05
**Feature**: 003-ai-robot-brain
**Purpose**: Define data structures, ROS message types, and file formats for Chapter 3 content

## Overview

This chapter deals primarily with **data flows** and **configuration structures** rather than traditional application data models. The "entities" are ROS 2 messages, sensor data formats, configuration files, and map representations.

---

## Entity 1: Perception Pipeline Data

### Purpose
Represents the flow of sensor data through processing stages to produce semantic understanding of the environment.

### Data Flow Stages

```
[Raw Sensors] → [Preprocessing] → [Feature Extraction] → [Semantic Understanding]
```

### Stage 1: Raw Sensor Data

**RGB Camera Images** (ROS 2 Message: `sensor_msgs/Image`)
```yaml
Fields:
  header:
    stamp: Time              # Acquisition timestamp
    frame_id: string         # Camera frame (e.g., "camera_left")
  height: uint32             # Image height (480px)
  width: uint32              # Image width (640px)
  encoding: string           # "rgb8" or "bgr8"
  is_bigendian: uint8
  step: uint32               # Row stride in bytes
  data: uint8[]              # Raw pixel data

Validation Rules:
  - width >= 320 and height >= 240 (minimum resolution)
  - encoding must be "rgb8" or "bgr8" (color images required)
  - timestamp must be monotonically increasing
```

**Depth Camera Data** (ROS 2 Message: `sensor_msgs/Image`)
```yaml
Fields:
  [Same header structure as RGB]
  encoding: string           # "32FC1" (32-bit float depth in meters)
  data: float32[]            # Depth values (meters)

Validation Rules:
  - encoding must be "32FC1" or "16UC1"
  - depth values in range [0.1, 10.0] meters (typical indoor range)
  - Synchronized with RGB via matching timestamps
```

**IMU Data** (ROS 2 Message: `sensor_msgs/Imu`)
```yaml
Fields:
  header:
    stamp: Time
    frame_id: string         # IMU frame (e.g., "imu_link")
  orientation:               # Quaternion (x, y, z, w)
    x, y, z, w: float64
  angular_velocity:          # rad/s
    x, y, z: float64
  linear_acceleration:       # m/s²
    x, y, z: float64
  orientation_covariance: float64[9]
  angular_velocity_covariance: float64[9]
  linear_acceleration_covariance: float64[9]

Validation Rules:
  - Quaternion must be normalized (x²+y²+z²+w² = 1)
  - Angular velocity magnitude typically < 3.14 rad/s (avoid gimbal lock)
  - Linear acceleration includes gravity (expect ~9.81 m/s² at rest)
```

### Stage 2: Processed Features

**Feature Points** (Internal to VSLAM, not directly published)
```python
FeaturePoint:
  id: int                    # Unique feature identifier
  position_2d: (u, v)        # Image coordinates
  position_3d: (x, y, z)     # World coordinates (if triangulated)
  descriptor: float[128]     # ORB/SIFT descriptor vector
  track_count: int           # Number of frames tracking this feature
  is_keypoint: bool          # Whether this is a keyframe feature
```

---

## Entity 2: VSLAM Map

### Purpose
Represents the spatial model built by Visual SLAM, containing keyframes, 3D feature points, loop closure constraints, and robot trajectory history.

### Map Structure

**Keyframes** (Stored in VSLAM state, exported via `/vslam/map`)
```python
Keyframe:
  id: int                    # Sequential keyframe ID
  timestamp: Time            # Capture time
  pose: SE3                  # 6-DOF pose (position + orientation)
    position: (x, y, z)      # meters
    orientation: Quaternion  # (x, y, z, w)
  features: List[FeaturePoint]  # Observed features in this frame
  connections: List[int]     # IDs of connected keyframes
  loop_closure_edges: List[(int, SE3)]  # Loop closure links

Validation Rules:
  - Keyframes spaced ≥0.5m apart or 30° rotation
  - Each keyframe must have ≥50 tracked features
  - Loop closure edges added only if reprojection error < 2px
```

**3D Point Cloud Map** (ROS 2 Message: `sensor_msgs/PointCloud2`)
```yaml
Fields:
  header:
    frame_id: "map"
  width: uint32              # Number of points
  height: 1                  # Unordered cloud
  fields:
    - name: "x", offset: 0, datatype: FLOAT32, count: 1
    - name: "y", offset: 4, datatype: FLOAT32, count: 1
    - name: "z", offset: 8, datatype: FLOAT32, count: 1
    - name: "rgb", offset: 12, datatype: UINT32, count: 1
  point_step: 16             # Bytes per point
  row_step: width * point_step
  data: uint8[]              # Packed point data

Validation Rules:
  - Point coordinates in world frame ("map")
  - Typical indoor map: 10k-100k points
  - Points filtered for outliers (statistical outlier removal)
```

**Pose Graph** (Internal representation, visualized in RViz)
```python
PoseGraph:
  nodes: Dict[int, Keyframe]           # Keyframe ID → Keyframe
  edges: List[(int, int, SE3, Matrix6)]  # (from_id, to_id, transform, covariance)
  loop_closures: List[(int, int)]      # Detected loop closure pairs

Optimization:
  - Pose graph optimized via g2o or GTSAM (backend)
  - Optimization triggered on loop closure detection
  - Minimizes reprojection error across all keyframes
```

---

## Entity 3: Costmap

### Purpose
Represents occupancy grid with obstacle information and traversability costs, used by Nav2 planners to generate safe paths.

### ROS 2 Message: `nav_msgs/OccupancyGrid`

```yaml
Fields:
  header:
    stamp: Time
    frame_id: "map"          # Must match VSLAM map frame
  info:
    resolution: float32      # meters/cell (e.g., 0.05 = 5cm cells)
    width: uint32            # Grid width in cells
    height: uint32           # Grid height in cells
    origin:                  # Bottom-left corner in world frame
      position: (x, y, z)
      orientation: Quaternion
  data: int8[]               # Occupancy values per cell

Cell Values:
  -1: Unknown (not observed)
  0: Free (traversable)
  1-99: Occupied (obstacle), higher = more certain
  100: Lethal obstacle (guaranteed collision)

Validation Rules:
  - Resolution typically 0.05m (5cm) for indoor navigation
  - Grid size typically 200x200 cells (10m x 10m coverage)
  - Costmap updated at 5Hz from sensor data
```

### Layered Costmap Composition

Nav2 uses **layered costmap** merging multiple sources:

```python
LayeredCostmap:
  layers:
    - StaticLayer:          # From VSLAM map (static obstacles)
        base_map: OccupancyGrid
        trinary_costmap: bool  # {free, obstacle, unknown} only
    - ObstacleLayer:        # From depth camera (dynamic obstacles)
        observation_sources:
          - camera_depth    # Topic: /camera/depth/image_raw
        clearing: true      # Clear old obstacles
        marking: true       # Add new obstacles
        raytrace_range: 3.0 # meters
    - InflationLayer:       # Expand obstacles by robot radius
        inflation_radius: 0.5  # meters
        cost_scaling_factor: 5.0

  master_grid: OccupancyGrid  # Merged output

Merge Logic:
  - MAX(static, obstacle, inflation) per cell
  - Update rate: 5Hz
```

### Humanoid-Specific Costmap Extensions

```yaml
HumanoidFootprintLayer:
  footprint_model:
    type: "two_circles"      # Model bipedal stance
    circle1:
      center: (0.15, 0.0)    # Right foot offset
      radius: 0.12           # Foot radius
    circle2:
      center: (-0.15, 0.0)   # Left foot offset
      radius: 0.12
  balance_margin: 0.2        # Extra clearance for balance (meters)

Validation Rules:
  - Footprint must fit within inflation radius
  - Balance margin added to inflation_radius
  - Dynamic footprint updates based on gait phase (future work)
```

---

## Entity 4: Behavior Tree

### Purpose
Represents hierarchical task structure defining navigation behaviors, conditions, and fallback strategies for autonomous robot control.

### Behavior Tree XML Schema

**Example: Humanoid Navigation Behavior Tree**

```xml
<root main_tree_to_execute="HumanoidNavigate">

  <!-- Main navigation tree -->
  <BehaviorTree ID="HumanoidNavigate">
    <RecoveryNode number_of_retries="5" name="NavigateRecovery">

      <!-- Primary navigation sequence -->
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1" name="ComputePathToPose">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            <ClearEntireCostmap name="ClearGlobalCostmap" service_name="global_costmap/clear_entirely_global_costmap"/>
          </RecoveryNode>
        </RateController>

        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath path="{path}" controller_id="FollowPath"/>
          <ClearEntireCostmap name="ClearLocalCostmap" service_name="local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>
      </PipelineSequence>

      <!-- Recovery behaviors (fallback if navigation fails) -->
      <SequenceStar name="RecoveryActions">
        <ClearEntireCostmap name="ClearAllCostmaps" service_name="local_costmap/clear_entirely_local_costmap"/>
        <Spin spin_dist="1.57" name="Spin90Degrees"/>
        <Wait wait_duration="3" name="StabilizationWait"/>
        <BackUp backup_dist="0.3" backup_speed="0.1" name="BackUpSlightly"/>
      </SequenceStar>

    </RecoveryNode>
  </BehaviorTree>

</root>
```

### Behavior Tree Node Types

**Control Nodes**:
```yaml
Sequence:
  description: Execute children in order; fail if any child fails
  use_case: Main navigation pipeline (compute → follow path)

Fallback:
  description: Try children in order; succeed if any child succeeds
  use_case: Recovery behaviors (try spin, then wait, then backup)

Parallel:
  description: Execute children concurrently
  use_case: Monitor obstacles while following path (not shown in example)

RecoveryNode:
  description: Retry child N times; execute recovery on failures
  use_case: Wrapper for navigation actions with fallback behaviors
```

**Action Nodes**:
```yaml
ComputePathToPose:
  inputs:
    goal: geometry_msgs/PoseStamped   # Target pose
    planner_id: string                # Planner name (GridBased, Smac, etc.)
  outputs:
    path: nav_msgs/Path               # Computed path
  failure_modes:
    - No valid path found (FAILURE)
    - Goal not reachable (FAILURE)
    - Timeout (FAILURE after 10s)

FollowPath:
  inputs:
    path: nav_msgs/Path
    controller_id: string             # Controller name (DWB, TEB, etc.)
  outputs:
    cmd_vel: geometry_msgs/Twist      # Velocity commands
  failure_modes:
    - Collision imminent (FAILURE)
    - Path lost (FAILURE)
    - Goal reached (SUCCESS)

Spin:
  inputs:
    spin_dist: float                  # Rotation angle (radians)
  outputs:
    cmd_vel: geometry_msgs/Twist
  use_case: Recovery behavior for stuck robot

Wait:
  inputs:
    wait_duration: float              # Seconds
  use_case: Stabilization pause for humanoid balance

BackUp:
  inputs:
    backup_dist: float                # Meters (negative = forward)
    backup_speed: float               # m/s
  use_case: Extricate from tight space
```

### Behavior Tree Validation Rules

```python
ValidationRules:
  - Root must have exactly one main_tree_to_execute
  - RecoveryNode must have 1 primary child + 1 recovery child
  - Action nodes must specify failure modes and timeouts
  - Blackboard variables (e.g., {goal}, {path}) must be defined before use
  - Max tree depth: 5 levels (prevent infinite recursion)
  - Spin angles limited to [-π, π] (avoid excessive rotation)
  - Wait durations limited to [0, 60] seconds
```

---

## Entity 5: Synthetic Dataset

### Purpose
Represents collection of sensor data (images, depth, segmentation, labels) generated from Isaac Sim simulation for training perception models.

### Dataset Directory Structure

```
synthetic_dataset/
├── metadata.json                # Dataset-level metadata
├── rgb/                         # RGB images
│   ├── 0000000.png
│   ├── 0000001.png
│   └── ...
├── depth/                       # Depth maps
│   ├── 0000000.npy              # NumPy arrays (float32)
│   ├── 0000001.npy
│   └── ...
├── segmentation/                # Semantic segmentation
│   ├── 0000000.png              # Indexed PNG (class ID per pixel)
│   ├── 0000001.png
│   └── ...
├── bounding_boxes/              # Object detection labels
│   ├── 0000000.json
│   ├── 0000001.json
│   └── ...
└── camera_info.yaml             # Camera intrinsic parameters
```

### Metadata Schema

**metadata.json**:
```json
{
  "dataset_name": "humanoid_indoor_navigation_v1",
  "creation_date": "2025-12-05T10:30:00Z",
  "isaac_sim_version": "4.0.0",
  "total_frames": 1000,
  "camera_config": {
    "resolution": [640, 480],
    "fov_horizontal": 90.0,
    "fov_vertical": 60.0,
    "depth_range": [0.1, 10.0]
  },
  "domain_randomization": {
    "lighting_variation": true,
    "texture_randomization": true,
    "object_placement_randomization": true,
    "randomization_seed": 42
  },
  "class_labels": {
    "0": "background",
    "1": "wall",
    "2": "floor",
    "3": "door",
    "4": "obstacle",
    "5": "furniture"
  },
  "statistics": {
    "mean_rgb": [0.485, 0.456, 0.406],
    "std_rgb": [0.229, 0.224, 0.225],
    "depth_histogram": "depth_hist.png"
  }
}
```

**bounding_boxes/0000000.json**:
```json
{
  "frame_id": 0,
  "timestamp": 1.234567,
  "camera_pose": {
    "position": [0.5, 0.0, 1.2],
    "orientation": [0.0, 0.0, 0.0, 1.0]
  },
  "objects": [
    {
      "class_id": 4,
      "class_name": "obstacle",
      "bbox_2d": {
        "xmin": 120,
        "ymin": 150,
        "xmax": 220,
        "ymax": 300
      },
      "bbox_3d": {
        "center": [2.5, 0.3, 0.5],
        "dimensions": [0.5, 0.5, 1.0],
        "rotation": [0.0, 0.0, 0.707, 0.707]
      },
      "confidence": 1.0
    }
  ]
}
```

### Camera Calibration

**camera_info.yaml** (ROS 2 `sensor_msgs/CameraInfo` format):
```yaml
height: 480
width: 640
distortion_model: "plumb_bob"
D: [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion in simulation
K: [554.25, 0.0, 320.5,       # Intrinsic matrix (3x3)
    0.0, 554.25, 240.5,
    0.0, 0.0, 1.0]
R: [1.0, 0.0, 0.0,            # Rectification matrix (identity)
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0]
P: [554.25, 0.0, 320.5, 0.0,  # Projection matrix (3x4)
    0.0, 554.25, 240.5, 0.0,
    0.0, 0.0, 1.0, 0.0]
binning_x: 0
binning_y: 0
roi:
  x_offset: 0
  y_offset: 0
  height: 0
  width: 0
  do_rectify: false
```

### Data Validation Rules

```python
SyntheticDatasetValidation:
  - RGB images: 640x480, PNG format, 3 channels (RGB)
  - Depth maps: 640x480, .npy format, float32, range [0.1, 10.0]
  - Segmentation: 640x480, PNG format, indexed (class ID per pixel)
  - Bounding boxes: Valid JSON, bbox coordinates within image bounds
  - Frame IDs: Sequential, zero-padded to 7 digits
  - Timestamp: Monotonically increasing, typical interval 33ms (30Hz)
  - Class labels: All segmentation pixels map to defined classes
  - Camera info: K matrix must be valid (positive focal lengths)
```

---

## Entity 6: ROS Topics/Messages

### Purpose
Represents data communication channels between perception (Isaac ROS), mapping (VSLAM), planning (Nav2), and control components.

### Topic Taxonomy

| Topic Name | Message Type | Publisher | Subscriber(s) | Rate (Hz) |
|------------|--------------|-----------|---------------|-----------|
| `/camera/left/image_raw` | sensor_msgs/Image | Isaac Sim Bridge | VSLAM | 30 |
| `/camera/right/image_raw` | sensor_msgs/Image | Isaac Sim Bridge | VSLAM | 30 |
| `/camera/imu` | sensor_msgs/Imu | Isaac Sim Bridge | VSLAM | 200 |
| `/camera/depth/image_raw` | sensor_msgs/Image | Isaac Sim Bridge | Nav2 Obstacle Layer | 30 |
| `/vslam/pose` | geometry_msgs/PoseStamped | VSLAM | Nav2 Localization | 30 |
| `/vslam/map` | sensor_msgs/PointCloud2 | VSLAM | RViz | 1 |
| `/vslam/status` | isaac_ros_visual_slam_interfaces/VisualSlamStatus | VSLAM | Monitoring | 10 |
| `/map` | nav_msgs/OccupancyGrid | Map Server | Nav2 Costmap | On demand |
| `/global_costmap/costmap` | nav_msgs/OccupancyGrid | Nav2 | Visualization | 5 |
| `/local_costmap/costmap` | nav_msgs/OccupancyGrid | Nav2 | Visualization | 5 |
| `/cmd_vel` | geometry_msgs/Twist | Nav2 Controller | Robot Controller | 20 |
| `/goal_pose` | geometry_msgs/PoseStamped | User/RViz | Nav2 | On demand |

### Message Synchronization Rules

```yaml
Synchronization:
  Stereo Cameras:
    policy: ApproximateTime
    max_time_diff: 0.01 sec    # 10ms tolerance
    requirement: Both left+right images required for VSLAM

  Camera + IMU:
    policy: ApproximateTime
    max_time_diff: 0.005 sec   # 5ms tolerance
    requirement: IMU updates used for motion prediction

  Depth + RGB:
    policy: ExactTime           # Must be perfectly aligned
    requirement: Both from same frame for obstacle detection

TF Tree Requirements:
  - map → odom → base_link → camera_left/right/depth → imu_link
  - VSLAM publishes map→odom transform
  - Robot controller publishes odom→base_link transform
  - Static transforms for camera frames defined in URDF
```

---

## Data Persistence

### Temporary/Runtime Data
- **ROS 2 topics**: Real-time streaming, no persistence
- **RViz visualizations**: Runtime only, not saved
- **VSLAM internal state**: Lost on node shutdown

### Persistent Data
- **Synthetic datasets**: File system (examples/ directory)
- **VSLAM maps**: Can be saved/loaded via service calls
  - Map save: `/vslam/save_map` (isaac_ros_visual_slam_interfaces/srv/SaveMap)
  - Map load: `/vslam/load_map` (isaac_ros_visual_slam_interfaces/srv/LoadMap)
- **Nav2 configuration files**: YAML files in config/ directory
- **Behavior trees**: XML files in config/ directory

### Storage Sizing Estimates

| Data Type | Size per Frame | 1000 Frames | Notes |
|-----------|----------------|-------------|-------|
| RGB Image (640x480 PNG) | ~500 KB | 500 MB | Compressed |
| Depth Map (640x480 float32) | 1.2 MB | 1.2 GB | Uncompressed NumPy |
| Segmentation (640x480 indexed) | ~100 KB | 100 MB | Indexed PNG |
| Bounding Box JSON | ~2 KB | 2 MB | Few objects per frame |
| **Total Dataset** | | **~2 GB** | Typical 1k frame dataset |
| VSLAM Map | | 10-50 MB | Depends on environment size |
| ROS Bag (all topics, 1 min) | | ~500 MB | 30Hz cameras + IMU |

---

## Summary

This chapter's data model focuses on **data flows** and **configuration structures** rather than traditional databases:

1. **Perception Pipeline Data**: Sensor messages (camera, depth, IMU) flowing through ROS topics
2. **VSLAM Map**: Keyframes, point clouds, pose graphs representing spatial understanding
3. **Costmap**: Occupancy grids for safe navigation planning
4. **Behavior Tree**: Hierarchical task definitions for autonomous behavior
5. **Synthetic Dataset**: Training data exported from Isaac Sim
6. **ROS Topics**: Communication channels tying all components together

All entities validated against ROS 2 message standards and performance requirements (30Hz perception, 5Hz costmap updates, <100ms latency).

---

**Data Model Completed**: 2025-12-05
**Next Step**: Generate contracts (code example templates, launch file schemas)
