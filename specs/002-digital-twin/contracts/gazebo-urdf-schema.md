# Contract: Gazebo URDF Extension Schema

**Purpose**: Define the required URDF structure and Gazebo-specific extensions for digital twin simulation.

**Version**: 1.0
**Date**: 2025-12-05
**ROS 2 Distro**: Humble
**Gazebo Version**: Classic 11.x

---

## URDF Base Structure (Required)

### Minimal Valid URDF

```xml
<?xml version="1.0"?>
<robot name="example_robot">
  <!-- At least 1 link required -->
  <link name="base_link">
    <!-- Visual geometry (required for rendering) -->
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>

    <!-- Collision geometry (required for physics) -->
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>

    <!-- Inertial properties (required for physics) -->
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0"
               iyy="0.2" iyz="0.0" izz="0.15"/>
    </inertial>
  </link>
</robot>
```

**Contract Requirements**:
- ✅ Every `<link>` MUST have `<inertial>` (Gazebo will not simulate without mass)
- ✅ Every `<link>` MUST have `<collision>` (otherwise no physics interactions)
- ✅ Every `<joint>` MUST reference existing parent/child links
- ✅ Inertia matrix MUST be positive definite (physically valid)
- ✅ `<robot name>` MUST match parameter passed to `spawn_entity.py`

---

## Gazebo-Specific Extensions

### Physics Properties

```xml
<gazebo reference="base_link">
  <!-- Surface friction (affects sliding/rolling) -->
  <mu1>0.8</mu1>  <!-- Friction coefficient direction 1 -->
  <mu2>0.8</mu2>  <!-- Friction coefficient direction 2 -->

  <!-- Collision properties -->
  <kp>1000000.0</kp>  <!-- Contact stiffness (N/m) -->
  <kd>1.0</kd>        <!-- Contact damping (N·s/m) -->

  <!-- Material (for visual appearance in Gazebo) -->
  <material>Gazebo/Blue</material>
</gazebo>
```

**Contract Requirements**:
- ✅ `reference` attribute MUST match a `<link name>` from URDF
- ✅ `mu1`, `mu2`: 0.0 (frictionless) to 1.0 (high friction)
- ✅ `kp`, `kd`: Higher = more rigid contacts (but can cause instability if too high)

---

## ros2_control Integration

### Controller Configuration in URDF

```xml
<ros2_control name="GazeboSystem" type="system">
  <hardware>
    <!-- Gazebo hardware interface plugin -->
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>

  <!-- Define controlled joints -->
  <joint name="shoulder_joint">
    <command_interface name="position">
      <param name="min">-1.57</param>  <!-- -90° -->
      <param name="max">1.57</param>   <!-- +90° -->
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>

  <joint name="elbow_joint">
    <command_interface name="position">
      <param name="min">0.0</param>
      <param name="max">3.14</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>

<!-- Load Gazebo ros2_control plugin -->
<gazebo>
  <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
    <parameters>$(find robot_description)/config/controllers.yaml</parameters>
  </plugin>
</gazebo>
```

**Contract Requirements**:
- ✅ `<joint name>` in ros2_control MUST match `<joint name>` in URDF
- ✅ `min`/`max` in command_interface SHOULD match URDF `<limit>` tags
- ✅ `state_interface` MUST include at least `position` (required for JointStateBroadcaster)
- ✅ `controllers.yaml` file MUST exist and define controller parameters

---

## Sensor Plugins

### LiDAR 2D (Ray Sensor)

```xml
<gazebo reference="lidar_link">
  <sensor name="laser_sensor" type="ray">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>  <!-- Show rays in Gazebo GUI -->
    <update_rate>30</update_rate>  <!-- Hz -->

    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>  <!-- -π -->
          <max_angle>3.14159</max_angle>   <!-- +π -->
        </horizontal>
      </scan>
      <range>
        <min>0.10</min>  <!-- 10 cm -->
        <max>30.0</max>  <!-- 30 m -->
        <resolution>0.01</resolution>  <!-- 1 cm -->
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>  <!-- 1 cm noise -->
      </noise>
    </ray>

    <!-- ROS 2 plugin -->
    <plugin name="laser_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/</namespace>
        <argument>~/out:=scan</argument>  <!-- Topic: /scan -->
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**Contract Requirements**:
- ✅ `reference` MUST match a URDF link (sensor's parent)
- ✅ `samples` * `update_rate` ≤ 72,000 (computational limit for real-time)
- ✅ `frame_name` MUST match TF frame (usually same as `reference` link)
- ✅ `output_type` MUST be `sensor_msgs/LaserScan` for 2D LiDAR

---

### Depth Camera (RGB-D)

```xml
<gazebo reference="camera_link">
  <sensor name="depth_camera" type="depth">
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>  <!-- 60° -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>   <!-- 10 cm -->
        <far>10.0</far>    <!-- 10 m -->
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>  <!-- Realistic for Intel RealSense -->
      </noise>
    </camera>

    <plugin name="depth_camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/camera</namespace>
        <argument>image_raw:=image_raw</argument>
        <argument>depth/image_raw:=depth/image_raw</argument>
        <argument>camera_info:=camera_info</argument>
      </ros>
      <camera_name>depth_camera</camera_name>
      <frame_name>camera_link</frame_name>
      <hack_baseline>0.07</hack_baseline>  <!-- Baseline for stereo -->
    </plugin>
  </sensor>
</gazebo>
```

**Contract Requirements**:
- ✅ `image/width` * `image/height` * `update_rate` * 3 bytes ≤ 100 MB/s (bandwidth limit)
- ✅ `horizontal_fov` in radians (π/3 = 60°, common for cameras)
- ✅ Publishes 3 topics: `/camera/image_raw`, `/camera/depth/image_raw`, `/camera/camera_info`

---

### IMU (Inertial Measurement Unit)

```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>  <!-- 100 Hz typical -->
    <imu>
      <!-- Accelerometer -->
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>  <!-- 0.01 rad/s -->
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </z>
      </angular_velocity>

      <!-- Gyroscope -->
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>  <!-- 0.01 m/s² -->
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>

    <plugin name="imu_controller" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/imu</namespace>
        <argument>~/out:=data</argument>  <!-- Topic: /imu/data -->
      </ros>
      <frame_name>imu_link</frame_name>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>
  </sensor>
</gazebo>
```

**Contract Requirements**:
- ✅ `update_rate` ≥ 50 Hz (minimum for reliable orientation estimation)
- ✅ Noise stddev values SHOULD match target hardware specs
- ✅ Publishes to `sensor_msgs/Imu` with orientation (quaternion), angular_velocity, linear_acceleration

---

## Validation Rules

### URDF Validation

```bash
# Check URDF syntax
check_urdf model.urdf

# Expected output:
# robot name is: example_robot
# ---------- Successfully Parsed XML ---------------
# root Link: base_link has 2 child(ren)
#   child(1):  link1
#   child(2):  link2
```

### Gazebo Spawn Test

```bash
# Terminal 1: Start Gazebo
gazebo --verbose

# Terminal 2: Spawn robot
ros2 run gazebo_ros spawn_entity.py \
  -topic robot_description \
  -entity my_robot

# Expected: Robot appears in Gazebo, no error messages
```

### ros2_control Test

```bash
# Check controller manager loaded
ros2 control list_controllers

# Expected output:
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
# position_controller[position_controllers/JointGroupPositionController] active
```

---

## Common Errors and Solutions

### Error: "No inertial data for link"

**Cause**: Missing `<inertial>` tag in URDF link

**Solution**:
```xml
<inertial>
  <mass value="1.0"/>
  <inertia ixx="0.01" ixy="0.0" ixz="0.0"
           iyy="0.01" iyz="0.0" izz="0.01"/>
</inertial>
```

### Error: "Joint exceeds [position] limits"

**Cause**: ros2_control command outside URDF `<limit>` tags

**Solution**: Verify command values:
```python
# In ROS node:
assert lower_limit <= command <= upper_limit
```

### Error: "Sensor plugin failed to load"

**Cause**: Plugin filename mismatch or missing gazebo_ros_pkgs installation

**Solution**:
```bash
# Install Gazebo ROS packages
sudo apt install ros-humble-gazebo-ros-pkgs
```

---

## Best Practices Checklist

- [ ] All links have `<inertial>` with realistic mass (use CAD or analytical formulas)
- [ ] Collision meshes are simplified (<1000 triangles each)
- [ ] Visual meshes can be high-poly (doesn't affect physics)
- [ ] Joint limits in ros2_control match URDF limits
- [ ] Sensor update rates are reasonable (10-100 Hz for most sensors)
- [ ] Friction coefficients match real materials (metal: 0.3-0.6, rubber: 0.7-1.0)
- [ ] Frame names in Gazebo plugins match TF tree
- [ ] All `<gazebo reference="">` tags reference valid URDF links

---

## References

- URDF Spec: http://wiki.ros.org/urdf/XML
- Gazebo Plugins: http://gazebosim.org/tutorials?tut=ros_gzplugins
- ros2_control: https://control.ros.org/humble/index.html