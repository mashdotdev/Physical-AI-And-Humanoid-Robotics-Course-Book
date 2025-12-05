---
title: 2.7 Exercises and Summary
sidebar_position: 7
---

# 2.7 Exercises and Summary

## Hands-On Exercises

### Exercise 1: Test Gravity Toggle

**Goal**: Spawn robot in Gazebo, toggle gravity, observe behavior

**Steps**:
1. Launch Gazebo: `ros2 launch digital_twin_chapter2 gazebo_sim.launch.py`
2. Run gravity test: `python3 test_gravity.py`
3. Observe robot floating vs. falling

**Expected Outcome**: Robot floats when gravity=0, falls at 9.81 m/s² when gravity=-9.81

**Validation Criteria**:
- ✅ Robot spawns without errors
- ✅ Gravity toggle changes robot motion
- ✅ No inter-penetration with ground plane

### Exercise 2: Create Unity Scene with Human

**Goal**: Import URDF to Unity, add human character, test HRI reachability

**Steps**:
1. Open Unity, create new scene
2. Import URDF: Assets → Import Robot from URDF
3. Add human character from Unity Asset Store
4. Position human 1m in front of robot
5. Test arm reach

**Expected Outcome**: Robot arms can reach human within 1.5m radius

### Exercise 3: Simulate LiDAR and Visualize

**Goal**: Add LiDAR to URDF, launch Gazebo, visualize in RViz

**Steps**:
1. Add LiDAR sensor block to URDF
2. Launch Gazebo
3. Launch RViz: `rviz2`
4. Add LaserScan display, topic: `/scan`

**Expected Outcome**: Red scan points showing obstacles

### Exercise 4: Run Complete Digital Twin

**Goal**: Launch entire pipeline with one command

**Steps**:
1. `ros2 launch digital_twin_chapter2 digital_twin_complete.launch.py`
2. Open Unity, press Play
3. Verify robot moves in both Gazebo and Unity

**Expected Outcome**: Perfect sync between Gazebo physics and Unity rendering

## Chapter Summary

### Key Concepts Learned

1. **Digital Twins**: Virtual replicas enable safe testing, rapid iteration
2. **Gazebo Physics**: Gravity, friction, collision detection, joint dynamics
3. **ros2_control**: Hardware-agnostic controller interfaces
4. **Unity Rendering**: Photorealistic visualization for demos and HRI
5. **Sensor Simulation**: LiDAR, cameras, IMUs with realistic noise
6. **Integration**: Single-command launch orchestrates entire pipeline

### Technical Skills Acquired

- ✅ Install and configure Gazebo Classic for ROS 2
- ✅ Create physics-accurate URDF models with inertial properties
- ✅ Configure ros2_control for position/velocity/effort control
- ✅ Import URDFs to Unity with ROS-Unity bridge
- ✅ Simulate sensors (LiDAR, depth camera, IMU) with noise
- ✅ Validate simulation accuracy (gravity, collision, joint limits)
- ✅ Orchestrate multi-component systems with hierarchical launch files

### Common Pitfalls Avoided

- ❌ Missing `<inertial>` tags → Robot explodes in Gazebo
- ❌ Collision meshes too complex → Real-time factor &lt; 0.5
- ❌ Forgetting `use_sim_time: True` → TF tree desynchronized
- ❌ Incorrect joint limits → Controller rejects commands
- ❌ No sensor noise → Sim-to-real gap causes hardware failures

## Looking Ahead: Module 3

**Next Module** (Chapters 8-10) transitions to:
- **NVIDIA Isaac Sim**: GPU-accelerated physics (100x faster)
- **Photorealistic rendering**: RTX ray tracing for camera simulation
- **VSLAM & Nav2**: Visual SLAM and autonomous navigation
- **Isaac ROS**: Hardware-accelerated perception pipelines

**What Transfers Directly**:
- ✅ URDF models (Isaac Sim imports URDF natively)
- ✅ ros2_control architecture
- ✅ Sensor configuration patterns
- ✅ Launch file structure

**What Changes**:
- World files: Gazebo `.world` → Isaac USD (Universal Scene Description)
- Physics: ODE (CPU) → PhysX 5 (GPU)
- Rendering: Ogre → RTX ray tracing

## Additional Resources

- **ROS 2 Humble Docs**: https://docs.ros.org/en/humble/
- **Gazebo Tutorials**: http://gazebosim.org/tutorials
- **Unity Robotics Hub**: https://github.com/Unity-Technologies/Unity-Robotics-Hub
- **ros2_control Docs**: https://control.ros.org/humble/

## Congratulations!

You now have a complete digital twin pipeline that enables:
- Safe algorithm testing before hardware deployment
- Rapid iteration (minutes vs. weeks)
- Realistic sensor simulation with noise models
- High-fidelity visualization for stakeholders

**This is the foundation for all advanced Physical AI development.**

---

**Ready for Module 3?** Proceed to Chapter 8: NVIDIA Isaac Sim and GPU-Accelerated Simulation
