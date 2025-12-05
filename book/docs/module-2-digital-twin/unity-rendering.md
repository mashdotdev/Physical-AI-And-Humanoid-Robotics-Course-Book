---
title: 2.4 Unity High-Fidelity Rendering
sidebar_position: 4
---

# 2.4 Unity High-Fidelity Rendering

## Introduction

Unity provides **photorealistic rendering** for stakeholder demos, human-robot interaction (HRI) testing, and visual debugging. While Gazebo handles physics, Unity mirrors the robot's state for high-quality visualization.

**Key Use Cases**:
- Stakeholder demos (board meetings, funding pitches)
- HRI testing (robot interacting with humans in realistic environments)
- Camera simulation (training computer vision models with synthetic data)

## Unity Installation

1. Download Unity Hub: https://unity.com/download
2. Install Unity 2021.3 LTS or newer
3. Install URDF Importer package:
   ```
   Window → Package Manager → Add package from git URL
   com.unity.robotics.urdf-importer
   ```

4. Install ROS TCP Connector:
   ```
   com.unity.robotics.ros-tcp-connector
   ```

## Import URDF to Unity

1. **Assets → Import Robot from URDF**
2. Select `humanoid_robot.urdf.xacro`
3. Unity auto-converts meshes and creates ArticulationBody components

**Result**: Robot GameObject with:
- ArticulationBody (Unity's advanced physics for robotics)
- MeshRenderer (visual geometry)
- Colliders (collision geometry)

## ROS-Unity Bridge

Create C# script to subscribe to `/joint_states`:

```csharp title="unity-digital-twin/Assets/Scripts/ROSBridge/JointStateSubscriber.cs"
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class JointStateSubscriber : MonoBehaviour
{
    private ROSConnection ros;
    private ArticulationBody[] joints;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<JointStateMsg>("/joint_states", UpdateJointStates);

        joints = GetComponentsInChildren<ArticulationBody>();
    }

    void UpdateJointStates(JointStateMsg msg)
    {
        for (int i = 0; i < msg.name.Length; i++)
        {
            ArticulationBody joint = FindJoint(msg.name[i]);
            if (joint != null)
            {
                var drive = joint.xDrive;
                drive.target = (float)msg.position[i] * Mathf.Rad2Deg;
                joint.xDrive = drive;
            }
        }
    }

    ArticulationBody FindJoint(string name)
    {
        foreach (var joint in joints)
        {
            if (joint.name == name) return joint;
        }
        return null;
    }
}
```

## Launch ROS TCP Endpoint

```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

In Unity: **Robotics → ROS Settings → ROS IP Address → localhost**

Press Play → Robot should sync with Gazebo!

## Section Summary

**Unity Use Cases**:
- Photorealistic rendering
- HRI testing with human characters
- Synthetic camera data generation

**Next Section**: [2.6 Full Digital Twin Pipeline Integration](./full-pipeline.md)
