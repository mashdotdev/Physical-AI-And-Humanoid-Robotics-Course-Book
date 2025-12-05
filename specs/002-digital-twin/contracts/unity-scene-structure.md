# Contract: Unity Scene Structure for ROS Integration

**Purpose**: Define required Unity scene hierarchy and component configuration for digital twin visualization.

**Version**: 1.0
**Date**: 2025-12-05
**Unity Version**: 2021.3 LTS or newer

---

## Required Unity Packages

```json
// Packages/manifest.json
{
  "dependencies": {
    "com.unity.robotics.ros-tcp-connector": "0.7.0",
    "com.unity.robotics.urdf-importer": "0.7.0"
  }
}
```

---

## Scene Hierarchy

```
Scene: HumanoidDigitalTwin
├── ROSConnection (GameObject)
│   └── ROSConnection (Component)
│       ├── ROS IP Address: 192.168.1.100
│       ├── ROS Port: 10000
│       └── Protocol: ROS 2
│
├── RobotModel (GameObject - imported from URDF)
│   ├── ArticulationBody (Component)
│   └── Child Links (nested GameObjects)
│
├── Environment (GameObject)
│   ├── Ground Plane
│   └── Obstacles
│
├── Human Character (GameObject - optional)
│   └── CharacterController
│
├── Main Camera
│   └── CameraController script
│
└── Directional Light
```

---

## Required Scripts

### TimeSync.cs (ROS Clock Subscriber)

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Rosgraph;

public class TimeSync : MonoBehaviour
{
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<ClockMsg>("/clock", OnClockReceived);
    }

    void OnClockReceived(ClockMsg msg)
    {
        // Sync Unity physics to ROS time
        float targetTime = msg.clock.sec + msg.clock.nanosec / 1e9f;
        // Implementation: Advance Physics.Simulate() to match
    }
}
```

### JointStateSubscriber.cs

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class JointStateSubscriber : MonoBehaviour
{
    private ArticulationBody[] articulationBodies;

    void Start()
    {
        ROSConnection.GetOrCreateInstance()
            .Subscribe<JointStateMsg>("/joint_states", UpdateJointStates);

        articulationBodies = GetComponentsInChildren<ArticulationBody>();
    }

    void UpdateJointStates(JointStateMsg msg)
    {
        for (int i = 0; i < msg.name.Length; i++)
        {
            // Match joint by name, update ArticulationBody position
        }
    }
}
```

---

## Contract Requirements

✅ ROSConnection GameObject MUST exist before scene starts
✅ RobotModel MUST use ArticulationBody (not legacy Joints)
✅ All sensor frames MUST match TF tree from ROS
✅ Physics timestep = 0.01s (100 Hz) to match ROS control rate

---

## Validation

1. Play Unity scene
2. Check Console: "ROS TCP Connector connected to 192.168.1.100:10000"
3. Verify joint movements sync with Gazebo (visual inspection)
4. Measure latency: Timestamp comparison between /joint_states and Unity update