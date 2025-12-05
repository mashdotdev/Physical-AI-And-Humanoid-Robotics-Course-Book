using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

/// <summary>
/// Joint State Subscriber for Unity Digital Twin
///
/// Subscribes to ROS /joint_states topic and updates Unity ArticulationBody components
/// to mirror the Gazebo simulation.
///
/// Author: Physical AI Textbook - Chapter 2
/// </summary>
public class JointStateSubscriber : MonoBehaviour
{
    private ROSConnection ros;
    private ArticulationBody[] joints;

    void Start()
    {
        // Get or create ROS connection
        ros = ROSConnection.GetOrCreateInstance();

        // Subscribe to /joint_states topic
        ros.Subscribe<JointStateMsg>("/joint_states", UpdateJointStates);

        // Get all articulation bodies in children (joints)
        joints = GetComponentsInChildren<ArticulationBody>();

        Debug.Log($"JointStateSubscriber initialized with {joints.Length} joints");
    }

    /// <summary>
    /// Callback for /joint_states topic
    /// Updates Unity joint positions to match ROS joint states
    /// </summary>
    void UpdateJointStates(JointStateMsg msg)
    {
        for (int i = 0; i < msg.name.Length; i++)
        {
            ArticulationBody joint = FindJoint(msg.name[i]);
            if (joint != null)
            {
                // Get current drive configuration
                var drive = joint.xDrive;

                // Convert radians to degrees (Unity uses degrees)
                drive.target = (float)msg.position[i] * Mathf.Rad2Deg;

                // Apply updated drive
                joint.xDrive = drive;
            }
        }
    }

    /// <summary>
    /// Find ArticulationBody by joint name
    /// </summary>
    ArticulationBody FindJoint(string name)
    {
        foreach (var joint in joints)
        {
            if (joint.name == name)
            {
                return joint;
            }
        }

        // Joint not found (may be fixed joint or not controlled)
        return null;
    }

    void OnDestroy()
    {
        // Unsubscribe when object is destroyed
        if (ros != null)
        {
            ros.Unsubscribe("/joint_states");
        }
    }
}
