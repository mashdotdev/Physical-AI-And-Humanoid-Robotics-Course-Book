# Data Model: Chapter 1: The Robotic Nervous System (ROS 2)

## Entities

### Robot System Node
- **Description**: A process that performs computation, represented as an "organ" in the analogy.
- **Fields**: Name (string), ID (unique identifier), State (running, stopped, error)
- **Relationships**: Can publish/subscribe to Topics, offer/use Services, send/receive Actions.

### Robot System Topic
- **Description**: A named bus over which nodes exchange messages asynchronously, represented as "nerves."
- **Fields**: Name (string), MessageType (string), PublisherNodes (list of Node IDs), SubscriberNodes (list of Node IDs)
- **Validation Rules**: MessageType must be a predefined ROS 2 message type.

### Robot System Service
- **Description**: A request/response communication mechanism for synchronous interactions, represented as "reflexes."
- **Fields**: Name (string), RequestType (string), ResponseType (string), ServiceServerNode (Node ID), ServiceClientNodes (list of Node IDs)
- **Validation Rules**: RequestType and ResponseType must be predefined ROS 2 service types.

### Robot System Action
- **Description**: A long-running, goal-oriented communication for complex tasks, represented as "long-running tasks."
- **Fields**: Name (string), GoalType (string), ResultType (string), FeedbackType (string), ActionServerNode (Node ID), ActionClientNodes (list of Node IDs)
- **Validation Rules**: GoalType, ResultType, and FeedbackType must be predefined ROS 2 action types.

### Robot Description Format (e.g., URDF)
- **Description**: A standard format (e.g., URDF) for describing the kinematic and dynamic properties of a robot, acting as the "skeleton."
- **Fields**: Links (list of Link objects), Joints (list of Joint objects), Materials (list of Material objects)
- **Link Object**: Name (string), Visual (geometry, material), Collision (geometry), Inertial (mass, inertia)
- **Joint Object**: Name (string), Type (revolute, prismatic, fixed, etc.), ParentLink (string), ChildLink (string), Origin (position, orientation), Axis (vector)
- **Schema Evolution**: URDF is XML-based; changes typically involve adding/modifying links and joints.

### High-Level Decision-Making System
- **Description**: A software entity (e.g., AI agent) responsible for decision-making and command generation.
- **Fields**: Name (string), Logic (description of AI algorithm), OutputCommands (list of ROS 2 messages/service requests/action goals), InputSensors (list of ROS 2 messages/service responses/action feedback)
- **Relationships**: Communicates with Robot System Nodes via Topics, Services, and Actions.

### Robot Controller
- **Description**: Low-level software or hardware that directly manipulates robot joints and actuators.
- **Fields**: Name (string), ControlledJoints (list of Joint names), ActuatorInterfaces (hardware interface), SensorInputs (list of sensor data types)
- **Relationships**: Receives commands from High-Level Decision-Making Systems via Robot System Nodes.
