# Quickstart: Chapter 1: The Robotic Nervous System (ROS 2)

This guide will help you set up your environment and run the basic ROS 2 examples from Chapter 1.

## Prerequisites

### System Requirements
- **Operating System**: Ubuntu 22.04 LTS (recommended) or WSL2 on Windows
- **Python**: 3.10 or higher
- **ROS 2**: Humble or Iron distribution
- **Hardware**: Any modern computer (GPU not required for basic examples)

### Installing ROS 2

If you haven't installed ROS 2 yet, follow the official installation guide:

**Ubuntu 22.04:**
```bash
# Set up sources
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Add ROS 2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble (or Iron)
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions

# Source ROS 2 environment (add to ~/.bashrc for persistence)
source /opt/ros/humble/setup.bash
```

Verify installation:
```bash
ros2 --version
# Expected: ros2 cli version 0.X.X
```

### Project Setup

Clone or navigate to the hackathon-book repository:
```bash
cd ~/path/to/hackathon-book
```

Make Python scripts executable:
```bash
chmod +x src/ros2_chapter1/nodes/*.py
```

## Running the Examples

### Example 1: Simple Publisher

Publish String messages to a topic at 1 Hz:

```bash
python3 src/ros2_chapter1/nodes/simple_publisher.py
```

**Expected Output:**
```text
[INFO] [timestamp] [simple_publisher]: Simple Publisher started. Publishing to /robot/message
[INFO] [timestamp] [simple_publisher]: Publishing: "Hello from ROS 2! Message count: 0"
[INFO] [timestamp] [simple_publisher]: Publishing: "Hello from ROS 2! Message count: 1"
...
```

**Inspect the topic (in another terminal):**
```bash
# List all active topics
ros2 topic list

# Show topic information
ros2 topic info /robot/message

# Echo messages in real-time
ros2 topic echo /robot/message
```

### Example 2: Simple Subscriber

Subscribe to messages from `/robot/message`:

**Terminal 1 (Publisher):**
```bash
python3 src/ros2_chapter1/nodes/simple_publisher.py
```

**Terminal 2 (Subscriber):**
```bash
python3 src/ros2_chapter1/nodes/simple_subscriber.py
```

**Expected Output (Terminal 2):**
```text
[INFO] [timestamp] [simple_subscriber]: Simple Subscriber started. Listening to /robot/message
[INFO] [timestamp] [simple_subscriber]: Received message #1: "Hello from ROS 2! Message count: 0"
[INFO] [timestamp] [simple_subscriber]: Received message #2: "Hello from ROS 2! Message count: 1"
...
```

### Example 3: Service Client and Server

Demonstrate request-response communication using a service that adds two integers.

**Terminal 1 (Server):**
```bash
python3 src/ros2_chapter1/nodes/service_client_server.py --ros-args -p mode:=server
```

**Expected Output:**
```text
[INFO] [timestamp] [add_two_ints_server]: AddTwoInts service server started at /add_two_ints
```

**Terminal 2 (Client):**
```bash
python3 src/ros2_chapter1/nodes/service_client_server.py --ros-args -p mode:=client -p a:=10 -p b:=25
```

**Expected Output:**
```text
[INFO] [timestamp] [add_two_ints_client]: Service client ready. Request: a=10, b=25
[INFO] [timestamp] [add_two_ints_client]: Calling service with a=10, b=25
[INFO] [timestamp] [add_two_ints_client]: Response: 10 + 25 = 35
```

**Call service manually:**
```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 42, b: 58}"
```

## Troubleshooting

### Issue: `ros2: command not found`
**Solution:** Source the ROS 2 setup script:
```bash
source /opt/ros/humble/setup.bash
# Add to ~/.bashrc to make permanent:
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Issue: `ModuleNotFoundError: No module named 'rclpy'`
**Solution:** Ensure ROS 2 Python client library is installed:
```bash
sudo apt install python3-rclpy
```

### Issue: Service call hangs or client can't find server
**Solution:**
1. Verify the server is running: `ros2 service list`
2. Check that both nodes are in the same ROS domain:
   ```bash
   echo $ROS_DOMAIN_ID
   # Should be the same in both terminals
   ```

### Issue: Permission denied when running Python scripts
**Solution:** Make scripts executable:
```bash
chmod +x src/ros2_chapter1/nodes/*.py
```

## Next Steps

Now that you've run the basic examples, proceed to the [full Chapter 1 documentation](../../book/docs/chapter1) in the Docusaurus book to:
- Understand the conceptual foundations of ROS 2
- Learn when to use topics vs. services vs. actions
- Integrate high-level AI systems with robot controllers
- Visualize robot structures using URDF

## Useful ROS 2 Commands

```bash
# List all nodes
ros2 node list

# Get info about a node
ros2 node info /simple_publisher

# List all topics
ros2 topic list

# Show message type of a topic
ros2 topic info /robot/message

# Echo topic messages
ros2 topic echo /robot/message

# Publish to a topic manually
ros2 topic pub /robot/message std_msgs/msg/String "{data: 'Hello from CLI'}"

# List all services
ros2 service list

# Call a service
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 10, b: 20}"

# Get system information
ros2 doctor
```
