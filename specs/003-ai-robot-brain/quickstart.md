# Quickstart Guide: Chapter 3 - The AI-Robot Brain

**Purpose**: Get your environment ready to run all Chapter 3 examples
**Time Required**: 30-60 minutes (depending on download speeds)
**Prerequisites**: Chapters 1 & 2 completed (ROS 2 installed, simulation basics understood)

---

## System Requirements

### Minimum Hardware
- **CPU**: Intel Core i5 / AMD Ryzen 5 (4 cores)
- **RAM**: 16GB
- **GPU**: NVIDIA RTX 2060 (8GB VRAM)
- **Storage**: 100GB free space (for Isaac Sim + datasets)
- **OS**: Ubuntu 22.04 LTS

### Recommended Hardware
- **CPU**: Intel Core i7 / AMD Ryzen 7 (8 cores)
- **RAM**: 32GB
- **GPU**: NVIDIA RTX 3060 or better (12GB+ VRAM)
- **Storage**: 200GB free space (NVMe SSD preferred)

### Software Versions
- **Ubuntu**: 22.04 LTS (Jammy)
- **ROS 2**: Humble Hawksbill or Iron Irwini
- **Isaac Sim**: 4.0+ (2024.1 release or later)
- **Isaac ROS**: 3.0+
- **Python**: 3.10+
- **NVIDIA Driver**: 535+ (for CUDA 12.1+)

---

## Installation Steps

### Step 1: Verify ROS 2 Installation

From Chapter 1, you should already have ROS 2 installed. Verify:

```bash
# Check ROS 2 version
ros2 --version
# Expected output: ros2 cli version X.X.X (Humble or Iron)

# Source ROS 2 (add to ~/.bashrc for permanence)
source /opt/ros/humble/setup.bash  # or 'iron'

# Verify core tools
ros2 topic list
ros2 node list
```

**Troubleshooting**: If ROS 2 is not installed, refer back to Chapter 1 setup guide.

---

### Step 2: Install NVIDIA Driver and CUDA

Isaac Sim and Isaac ROS require NVIDIA GPU drivers with CUDA support.

```bash
# Check current NVIDIA driver version
nvidia-smi
# Should show driver version 535+ and CUDA 12.1+

# If driver needs update, install recommended driver
sudo ubuntu-drivers devices
sudo ubuntu-drivers autoinstall
sudo reboot

# Verify after reboot
nvidia-smi
```

**Expected output**:
```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 535.XX       Driver Version: 535.XX       CUDA Version: 12.1    |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|===============================+======================+======================|
|   0  NVIDIA GeForce ...  Off  | 00000000:01:00.0  On |                  N/A |
| 30%   45C    P8    15W / 170W |    500MiB /  8192MiB |      2%      Default |
+-------------------------------+----------------------+----------------------+
```

---

### Step 3: Install NVIDIA Isaac Sim

Isaac Sim is distributed via NVIDIA Omniverse platform.

#### 3a. Install Omniverse Launcher

```bash
# Download Omniverse Launcher
cd ~/Downloads
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# Make executable and run
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage
```

#### 3b. Install Isaac Sim via Launcher

1. Sign in to Omniverse Launcher (create free NVIDIA account if needed)
2. Navigate to **Exchange** tab
3. Search for "Isaac Sim"
4. Click **Install** on Isaac Sim 4.0+ (latest version)
5. Choose installation directory (default: `~/.local/share/ov/pkg/isaac-sim-4.0.0`)
6. Wait for download and installation (~20GB download, ~40GB installed)

#### 3c. Verify Isaac Sim Installation

```bash
# Launch Isaac Sim (replace version number as needed)
~/.local/share/ov/pkg/isaac-sim-4.0.0/isaac-sim.sh

# Isaac Sim window should open
# You should see the default empty stage
```

**First Launch**: May take 3-5 minutes to initialize shaders and cache.

**Common Issues**:
- **Vulkan error**: Install `vulkan-tools` via `sudo apt install vulkan-tools`
- **"No display" error on headless server**: Isaac Sim requires display; use remote desktop or X forwarding

---

### Step 4: Install Isaac ROS Packages

Isaac ROS provides GPU-accelerated perception algorithms.

#### 4a. Install Dependencies

```bash
# Install required packages
sudo apt update
sudo apt install -y \
    ros-humble-isaac-ros-visual-slam \
    ros-humble-isaac-ros-apriltag \
    ros-humble-isaac-ros-dnn-inference \
    ros-humble-isaac-ros-image-proc \
    ros-humble-cv-bridge \
    python3-opencv
```

**Alternative**: Build from source if binary packages not available:

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone Isaac ROS repositories
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

# Build with colcon
cd ~/ros2_ws
colcon build --symlink-install --packages-select isaac_ros_visual_slam

# Source workspace
source install/setup.bash
```

#### 4b. Verify Isaac ROS Installation

```bash
# Check for isaac_ros packages
ros2 pkg list | grep isaac_ros

# Expected output:
# isaac_ros_apriltag
# isaac_ros_visual_slam
# isaac_ros_dnn_inference
# ...
```

---

### Step 5: Install Nav2 Navigation Stack

Nav2 is the standard ROS 2 navigation framework.

```bash
# Install Nav2 meta-package
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-* \
    ros-humble-turtlebot3*  # For testing (optional)
```

#### Verify Nav2 Installation

```bash
# Check for Nav2 packages
ros2 pkg list | grep nav2

# Expected packages:
# nav2_amcl
# nav2_bt_navigator
# nav2_controller
# nav2_costmap_2d
# nav2_planner
# nav2_recoveries
# ...
```

---

### Step 6: Install RViz2 and Visualization Tools

```bash
# Install RViz2 (visualization)
sudo apt install -y \
    ros-humble-rviz2 \
    ros-humble-rviz-visual-tools \
    ros-humble-rqt* \
    ros-humble-plotjuggler-ros

# Verify RViz2
rviz2
# RViz window should open
```

---

### Step 7: Install Python Dependencies

Chapter examples use Python for scripting and data processing.

```bash
# Install Python packages
pip3 install --user \
    numpy \
    opencv-python \
    matplotlib \
    pillow \
    pyyaml \
    scipy

# Verify installations
python3 -c "import numpy, cv2, matplotlib; print('OK')"
```

---

### Step 8: Clone Chapter Examples Repository

All code examples for the book are available in a companion repository.

```bash
# Clone repository
cd ~/
git clone https://github.com/your-org/physical-ai-textbook-examples.git
cd physical-ai-textbook-examples/examples/module-3-ai-robot-brain

# Directory structure:
# isaac-sim/          - Synthetic data generation scripts
# isaac-ros-vslam/    - VSLAM launch files and configs
# nav2-humanoid/      - Nav2 configuration for humanoid
# complete-pipeline/  - Integrated pipeline launch files
```

---

### Step 9: Validate Environment

Run the automated validation script:

```bash
cd ~/physical-ai-textbook-examples/examples/module-3-ai-robot-brain
python3 validate_environment.py
```

**Expected output**:
```
✅ ROS 2 version: Humble Hawksbill
✅ NVIDIA GPU detected: GeForce RTX 3060 (8GB VRAM)
✅ Isaac Sim installed: v4.0.0
✅ Isaac ROS packages: 5/5 found
✅ Nav2 packages: 15/15 found
✅ RViz2 available
✅ Python dependencies: 6/6 installed
✅ Free disk space: 150GB

All checks passed! Environment ready for Chapter 3.
```

**Troubleshooting**: If any checks fail, refer to error messages and re-run corresponding installation step.

---

## Quick Test: Launch Isaac Sim with ROS 2

Verify Isaac Sim can communicate with ROS 2:

### Terminal 1: Launch Isaac Sim

```bash
~/.local/share/ov/pkg/isaac-sim-4.0.0/isaac-sim.sh
```

Once Isaac Sim loads:
1. Go to **Isaac Examples** → **ROS2** → **ROS2 Camera**
2. Click **Play** button (bottom left)
3. Scene should load with a camera and objects

### Terminal 2: Check ROS 2 Topics

```bash
source /opt/ros/humble/setup.bash
ros2 topic list

# Expected output (includes):
# /camera/rgb/image_raw
# /camera/depth/image_raw
# /camera/camera_info
```

### Terminal 3: Visualize in RViz

```bash
rviz2

# In RViz:
# 1. Add → By Topic → /camera/rgb/image_raw → Image
# 2. You should see camera feed from Isaac Sim
```

**Success Criteria**: You see live camera feed from Isaac Sim in RViz.

---

## Common Issues and Solutions

### Issue 1: Isaac Sim crashes on launch

**Symptom**: Black screen or segfault
**Solutions**:
1. Update NVIDIA driver: `sudo ubuntu-drivers autoinstall`
2. Check GPU memory: Close other applications using GPU
3. Try headless mode: `./isaac-sim.sh --headless` (for servers)

### Issue 2: Isaac ROS VSLAM not starting

**Symptom**: `isaac_ros_visual_slam` node dies immediately
**Solutions**:
1. Verify CUDA available: `nvidia-smi` should show CUDA version
2. Check camera topics publishing: `ros2 topic hz /camera/left/image_raw`
3. Validate camera_info has calibration: `ros2 topic echo /camera/left/camera_info --once`

### Issue 3: Nav2 planning fails

**Symptom**: "No plan found" errors in Nav2
**Solutions**:
1. Check VSLAM is tracking: `ros2 topic echo /vslam/status`
2. Verify map is published: `ros2 topic echo /map --once`
3. Ensure costmaps are updating: `ros2 topic hz /global_costmap/costmap`

### Issue 4: Slow performance / low FPS

**Symptom**: Isaac Sim runs at <10 FPS, VSLAM misses frames
**Solutions**:
1. Lower Isaac Sim quality: Settings → Rendering → Quality: LOW/MEDIUM
2. Reduce camera resolution: 640x480 instead of 1920x1080
3. Close background applications (browsers, IDEs)
4. Check GPU usage: `nvidia-smi` (should be <80% memory)

---

## Performance Benchmarks

After successful setup, your system should achieve:

| Metric | Target | How to Measure |
|--------|--------|----------------|
| Isaac Sim FPS | ≥30 FPS | Bottom-right corner in Isaac Sim window |
| Camera topic rate | 30Hz | `ros2 topic hz /camera/left/image_raw` |
| VSLAM pose rate | 30Hz | `ros2 topic hz /vslam/pose` |
| Nav2 costmap update | 5Hz | `ros2 topic hz /local_costmap/costmap` |
| End-to-end latency | <100ms | `ros2 topic delay /vslam/pose /cmd_vel` |

**If targets not met**: Reduce Isaac Sim quality or camera resolution.

---

## Next Steps

Your environment is now ready! Proceed to:
- **Section 3.1**: Conceptual introduction (no coding)
- **Section 3.2**: Hands-on synthetic data generation with Isaac Sim
- **Section 3.3**: Launch Isaac ROS VSLAM and build maps
- **Section 3.4**: Configure Nav2 for humanoid navigation
- **Section 3.5**: Integrate complete perception-navigation pipeline

---

## Quick Reference Commands

```bash
# Launch Isaac Sim
~/.local/share/ov/pkg/isaac-sim-4.0.0/isaac-sim.sh

# Source ROS 2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash  # If built from source

# Launch VSLAM
ros2 launch isaac-ros-vslam vslam_isaac_sim.launch.py

# Launch Nav2
ros2 launch nav2-humanoid nav2_humanoid.launch.py

# Launch complete pipeline
ros2 launch complete-pipeline full_pipeline.launch.py

# Monitor topics
ros2 topic list
ros2 topic hz /vslam/pose
ros2 topic echo /vslam/status

# Visualize
rviz2 -d config/pipeline_rviz.rviz

# Check GPU status
nvidia-smi
watch -n 1 nvidia-smi  # Continuous monitoring
```

---

## Getting Help

- **Isaac Sim Issues**: https://forums.developer.nvidia.com/c/omniverse/simulation/69
- **Isaac ROS Issues**: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/issues
- **Nav2 Issues**: https://github.com/ros-planning/navigation2/issues
- **Book Discord**: [Link to book community Discord]
- **Office Hours**: [Schedule if applicable]

---

**Quickstart Complete! Ready to dive into Chapter 3 content.**
