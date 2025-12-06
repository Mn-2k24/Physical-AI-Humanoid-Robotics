# Appendix B: Software Installation Guide

Step-by-step installation instructions for all software required throughout the course.

## Installation Order

Follow this sequence to avoid dependency issues:

1. System Updates & Base Tools
2. ROS 2 Humble
3. Gazebo Garden (or Classic)
4. Python Development Environment
5. Unity 2022 LTS (Module 2)
6. NVIDIA Isaac Sim (Module 3)
7. VLA Dependencies (Module 4)

## 1. System Updates & Base Tools

```bash
# Update package lists
sudo apt update && sudo apt upgrade -y

# Install essential build tools
sudo apt install -y \
  build-essential \
  cmake \
  git \
  curl \
  wget \
  python3-pip \
  python3-venv \
  vim \
  tmux

# Install Python 3.10+ (Ubuntu 22.04 includes 3.10 by default)
python3 --version  # Verify 3.10 or higher
```

## 2. ROS 2 Humble Installation

**Official Debian packages** (recommended):

```bash
# Set locale
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt install -y ros-humble-desktop

# Install additional tools
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  python3-colcon-common-extensions \
  python3-rosdep

# Initialize rosdep
sudo rosdep init
rosdep update

# Add to ~/.bashrc for auto-sourcing
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
ros2 --version  # Should show: ros2 cli version: humble
```

## 3. Gazebo Installation

### Option A: Gazebo Garden (Recommended, Latest)

```bash
# Add Gazebo repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Garden
sudo apt update
sudo apt install -y gz-garden

# Verify
gz sim --version  # Should show Gazebo Sim 7.x
```

### Option B: Gazebo Classic (Alternative, More Stable)

```bash
# Install Gazebo 11 (Classic)
sudo apt install -y gazebo11 libgazebo11-dev

# Verify
gazebo --version  # Should show Gazebo 11.x
```

## 4. Python Development Environment

```bash
# Create virtual environment for course
cd ~
python3 -m venv ~/physical-ai-venv
source ~/physical-ai-venv/bin/activate

# Clone companion code repository
git clone https://github.com/nizam/physical-ai-code.git
cd physical-ai-code

# Install dependencies
pip install --upgrade pip
pip install -r requirements.txt

# Verify key packages
python3 -c "import rclpy; print('rclpy OK')"
python3 -c "import numpy; print('numpy OK')"
python3 -c "import cv2; print('opencv OK')"

# Add venv activation to ~/.bashrc (optional)
echo "alias physical-ai='source ~/physical-ai-venv/bin/activate'" >> ~/.bashrc
```

## 5. Unity 2022 LTS (Module 2 Only)

**Install Unity Hub**:

```bash
# Download Unity Hub AppImage
wget https://public-cdn.cloud.unity3d.com/hub/prod/UnityHub.AppImage
chmod +x UnityHub.AppImage
sudo mv UnityHub.AppImage /usr/local/bin/unity-hub

# Run Unity Hub
unity-hub
```

**Install Unity Editor** (via Unity Hub GUI):
1. Open Unity Hub
2. Navigate to **Installs** → **Install Editor**
3. Select **2022.3 LTS** (or latest 2022 LTS)
4. Add modules:
   - Linux Build Support
   - Documentation
5. Accept license and wait for installation (~10GB download)

**Install Unity Robotics Hub**:
```bash
# Clone repository
git clone https://github.com/Unity-Technologies/Unity-Robotics-Hub.git
cd Unity-Robotics-Hub/tutorials/ros_unity_integration
# Follow README instructions for ROS-TCP-Connector setup
```

## 6. NVIDIA Isaac Sim (Module 3 Only)

**Prerequisites**:
- NVIDIA GPU with RTX support
- NVIDIA driver 525+ installed (see Appendix A)
- CUDA 12.1+ installed

**Install Omniverse Launcher**:

```bash
# Download launcher
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# Make executable and run
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage
```

**Install Isaac Sim** (via Omniverse Launcher GUI):
1. Open Omniverse Launcher
2. Navigate to **Exchange** tab
3. Search for **Isaac Sim**
4. Click **Install** (version 2023.1.1 or later)
5. Wait for download (~30GB, may take 1-2 hours)

**Install Isaac ROS**:

```bash
# Install Isaac ROS common packages
sudo apt install -y ros-humble-isaac-ros-common

# Clone Isaac ROS repositories (optional, for advanced usage)
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
cd ~/isaac_ros_ws
colcon build
source install/setup.bash
```

**Verify Isaac Sim**:
1. Open Omniverse Launcher
2. Click **Library** → **Isaac Sim** → **Launch**
3. Wait for Isaac Sim to open (~1-2 minutes)
4. Load sample scene: **File** → **Open** → **Isaac/Samples/ROS2/Turtlebot**
5. Press **Play** to verify physics simulation

## 7. VLA Dependencies (Module 4 Only)

**Install Whisper** (OpenAI speech recognition):

```bash
# Activate virtual environment
source ~/physical-ai-venv/bin/activate

# Install Whisper
pip install -U openai-whisper

# Download model (first run will auto-download)
whisper --model base --language en --help

# Verify
python3 -c "import whisper; print('Whisper OK')"
```

**Install ROS 2 Audio Common**:

```bash
# Install from apt
sudo apt install -y ros-humble-audio-common

# Verify
ros2 pkg list | grep audio_common
```

**API Keys** (for cloud-based LLMs):

If using cloud APIs instead of local models:

```bash
# Create .env file
cd ~/physical-ai-code
cat > .env << EOF
# OpenAI API (for GPT-4 cognitive planning)
OPENAI_API_KEY=sk-your-key-here

# Anthropic API (for Claude cognitive planning)
ANTHROPIC_API_KEY=sk-ant-your-key-here
EOF

# Protect secrets
chmod 600 .env

# Load in Python with python-dotenv (already in requirements.txt)
# from dotenv import load_dotenv; load_dotenv()
```

**Note**: Free alternatives (local LLMs) are provided in Module 4 tutorials.

## Isaac Sim Docker Installation (Alternative)

For environments where native installation is challenging (e.g., remote servers):

```bash
# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER
newgrp docker

# Install NVIDIA Container Toolkit
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/libnvidia-container/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo systemctl restart docker

# Pull Isaac Sim Docker image
docker pull nvcr.io/nvidia/isaac-sim:2023.1.1

# Run Isaac Sim in Docker with GUI support
xhost +local:docker
docker run --name isaac-sim --entrypoint bash -it --gpus all \
  -e "ACCEPT_EULA=Y" \
  -e "DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ~/isaac-sim-data:/root/Documents \
  nvcr.io/nvidia/isaac-sim:2023.1.1

# Inside container, launch Isaac Sim
./runapp.sh
```

**Advantages**:
- No Omniverse Launcher needed
- Easier to version control
- Reproducible across machines
- Ideal for CI/CD pipelines

**Limitations**:
- Larger disk usage (~40GB)
- Requires NVIDIA Docker runtime
- Slightly more complex networking for ROS 2

## 8. NVIDIA Jetson Orin Setup

For deploying AI models to edge devices (real robots):

### Initial Setup

**Flash JetPack OS**:

1. Download NVIDIA SDK Manager on x86 Ubuntu host: https://developer.nvidia.com/sdk-manager
2. Connect Jetson Orin to host via USB-C
3. Put Jetson in recovery mode:
   - Hold RECOVERY button
   - Press POWER button
   - Release RECOVERY after 2 seconds
4. Flash JetPack 6.0 (includes Ubuntu 22.04 + ROS 2 Humble support)

**First Boot Configuration**:

```bash
# SSH into Jetson (find IP with router or `arp -a`)
ssh nvidia@<jetson-ip>  # Default password: nvidia

# Update system
sudo apt update && sudo apt upgrade -y

# Install development tools
sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-pip \
  python3-venv \
  nano

# Install ROS 2 Humble (lighter desktop installation)
sudo apt install -y ros-humble-ros-base ros-humble-perception

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Isaac ROS on Jetson

Isaac ROS provides hardware-accelerated perception:

```bash
# Install Isaac ROS prerequisites
sudo apt install -y \
  ros-humble-isaac-ros-visual-slam \
  ros-humble-isaac-ros-object-detection \
  ros-humble-isaac-ros-depth-segmentation

# Verify
ros2 pkg list | grep isaac_ros

# Test VSLAM node
ros2 run isaac_ros_visual_slam isaac_ros_visual_slam

# Expected: Node starts (may error without camera - that's OK)
```

### Cross-Compilation Workflow

Develop on x86 workstation, deploy to Jetson:

**On Development Machine (x86)**:

```bash
# Install cross-compilation tools
sudo apt install -y \
  qemu-user-static \
  binfmt-support

# Create ROS 2 workspace
mkdir -p ~/jetson_ws/src
cd ~/jetson_ws

# Clone your robot code
cd src
git clone <your-robot-repo>

# Build with colcon (arm64 target)
colcon build --merge-install \
  --cmake-args \
  -DCMAKE_TOOLCHAIN_FILE=~/jetson_ws/src/toolchain-aarch64.cmake
```

**Deploy to Jetson**:

```bash
# From development machine
rsync -avz --exclude 'build' --exclude 'log' \
  ~/jetson_ws/ nvidia@<jetson-ip>:~/robot_ws/

# On Jetson
cd ~/robot_ws
source install/setup.bash
ros2 launch your_package robot_launch.py
```

### Performance Optimization

**Enable MAX-N Mode** (highest performance):

```bash
# Set power mode to MAXN
sudo nvpmodel -m 0

# Set CPU/GPU to max frequency
sudo jetson_clocks

# Verify
sudo jetson_clocks --show
```

**Monitor Resources**:

```bash
# Install jtop (Jetson monitoring tool)
sudo pip3 install -U jetson-stats

# Run monitor
jtop

# Press 1-5 to navigate:
# 1: CPU/GPU usage
# 2: Memory
# 3: Power
# 4: Temperature
```

### Common Jetson Deployment Patterns

**1. Camera Integration (RealSense D435)**:

```bash
# Install RealSense SDK
sudo apt install -y ros-humble-realsense2-camera

# Launch camera node
ros2 launch realsense2_camera rs_launch.py \
  enable_depth:=true \
  enable_color:=true
```

**2. Object Detection (Isaac ROS)**:

```bash
# Launch YOLOv5 on Jetson
ros2 launch isaac_ros_yolov5 isaac_ros_yolov5.launch.py \
  model_path:=/opt/nvidia/isaac_ros_models/yolov5s.onnx
```

**3. VSLAM (Visual Odometry)**:

```bash
# Launch Visual SLAM
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py
```

### Jetson Troubleshooting

**Issue**: Thermal throttling during inference

**Solution**:
```bash
# Add heatsink/fan
# Enable fan (if available)
sudo sh -c 'echo 255 > /sys/devices/pwm-fan/target_pwm'

# Monitor temperature
watch -n 1 'cat /sys/devices/virtual/thermal/thermal_zone*/temp'
```

**Issue**: Insufficient memory for models

**Solution**:
```bash
# Increase swap size
sudo systemctl disable nvzramconfig
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

## 9. Verification Script

Run the environment verification script to check all installations:

```bash
cd ~/physical-ai-code
bash scripts/verify-environment.sh
```

Expected output:
```
=== Core System Tools ===
✓ bash found
✓ git found
✓ python3 found

=== ROS 2 Environment ===
✓ ROS 2 Humble installation found
✓ ros2 found

=== Gazebo Simulation ===
✓ gz (Gazebo Garden) found

=== Python Packages ===
✓ numpy found
✓ opencv-python found
✓ pytest found

=== Summary ===
All checks passed! (0 failures)
```

## Troubleshooting

### ROS 2 Installation Issues

**Problem**: `ros2: command not found`
**Solution**:
```bash
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Gazebo Crashes on Launch

**Problem**: Gazebo crashes with GPU errors
**Solution**:
```bash
# Force software rendering
export LIBGL_ALWAYS_SOFTWARE=1
gz sim
```

### Isaac Sim Won't Launch

**Problem**: Black screen or crash on launch
**Solution**:
1. Check NVIDIA driver version: `nvidia-smi` (should be 525+)
2. Verify GPU is RTX-capable: `lspci | grep -i nvidia`
3. Check Isaac Sim logs: `~/.nvidia-omniverse/logs/Isaac-Sim/`

### Whisper Model Download Fails

**Problem**: `ConnectionError` when downloading models
**Solution**:
```bash
# Manually download model
wget https://openaipublic.azureedge.net/main/whisper/models/base.pt -P ~/.cache/whisper/
```

For more issues, see [Appendix C: Troubleshooting](./troubleshooting.md).

---

**Previous**: [← Appendix A: Hardware Setup](./hardware-setup.md)
**Next**: [Appendix C: Troubleshooting →](./troubleshooting.md)
