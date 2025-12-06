# Appendix C: Troubleshooting Common Issues

This appendix addresses frequently encountered problems across all modules.

## General Troubleshooting Strategy

Before diving into specific issues:

1. **Check system logs**: `journalctl -xe` (system), `dmesg` (kernel), ROS logs (`~/.ros/log/`)
2. **Verify versions**: `ros2 --version`, `gz sim --version`, `nvidia-smi`, `python3 --version`
3. **Search official docs**: ROS 2 Answers, Gazebo Community, NVIDIA Forums
4. **Isolate the problem**: Create minimal reproduction (single node, simple launch file)
5. **Ask for help**: Include error messages, system info, and minimal reproduction steps

---

## ROS 2 Issues

### Problem: `ros2: command not found`

**Symptoms**: Bash cannot find `ros2` command after installation.

**Cause**: ROS 2 environment not sourced.

**Solution**:
```bash
# Source ROS 2 setup (temporary)
source /opt/ros/humble/setup.bash

# Make permanent (add to ~/.bashrc)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify
ros2 --version
```

---

### Problem: `ModuleNotFoundError: No module named 'rclpy'`

**Symptoms**: Python scripts fail with missing `rclpy` import.

**Cause**: Wrong Python interpreter or ROS 2 not in PYTHONPATH.

**Solution**:
```bash
# Check which Python is being used
which python3

# Ensure ROS 2 is sourced
source /opt/ros/humble/setup.bash

# Verify rclpy is available
python3 -c "import rclpy; print('OK')"

# If using virtual environment, reinstall rclpy
pip install --upgrade rclpy
```

---

### Problem: `package 'my_package' not found`

**Symptoms**: `ros2 run my_package my_node` fails with package not found.

**Cause**: Package not built or workspace not sourced.

**Solution**:
```bash
# Navigate to workspace
cd ~/ros2_ws

# Build workspace
colcon build --symlink-install

# Source workspace (AFTER sourcing ROS 2)
source /opt/ros/humble/setup.bash
source install/setup.bash

# Verify package is listed
ros2 pkg list | grep my_package

# If still not found, check package.xml exists
ls src/my_package/package.xml
```

---

## Gazebo Issues

### Problem: Gazebo crashes on launch with GPU error

**Symptoms**: `Segmentation fault` or black screen when launching Gazebo.

**Cause**: GPU driver issues or incompatible graphics card.

**Solution**:

```bash
# Option 1: Force software rendering (slow but stable)
export LIBGL_ALWAYS_SOFTWARE=1
gz sim

# Option 2: Update NVIDIA drivers (if applicable)
ubuntu-drivers devices  # Check recommended driver
sudo apt install nvidia-driver-535  # Install recommended version
sudo reboot

# Option 3: Use Gazebo Classic (more stable)
sudo apt install gazebo11
gazebo  # Launch Gazebo 11
```

---

### Problem: Models not loading in Gazebo

**Symptoms**: Gazebo launches but world appears empty or models missing.

**Cause**: Model paths not configured correctly.

**Solution**:
```bash
# Download Gazebo models
git clone https://github.com/osrf/gazebo_models ~/gazebo_models

# Set model path (add to ~/.bashrc)
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/gazebo_models
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/gazebo_models

# For ROS 2 + Gazebo integration
export GAZEBO_MODEL_PATH=/opt/ros/humble/share/gazebo_plugins/models:$GAZEBO_MODEL_PATH

# Reload environment
source ~/.bashrc
```

---

### Problem: Physics simulation is unstable (robot explodes, falls through floor)

**Symptoms**: Robot model vibrates violently or falls through ground plane.

**Cause**: Insufficient physics solver iterations, timestep too large, or inertia tensor errors.

**Solution**:

**In URDF** (check inertia values):
```xml
<!-- Ensure inertia tensors are realistic -->
<inertial>
  <mass value="1.0"/>
  <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
</inertial>
```

**In Gazebo world file** (increase solver iterations):
```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>  <!-- Smaller timestep -->
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>  <!-- Increase from default 20 -->
    </solver>
  </ode>
</physics>
```

---

## NVIDIA Isaac Sim Issues

### Problem: Isaac Sim won't launch (black screen or crash)

**Symptoms**: Omniverse Launcher shows "Running" but Isaac Sim never opens.

**Cause**: Incompatible GPU, outdated driver, or insufficient VRAM.

**Solution**:

1. **Check GPU compatibility**:
```bash
nvidia-smi  # Verify driver 525+ and GPU is RTX series
lspci | grep -i nvidia  # Check GPU model
```

2. **Update drivers** (if needed):
```bash
sudo apt install nvidia-driver-535
sudo reboot
```

3. **Check Isaac Sim logs**:
```bash
cat ~/.nvidia-omniverse/logs/Isaac-Sim/isaac-sim.log
# Look for errors like "CUDA error", "Vulkan initialization failed"
```

4. **Free up VRAM** (close other GPU applications):
```bash
# Check GPU memory usage
nvidia-smi
# Kill processes using GPU (replace PID)
kill -9 <PID>
```

---

### Problem: Isaac ROS nodes fail with `Failed to import isaac_ros_common`

**Symptoms**: Python scripts importing `isaac_ros_common` fail.

**Cause**: Isaac ROS workspace not sourced or package not built.

**Solution**:
```bash
# Rebuild Isaac ROS workspace
cd ~/isaac_ros_ws
colcon build --symlink-install

# Source workspace (AFTER ROS 2)
source /opt/ros/humble/setup.bash
source install/setup.bash

# Verify package exists
ros2 pkg list | grep isaac_ros
```

---

### Problem: Synthetic data generation is very slow (< 5 FPS)

**Symptoms**: Isaac Sim runs but renders at extremely low FPS during dataset generation.

**Cause**: RTX ray tracing disabled or insufficient VRAM.

**Solution**:

1. **Enable RTX in Isaac Sim**:
   - Open Isaac Sim
   - Go to **Window** → **Render Settings**
   - Set **Renderer** to **Path Traced** (not Ray Traced)
   - Enable **DLSS** (if supported by GPU)

2. **Reduce scene complexity**:
   - Lower resolution: 512x512 instead of 1920x1080
   - Fewer objects in scene
   - Disable shadows/reflections for non-critical objects

---

## Unity Issues

### Problem: Unity fails to connect to ROS 2

**Symptoms**: Unity ROS-TCP-Connector shows "Connection failed" or timeout.

**Cause**: ROS-TCP-Endpoint not running or firewall blocking port.

**Solution**:

1. **Start ROS-TCP-Endpoint**:
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Launch endpoint (default port 10000)
ros2 launch ros_tcp_endpoint endpoint.launch.py
```

2. **Check firewall** (if running on separate machines):
```bash
# Allow port 10000
sudo ufw allow 10000/tcp
```

3. **Verify Unity settings**:
   - In Unity: **Robotics** → **ROS Settings**
   - Set **ROS IP Address** to `127.0.0.1` (local) or ROS machine IP
   - Set **ROS Port** to `10000`

---

## Module 4 (VLA) Issues

### Problem: Whisper model download fails or times out

**Symptoms**: `ConnectionError` or `HTTPError` when running Whisper for first time.

**Cause**: Network issues or corrupted cache.

**Solution**:

1. **Manually download model**:
```bash
# Download base model (74MB)
wget https://openaipublic.azureedge.net/main/whisper/models/base.pt \
  -P ~/.cache/whisper/

# Or download small model (244MB, more accurate)
wget https://openaipublic.azureedge.net/main/whisper/models/small.pt \
  -P ~/.cache/whisper/
```

2. **Clear cache and retry**:
```bash
rm -rf ~/.cache/whisper/
python3 -c "import whisper; whisper.load_model('base')"
```

---

### Problem: No audio input detected (microphone not found)

**Symptoms**: `arecord -l` shows no devices or Python audio libraries fail.

**Cause**: PulseAudio not running or permissions issue.

**Solution**:

1. **Check audio devices**:
```bash
# List capture devices
arecord -l

# If empty, restart PulseAudio
pulseaudio --kill
pulseaudio --start
```

2. **Grant microphone permissions** (Ubuntu):
```bash
# Install PulseAudio control
sudo apt install pavucontrol

# Run GUI and check Input Devices tab
pavucontrol
```

3. **Test recording**:
```bash
# Record 5 seconds
arecord -d 5 -f cd test.wav
aplay test.wav
```

---

### Problem: LLM API calls fail with `401 Unauthorized`

**Symptoms**: Python scripts using OpenAI/Anthropic APIs return authentication errors.

**Cause**: Missing or incorrect API key in `.env` file.

**Solution**:

1. **Check `.env` file exists**:
```bash
cd ~/physical-ai-code
cat .env  # Should show API keys (masked)
```

2. **Verify API key is valid**:
```python
# Test OpenAI key
from openai import OpenAI
client = OpenAI(api_key="sk-your-key-here")
print(client.models.list())  # Should not error
```

3. **Ensure `.env` is loaded** in Python:
```python
from dotenv import load_dotenv
load_dotenv()  # Must be called before importing API clients

import os
print(os.getenv("OPENAI_API_KEY"))  # Should print key (first 8 chars)
```

---

## Sim-to-Real Deployment Issues

### Problem: Jetson Orin not booting after JetPack flash

**Symptoms**: Jetson shows no display output or gets stuck at NVIDIA logo after flashing JetPack.

**Cause**: Corrupted flash, wrong JetPack version for hardware, or incompatible carrier board.

**Solution**:

1. **Re-flash in recovery mode**:
```bash
# Put Jetson in recovery mode (press recovery + power buttons)
lsusb | grep NVIDIA  # Should show "NVIDIA Corp. APX"

# Re-flash with SDK Manager (on Ubuntu host)
# Download from: https://developer.nvidia.com/sdk-manager
sdkmanager --cli --action install --product Jetson --version 5.1.2 \
  --targetos Linux --target JETSON_AGX_ORIN_TARGETS
```

2. **Verify hardware compatibility**:
   - Check Jetson model matches JetPack version
   - Orin Nano requires JetPack 5.1+
   - AGX Orin supports JetPack 5.0+

3. **Use serial console for debugging**:
```bash
# Connect USB-C cable to host machine
screen /dev/ttyACM0 115200
# Look for boot errors in serial output
```

---

### Problem: RealSense camera not detected on Jetson

**Symptoms**: `rs-enumerate-devices` shows no devices or `No device connected` error.

**Cause**: USB 3.0 not enabled, insufficient power, or missing udev rules.

**Solution**:

1. **Check USB connection**:
```bash
# Verify RealSense is connected via USB 3.0 (not 2.0)
lsusb | grep Intel
# Should show: "Intel Corp. RealSense D435"

# Check USB speed
lsusb -t
# Look for "5000M" (USB 3.0) not "480M" (USB 2.0)
```

2. **Install udev rules**:
```bash
# Download RealSense udev rules
cd /tmp
wget https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules

# Install rules
sudo cp 99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# Reconnect camera
```

3. **Check power supply**:
   - RealSense D435 requires 2.5A at 5V
   - Use powered USB hub if Jetson USB ports insufficient
   - Ensure Jetson power adapter is 60W (AGX) or 15W (Nano)

---

### Problem: TensorRT engine runs slower on Jetson than expected

**Symptoms**: TensorRT optimized model still runs at < 10 FPS on Jetson Orin.

**Cause**: Engine not using FP16/INT8 precision, or GPU not in MAX-N mode.

**Solution**:

1. **Enable MAX-N performance mode**:
```bash
# Check current power mode
sudo nvpmodel -q

# Set to MAX-N (highest performance)
sudo nvpmodel -m 0  # Mode 0 = MAXN
sudo jetson_clocks  # Max out clocks

# Verify GPU is at max frequency
sudo tegrastats
```

2. **Re-convert model with lower precision**:
```python
# Ensure FP16 is enabled during TensorRT conversion
config.set_flag(trt.BuilderFlag.FP16)

# For even faster inference, use INT8 (requires calibration)
config.set_flag(trt.BuilderFlag.INT8)
config.int8_calibrator = MyCalibrator()  # Provide calibration data
```

3. **Reduce batch size**:
   - Jetson benefits from batch_size=1 for real-time inference
   - Batching increases throughput but latency

---

### Problem: Domain randomization causes model to learn wrong features

**Symptoms**: Model performs well in simulation but fails on real robot, even with randomization.

**Cause**: Randomization too aggressive, destroying task-relevant features.

**Solution**:

1. **Visualize randomized data**:
```python
# Save sample of randomized images
import matplotlib.pyplot as plt

for i in range(10):
    fig, axes = plt.subplots(1, 2)
    axes[0].imshow(original_image)
    axes[1].imshow(randomized_image)
    plt.savefig(f'randomization_sample_{i}.png')
```

2. **Reduce randomization ranges**:
```yaml
# Start conservative, gradually increase
lighting:
  intensity:
    min: 0.7    # Was 0.5 - less variation
    max: 1.3    # Was 2.0 - less variation
```

3. **Use curriculum learning**:
   - Train first 10k iterations: no randomization
   - Next 10k: mild randomization (±10%)
   - Final 20k: full randomization (±30%)

4. **Check FID score**:
```bash
# FID should be less than 50 between synthetic and real data
python -m pytorch_fid synthetic_images/ real_images/
# If FID greater than 100, randomization is insufficient
```

---

### Problem: `colcon build` fails on Jetson with memory error

**Symptoms**: Build crashes with `c++: fatal error: Killed` or `out of memory`.

**Cause**: Compilation uses too much RAM for Jetson (8-16GB typical).

**Solution**:

1. **Limit parallel jobs**:
```bash
# Use fewer parallel jobs (default uses all cores)
colcon build --symlink-install --parallel-workers 2
```

2. **Add swap space**:
```bash
# Check current swap
free -h

# Add 8GB swap file
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Make permanent
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

3. **Build packages sequentially**:
```bash
# Build one package at a time
colcon build --packages-select my_package --symlink-install
```

---

### Problem: Isaac Replicator fails with `Attribute not found` error

**Symptoms**: Domain randomization script crashes with `RuntimeError: Attribute 'inputs:diffuse_tint' not found`.

**Cause**: Trying to modify non-existent material attribute or wrong shader type.

**Solution**:

1. **Check material type**:
```python
# Inspect material to see available attributes
import omni.usd
stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath("/World/MyObject/Looks/Material")

# Print all attributes
for attr in prim.GetAttributes():
    print(f"{attr.GetName()}: {attr.GetTypeName()}")
```

2. **Use correct attribute names** for OmniPBR shader:
```python
# Correct attributes for OmniPBR
rep.modify.attribute("inputs:diffuse_color_constant", ...)  # Not diffuse_tint
rep.modify.attribute("inputs:metallic_constant", ...)       # Not metallic
rep.modify.attribute("inputs:roughness_constant", ...)      # Not roughness
```

3. **Ensure material is OmniPBR**:
   - Select object in Isaac Sim
   - Property panel → Materials → Check shader type
   - If not OmniPBR, right-click → Convert to OmniPBR

---

### Problem: Whisper runs on CPU instead of GPU (very slow transcription)

**Symptoms**: Speech recognition takes 5-10 seconds per utterance instead of real-time.

**Cause**: PyTorch not detecting CUDA or Whisper not configured for GPU.

**Solution**:

1. **Verify CUDA is available**:
```python
import torch
print(f"CUDA available: {torch.cuda.is_available()}")
print(f"CUDA device: {torch.cuda.get_device_name(0)}")
```

2. **Explicitly specify GPU device** when loading Whisper:
```python
import whisper

# Force GPU
model = whisper.load_model("base", device="cuda")

# Verify device
print(f"Model device: {next(model.parameters()).device}")
# Should print: "cuda:0" not "cpu"
```

3. **Install correct PyTorch version** with CUDA support:
```bash
# Uninstall CPU-only version
pip uninstall torch

# Install CUDA 11.8 version
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

---

### Problem: Cognitive planner generates invalid action sequences

**Symptoms**: LLM planner outputs actions that violate safety constraints or have impossible parameters.

**Cause**: Insufficient prompt engineering or missing action library constraints.

**Solution**:

1. **Improve system prompt** with explicit constraints:
```python
system_prompt = """You are a robot action planner. Generate ONLY valid actions from this library:

Available Actions:
- move_to(x, y, z): Move end-effector to position
  Constraints: x in [-1.0, 1.0], y in [-1.0, 1.0], z in [0.0, 1.5]
- grasp(): Close gripper
  Constraints: Must be preceded by move_to within 5cm of object
- release(): Open gripper

Safety Rules:
- Never move to z < 0 (below table)
- Always verify object detected before grasp
- Maximum 10 actions per sequence

Output Format: JSON array of actions.
"""
```

2. **Validate LLM output** before execution:
```python
from action_sequence import ActionSequenceGenerator
from safety_validator import SafetyValidator

validator = SafetyValidator()
actions = parse_llm_response(response)

# Validate each action
is_safe, violations = validator.validate_sequence(actions)
if not is_safe:
    print(f"Unsafe sequence: {violations}")
    # Retry with violations in prompt
```

3. **Use few-shot examples** in prompt:
```python
examples = """
Example 1:
User: "Pick up the red cube"
Output: [
  {"action": "move_to", "target": "red_cube", "offset": [0, 0, 0.1]},
  {"action": "grasp"},
  {"action": "move_to", "position": [0.5, 0, 0.5]}
]
"""
# Append examples to system prompt
```

---

### Problem: VLA action executor fails with `Action timeout exceeded`

**Symptoms**: Actions hang indefinitely or timeout after 30 seconds without completion.

**Cause**: ROS action server not responding or action goal unreachable.

**Solution**:

1. **Check action server is running**:
```bash
# List active action servers
ros2 action list

# Should show: /robot_controller/follow_joint_trajectory
# If missing, launch controller
```

2. **Increase timeout** for slow actions:
```python
# In action_executor.py
future = self.action_client.send_goal_async(
    goal,
    feedback_callback=self.feedback_callback
)

# Wait with longer timeout (default 30s may be insufficient)
rclpy.spin_until_future_complete(
    self.node,
    future,
    timeout_sec=60.0  # Increase to 60s
)
```

3. **Add pre-execution validation**:
```python
# Check goal is reachable before sending
def is_reachable(self, joint_positions):
    """Verify joint limits and IK solution exists."""
    for i, pos in enumerate(joint_positions):
        if not (self.joint_limits[i][0] <= pos <= self.joint_limits[i][1]):
            return False
    return True
```

---

## Performance Issues

### Problem: Simulation is very laggy (< 10 FPS)

**Symptoms**: Gazebo or Isaac Sim runs slower than real-time.

**Causes and Solutions**:

1. **Insufficient CPU/GPU**:
   - Reduce physics update rate in Gazebo
   - Lower render quality in Isaac Sim (disable RTX)
   - Close other applications

2. **Complex models**:
   - Simplify collision meshes (use boxes/cylinders instead of meshes)
   - Reduce polygon count in visual meshes
   - Disable shadows for non-critical objects

3. **Too many sensors**:
   - Reduce camera resolution (640x480 instead of 1920x1080)
   - Lower LiDAR scan frequency (10 Hz instead of 30 Hz)

---

## Getting Help

If your issue isn't covered here:

1. **Search official forums**:
   - [ROS 2 Answers](https://answers.ros.org/)
   - [Gazebo Community](https://community.gazebosim.org/)
   - [NVIDIA Isaac Sim Forums](https://forums.developer.nvidia.com/c/simulation/isaac-sim/)

2. **File an issue** in the companion code repository:
   - [GitHub Issues](https://github.com/nizam/physical-ai-code/issues)
   - Include: error message, system info (`uname -a`, `nvidia-smi`), minimal reproduction steps

3. **Ask on Discord/Slack** (if available):
   - Physical AI community channels
   - ROS Discord server

---

**Previous**: [← Appendix B: Software Installation](./software-installation.md)
**Next**: [Appendix D: Glossary →](./glossary.md)
