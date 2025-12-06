---
sidebar_position: 5
---

# Tutorial 3: Isaac Visual SLAM Demo

**Estimated Time**: 2-3 hours
**Difficulty**: Intermediate
**Prerequisites**: Completed Tutorials 1 and 2, Isaac Sim installed

## Learning Goals

By the end of this tutorial, you will be able to:
- Set up a photorealistic Isaac Sim environment for VSLAM testing
- Configure and run Isaac ROS Visual SLAM (cuVSLAM) on a humanoid robot
- Integrate stereo cameras with IMU for robust localization
- Visualize SLAM output (trajectory, map, landmarks) in RViz
- Validate VSLAM accuracy and troubleshoot common issues
- Navigate a humanoid robot using VSLAM-based odometry

## Overview

This tutorial demonstrates **complete Visual SLAM pipeline** using NVIDIA Isaac ROS on a simulated humanoid robot. You'll:

1. Create an Isaac Sim scene with a humanoid robot and stereo camera
2. Configure Isaac ROS Visual SLAM with IMU fusion
3. Teleoperate the robot to build a map
4. Visualize SLAM performance metrics
5. Use VSLAM odometry for autonomous navigation

**Architecture**:
```mermaid
graph LR
    subgraph "Isaac Sim"
        Robot[Humanoid Robot]
        Camera[Stereo Camera]
        IMU[IMU Sensor]
        Env[Warehouse Scene]
    end

    subgraph "ROS 2 Bridge"
        Images[/stereo/left/right]
        ImuData[/imu/data]
    end

    subgraph "Isaac ROS VSLAM"
        cuVSLAM[cuVSLAM Node]
        Odom[Odometry Output]
        Map[3D Landmarks]
    end

    subgraph "Visualization"
        RViz[RViz2]
        Metrics[Performance Monitor]
    end

    Robot --> Camera
    Robot --> IMU
    Camera --> Images
    IMU --> ImuData

    Images --> cuVSLAM
    ImuData --> cuVSLAM

    cuVSLAM --> Odom
    cuVSLAM --> Map

    Odom --> RViz
    Map --> RViz
    cuVSLAM --> Metrics

    style cuVSLAM fill:#76b947
    style RViz fill:#1a73e8
```

## Prerequisites

### Hardware Requirements

- **GPU**: NVIDIA RTX 2060 or higher (RTX 3060+ recommended)
- **VRAM**: 8GB minimum, 12GB+ recommended
- **RAM**: 16GB minimum
- **OS**: Ubuntu 22.04 LTS

### Software Requirements

**Verify installations**:
```bash
# 1. ROS 2 Humble
ros2 --version
# Expected: ros2 cli version: 0.25.x

# 2. Isaac Sim (installed via Omniverse Launcher or Docker)
ls ~/.local/share/ov/pkg/ | grep isaac_sim
# Expected: isaac_sim-2023.1.1

# 3. Isaac ROS Visual SLAM
ros2 pkg list | grep isaac_ros_visual_slam
# Expected: isaac_ros_visual_slam

# 4. GPU driver
nvidia-smi
# Expected: Driver Version 525.60.11 or later
```

**If missing, install**:
```bash
# Install Isaac ROS Visual SLAM
sudo apt update
sudo apt install ros-humble-isaac-ros-visual-slam \
                 ros-humble-isaac-ros-common

# Install visualization tools
sudo apt install ros-humble-rviz2 \
                 ros-humble-rqt-image-view
```

## Step 1: Create Isaac Sim Scene

We'll create a warehouse environment with a humanoid robot equipped with stereo cameras and IMU.

### 1.1 Launch Isaac Sim

```bash
# Option 1: Omniverse Launcher installation
~/.local/share/ov/pkg/isaac_sim-2023.1.1/isaac-sim.sh

# Option 2: Docker
docker run --gpus all -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  nvcr.io/nvidia/isaac-sim:2023.1.1
```

### 1.2 Load Tutorial Scene

**Method A: Load Pre-Built Scene** (Recommended):

1. In Isaac Sim, go to **File → Open**
2. Navigate to tutorial repository:
   ```
   physical-ai-code/tutorials/module-3-isaac/01-isaac-vslam/scene/vslam_demo.usd
   ```
3. Click **Open**

**Method B: Build Scene from Scratch**:

Follow these steps in Isaac Sim GUI:

1. **Create Environment**:
   - Go to **Isaac Utils → Environments → Simple Warehouse**
   - A warehouse scene will load with pallets, shelves, and floor

2. **Add Humanoid Robot**:
   - Go to **Content Browser** → **Isaac** → **Robots**
   - Drag `Carter_v1.usd` into the viewport (placeholder for humanoid)
   - Position at origin: `(0, 0, 0)`

3. **Add Stereo Camera**:
   - Right-click in **Stage** panel → **Create → Camera**
   - Rename to `left_camera`
   - Set transform:
     - Position: `(0.05, 0.032, 1.5)` (1.5m high, 6.4cm baseline)
     - Rotation: `(0, 0, 0)`
   - Duplicate camera → Rename to `right_camera`
   - Set position: `(0.05, -0.032, 1.5)`

4. **Add IMU**:
   - Select robot in Stage panel
   - Go to **Property Panel → Add → Isaac Sensor → IMU Sensor**
   - Name: `imu`
   - Linear acceleration noise: `0.001`
   - Angular velocity noise: `0.001`

5. **Configure ROS 2 Bridge**:
   - Window → Extensions → Isaac ROS Bridge
   - Enable `omni.isaac.ros2_bridge`

6. **Save Scene**:
   - File → Save As → `vslam_demo.usd`

### 1.3 Verify Scene Setup

Click **Play** (▶) button in Isaac Sim. You should see:
- Warehouse environment rendered
- Robot model visible
- No errors in console

## Step 2: Configure Isaac ROS Visual SLAM

### 2.1 Create Launch File

The launch file configures cuVSLAM and connects it to Isaac Sim sensors.

**File**: `physical-ai-code/tutorials/module-3-isaac/01-isaac-vslam/launch/vslam_launch.py`

See the complete launch file in the companion code repository. Key parameters:

```python
# Camera configuration
'denoise_input_images': True,
'rectified_images': True,

# IMU fusion (critical for accuracy)
'enable_imu_fusion': True,
'gyro_noise_density': 0.000244,
'accel_noise_density': 0.001862,

# SLAM features
'enable_localization_n_mapping': True,
'enable_loop_closure': True,
'max_features_per_frame': 1200,

# Frames
'map_frame': 'map',
'odom_frame': 'odom',
'base_frame': 'base_link',
```

### 2.2 Create Parameter File

**File**: `physical-ai-code/tutorials/module-3-isaac/01-isaac-vslam/config/vslam_params.yaml`

This file contains fine-tuned parameters for the warehouse environment. Key settings:

```yaml
visual_slam:
  # Feature detection
  max_features_per_frame: 1200
  min_num_images: 4

  # Loop closure (reduces drift)
  enable_loop_closure: true
  loop_closure_frequency: 5.0

  # Visualization
  enable_slam_visualization: true
  path_max_size: 1024
```

## Step 3: Run Visual SLAM

### 3.1 Terminal Setup

Open **4 terminals** and source ROS 2 in each:

```bash
# In each terminal
source /opt/ros/humble/setup.bash
cd ~/physical-ai-code/tutorials/module-3-isaac/01-isaac-vslam
```

### 3.2 Launch Isaac Sim with ROS 2 Bridge

**Terminal 1**:
```bash
# Launch Isaac Sim
~/.local/share/ov/pkg/isaac_sim-2023.1.1/isaac-sim.sh

# In Isaac Sim:
# 1. File → Open → vslam_demo.usd
# 2. Click Play (▶)
# 3. Verify ROS 2 topics:
```

**Verify Topics** (Terminal 2):
```bash
ros2 topic list

# Expected topics:
# /camera/left/image_raw
# /camera/left/camera_info
# /camera/right/image_raw
# /camera/right/camera_info
# /imu/data
# /tf
# /tf_static
```

### 3.3 Launch Visual SLAM

**Terminal 2**:
```bash
ros2 launch vslam_launch.py

# Expected output:
# [visual_slam]: Visual SLAM node started
# [visual_slam]: Waiting for stereo images...
# [visual_slam]: Waiting for IMU data...
# [visual_slam]: Initialization complete
# [visual_slam]: Tracking: 1245 features
```

**Monitor VSLAM Status**:
```bash
# Terminal 3: Check odometry output
ros2 topic echo /visual_slam/tracking/odometry --field pose.pose.position

# Expected output (robot moving):
# x: 0.123
# y: 0.045
# z: 0.0
# ---
# x: 0.234  # Position updates as robot moves
# y: 0.067
# z: 0.0
```

### 3.4 Visualize in RViz

**Terminal 3**:
```bash
ros2 run rviz2 rviz2

# In RViz:
# 1. Set Fixed Frame → "map"
# 2. Add displays:
#    - Add → TF (show all frames)
#    - Add → Odometry → Topic: /visual_slam/tracking/odometry
#    - Add → Path → Topic: /visual_slam/tracking/slam_path
#    - Add → MarkerArray → Topic: /visual_slam/vis/landmarks_cloud
#    - Add → Image → Topic: /camera/left/image_raw
```

**Expected Visualization**:
- **Green trajectory**: Robot path from VSLAM
- **Blue points**: 3D landmarks (map features)
- **TF frames**: `map` → `odom` → `base_link` → `camera_link`
- **Camera image**: Live stereo feed (bottom panel)

## Step 4: Teleoperate and Build Map

### 4.1 Keyboard Teleoperation

**Terminal 4**:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Controls:
# i - forward
# k - stop
# j - turn left
# l - turn right
# u/o - forward + turn
# m/. - backward + turn
```

### 4.2 Mapping Strategy

To build a good map:

1. **Start slow** (speed: 0.2 m/s): VSLAM needs time to extract features
2. **Move forward 2 meters**: Build initial map
3. **Turn 90° left**: Add perpendicular features (reduces drift)
4. **Move forward 2 meters**: Expand map
5. **Turn 90° left again**: Complete square pattern
6. **Return to start**: Trigger loop closure

**Watch RViz** while teleoperating:
- **Landmarks**: Blue points should accumulate as you move
- **Trajectory**: Green path should be smooth
- **Loop Closure**: When returning to start, trajectory should "snap" to close the loop

### 4.3 Validate VSLAM Performance

Monitor tracking quality:

```bash
# Check VSLAM status
ros2 topic echo /visual_slam/status

# Key fields:
# tracking_status: TRACKING  # Good
# num_observations: 1200     # High is better
# inlier_ratio: 0.85         # greater than 0.7 is good

# Measure odometry accuracy (return to start position)
ros2 topic echo /visual_slam/tracking/odometry --field pose.pose.position

# After completing square loop:
# x: 0.05   # Should be close to 0.0 (good loop closure)
# y: -0.03
# z: 0.0
```

**Performance Metrics**:
- **Good**: `inlier_ratio > 0.7`, `num_observations > 800`, trajectory smooth
- **Poor**: `inlier_ratio < 0.5`, `num_observations < 300`, trajectory jittery

## Step 5: Autonomous Navigation with VSLAM

Now use VSLAM odometry for autonomous navigation.

### 5.1 Launch Nav2 Stack

**Terminal 2** (stop VSLAM if running, then):
```bash
# Launch Nav2 with VSLAM odometry
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=../../../examples/navigation/nav2_humanoid_params.yaml \
  use_sim_time:=false
```

### 5.2 Set Initial Pose

In RViz:
1. Click **"2D Pose Estimate"** button (top toolbar)
2. Click robot's current position on map
3. Drag to set orientation
4. VSLAM will align `map` → `odom` transform

### 5.3 Send Navigation Goal

In RViz:
1. Click **"Nav2 Goal"** button
2. Click destination on map
3. Drag to set goal orientation
4. Robot should plan path and navigate autonomously

**Watch for**:
- **Global Plan**: Blue line from robot to goal
- **Local Plan**: Red line for immediate path
- **Costmap**: Gray obstacles, inflated margins
- **Robot Motion**: Smooth movement along path

## Step 6: Verification and Testing

### 6.1 Automated Verification Script

Run the verification script to check VSLAM is working correctly:

```bash
cd ~/physical-ai-code/tutorials/module-3-isaac/01-isaac-vslam
python3 verify.py

# Output:
# ✓ ROS 2 is running
# ✓ Isaac Sim topics active
# ✓ Visual SLAM node detected
# ✓ Odometry publishing (30 Hz)
# ✓ TF tree valid (map → odom → base_link)
# ✓ Features tracking (1245 features)
# ✓ Loop closure enabled
#
# VSLAM Status: HEALTHY
# All checks passed! ✓
```

### 6.2 Manual Verification Checklist

- [ ] Isaac Sim scene loads without errors
- [ ] Stereo camera images publish at 30 Hz
- [ ] IMU data publish at 200 Hz
- [ ] cuVSLAM tracks greater than 800 features consistently
- [ ] Odometry updates smoothly (no jumps)
- [ ] Loop closure triggers when revisiting locations
- [ ] RViz shows trajectory and landmarks
- [ ] Nav2 can plan and execute paths using VSLAM odometry

### 6.3 Performance Benchmarks

**Expected Performance** (RTX 3060):

| Metric | Target | Measured |
|--------|--------|----------|
| VSLAM latency | &lt;50ms | ✓ 35ms |
| Odometry rate | 30 Hz | ✓ 30.2 Hz |
| Feature count | &gt;800 | ✓ 1245 |
| Inlier ratio | &gt;0.7 | ✓ 0.87 |
| Loop closure delay | &lt;500ms | ✓ 320ms |
| GPU utilization | 40-60% | ✓ 52% |

**Measure Performance**:
```bash
# Odometry rate
ros2 topic hz /visual_slam/tracking/odometry

# Latency
ros2 topic delay /camera/left/image_raw /visual_slam/tracking/odometry

# GPU usage
nvidia-smi dmon -s u -c 10
```

## Troubleshooting

### Issue 1: cuVSLAM Not Tracking

**Symptoms**:
- `tracking_status: LOST`
- `num_observations: 0`
- No landmarks in RViz

**Causes and Solutions**:

1. **Insufficient visual features** (blank walls, uniform textures):
   ```bash
   # Solution: Move to textured area or add visual markers
   # In Isaac Sim, add objects to scene for more features
   ```

2. **Camera not publishing**:
   ```bash
   # Check topics
   ros2 topic hz /camera/left/image_raw

   # If 0 Hz, verify Isaac Sim ROS 2 bridge is enabled:
   # Window → Extensions → Isaac ROS Bridge → Enable
   ```

3. **IMU fusion disabled**:
   ```yaml
   # In vslam_params.yaml, ensure:
   enable_imu_fusion: true
   ```

### Issue 2: High Drift / Poor Loop Closure

**Symptoms**:
- Loop doesn't close (greater than 0.5m error when returning to start)
- Trajectory drifts over time

**Solutions**:

1. **Enable loop closure**:
   ```yaml
   enable_loop_closure: true
   loop_closure_frequency: 5.0  # Check every 5 seconds
   ```

2. **Increase feature count**:
   ```yaml
   max_features_per_frame: 1500  # Increase from 1200
   ```

3. **Move slower**: High speed degrades feature matching
   ```bash
   # Reduce teleop speed to 0.2 m/s
   ```

4. **IMU calibration**: Poor IMU can cause drift
   ```yaml
   calibration_frequency: 200.0  # Hz
   gyro_noise_density: 0.000244  # Tune for your IMU
   ```

### Issue 3: GPU Out of Memory

**Symptoms**:
- `CUDA out of memory` error
- Isaac Sim crashes

**Solutions**:

1. **Close other GPU applications** (browsers, games)

2. **Reduce Isaac Sim quality**:
   ```bash
   # Launch with reduced rendering
   isaac-sim.sh --/renderer/pathTracingSamplesPerPixel=1
   ```

3. **Use headless mode**:
   ```python
   # In launch file
   simulation_app = SimulationApp({"headless": True})
   ```

4. **Upgrade GPU**: Minimum 8GB VRAM, recommended 12GB+

### Issue 4: TF Transform Errors

**Symptoms**:
- `"map" → "odom" transform timeout`
- RViz shows "No TF data"

**Solutions**:

1. **Check VSLAM is publishing TF**:
   ```bash
   ros2 run tf2_ros tf2_echo map odom

   # Expected: Transform updates at 30 Hz
   ```

2. **Verify frame names match**:
   ```yaml
   # In vslam_params.yaml
   map_frame: 'map'
   odom_frame: 'odom'
   base_frame: 'base_link'
   ```

3. **Enable TF publishing**:
   ```yaml
   publish_map_to_odom_tf: true
   publish_odom_to_base_tf: true
   ```

### Issue 5: Isaac Sim ROS 2 Topics Not Publishing

**Symptoms**:
- `ros2 topic list` shows no Isaac Sim topics
- VSLAM waits indefinitely for images

**Solutions**:

1. **Enable ROS 2 Bridge in Isaac Sim**:
   - Window → Extensions
   - Search "ros2_bridge"
   - Enable checkbox
   - Restart Isaac Sim

2. **Check ROS_DOMAIN_ID**:
   ```bash
   # Both Isaac Sim and ROS 2 must use same domain
   export ROS_DOMAIN_ID=0
   echo $ROS_DOMAIN_ID
   ```

3. **Verify network settings**:
   ```bash
   # Check ROS 2 discovery
   ros2 doctor

   # Ensure multicast is enabled
   ```

## Learning Checkpoint

You have successfully completed Tutorial 3 if you can:

- ✅ Create Isaac Sim scene with stereo camera and IMU
- ✅ Launch cuVSLAM and achieve tracking status
- ✅ Teleoperate robot and build consistent map
- ✅ Observe loop closure when revisiting locations
- ✅ Visualize VSLAM trajectory and landmarks in RViz
- ✅ Use VSLAM odometry for Nav2 autonomous navigation
- ✅ Troubleshoot common VSLAM issues

**Next Steps**:
- Experiment with different environments (outdoor, cluttered)
- Tune VSLAM parameters for your specific use case
- Integrate VSLAM with object detection (Chapter 11)
- Proceed to Module 4: Vision-Language-Action models

## Additional Resources

- [Isaac ROS Visual SLAM Docs](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/index.html)
- [cuVSLAM Technical Paper](https://developer.nvidia.com/isaac-ros-visual-slam)
- [RViz User Guide](http://wiki.ros.org/rviz/UserGuide)
- [Companion Code Repository](https://github.com/nizam/physical-ai-code)

## References

[1] NVIDIA Corporation, "Isaac ROS Visual SLAM: GPU-Accelerated Visual Odometry," NVIDIA Isaac ROS Documentation, 2023. [Online]. Available: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/

[2] Mur-Artal, R., and Tardós, J. D., "ORB-SLAM2: An Open-Source SLAM System for Monocular, Stereo, and RGB-D Cameras," *IEEE Transactions on Robotics*, vol. 33, no. 5, pp. 1255-1262, 2017. DOI: 10.1109/TRO.2017.2705103

---

**Congratulations!** You've completed Module 3 and can now deploy GPU-accelerated perception and navigation for humanoid robots.

**Previous**: [← Chapter 12: Nav2 Navigation](./12-nav2-navigation.md)
**Next**: [Module 4: Vision-Language-Action Integration →](../module-4-vla/index.md)
