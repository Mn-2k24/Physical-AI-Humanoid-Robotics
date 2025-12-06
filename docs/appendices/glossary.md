# Appendix D: Glossary of Terms

Comprehensive glossary of robotics, AI, and course-specific terminology.

---

## A

**Action** (ROS 2): Long-running, preemptable communication pattern with feedback. Used for tasks like navigation or manipulation that take seconds/minutes. See also: Topic, Service.

**Action Space**: The set of all possible actions a robot can take (e.g., joint velocities, end-effector poses). Continuous (infinite precision) or discrete (finite set).

**Actuator**: A mechanical component that produces motion (motors, hydraulics, pneumatics).

**ALOHA (A Low-cost Open-source Hardware System for Bimanual Teleoperation)**: Stanford research platform for imitation learning with dual robot arms.

---

## B

**BibTeX**: Citation management format used with LaTeX. Stores bibliographic entries in `.bib` files.

**Bipedal**: Two-legged locomotion (humans, humanoid robots). Contrast: quadrupedal (four-legged).

---

## C

**Callback**: A function automatically called when an event occurs (e.g., ROS 2 topic message received).

**CenterTrack**: Computer vision algorithm for tracking multiple objects across video frames.

**CLIP (Contrastive Language-Image Pre-training)**: OpenAI model that learns joint embeddings of images and text, enabling zero-shot object recognition.

**Collision Mesh**: Simplified 3D geometry used for physics collision detection (faster than visual mesh). Often boxes, cylinders, or low-poly approximations.

**Configuration Space (C-space)**: The space of all possible robot joint configurations (e.g., 6D for a 6-DOF arm).

**CUDA**: NVIDIA's parallel computing platform for GPU-accelerated applications.

---

## D

**DDS (Data Distribution Service)**: Middleware standard used by ROS 2 for publish-subscribe communication. Implementations: Fast-DDS, Cyclone DDS.

**Depth Map**: 2D image where each pixel value represents distance to the camera (from depth cameras or stereo vision).

**DINO (Self-Distillation with No Labels)**: Meta AI's self-supervised vision transformer for feature extraction.

**DOF (Degrees of Freedom)**: Number of independent ways a robot can move. A humanoid typically has 30-50 DOF (head, arms, torso, legs).

**Domain Randomization**: Training technique where simulation parameters (lighting, textures, physics) are randomly varied to improve sim-to-real transfer.

**DQN (Deep Q-Network)**: Reinforcement learning algorithm that uses neural networks to approximate Q-values (expected future rewards).

---

## E

**Embodied AI**: AI systems that interact with the physical world through sensors and actuators (robots). Contrast: disembodied AI (chatbots, image classifiers).

**End-Effector**: The tool at the end of a robotic arm (gripper, hand, welding torch).

**EKF (Extended Kalman Filter)**: Algorithm for estimating robot state (position, velocity) from noisy sensor measurements.

---

## F

**Flesch-Kincaid Grade Level**: Readability metric based on sentence length and syllable count. Score of 12 means high school senior reading level.

**Forward Kinematics**: Computing end-effector pose from joint angles. Inverse: computing joint angles from desired pose.

---

## G

**Gazebo**: Open-source physics simulator for robotics. Gazebo Classic (11.x) is stable; Gazebo Garden/Harmonic (new architecture) is actively developed.

**Grasping**: Planning and executing a robot grip on an object (requires perception, planning, control).

**Ground Truth**: True data labels for training/evaluation (e.g., human-annotated bounding boxes, known robot positions).

---

## H

**Humanoid**: Robot with human-like body structure (head, torso, two arms, two legs). Examples: Atlas, Digit, Tesla Optimus.

**HRI (Human-Robot Interaction)**: Field studying how humans and robots communicate and collaborate.

---

## I

**IMU (Inertial Measurement Unit)**: Sensor measuring acceleration and rotation rate (used for estimating robot orientation).

**Imitation Learning**: Training robots by demonstrating tasks (teleoperation, kinesthetic teaching). See also: Behavioral cloning.

**Inverse Kinematics (IK)**: Computing joint angles needed to achieve a desired end-effector pose. Often has multiple solutions or no solution.

**Isaac ROS**: NVIDIA's GPU-accelerated ROS 2 perception packages (VSLAM, object detection, depth processing).

**Isaac Sim**: NVIDIA's photorealistic robot simulator built on Omniverse. Supports RTX ray tracing and synthetic data generation.

---

## J

**Joint**: Connection between two robot links allowing relative motion (revolute = rotation, prismatic = translation).

**Joint Space**: Space of all joint angle combinations (e.g., 6D space for a 6-joint arm).

---

## K

**Kinematics**: Study of motion without considering forces. See: Forward kinematics, Inverse kinematics.

**KITTI**: Benchmark dataset for autonomous driving (stereo vision, LiDAR, GPS).

---

## L

**Latency**: Time delay between input and output (critical for real-time control). Acceptable: `less than 10ms` for reactive control, `less than 100ms` for high-level planning.

**LiDAR (Light Detection and Ranging)**: Sensor that measures distance using laser pulses. Produces 3D point clouds.

**Link**: Rigid body segment of a robot (e.g., upper arm, forearm).

**LLM (Large Language Model)**: AI model trained on massive text corpora (GPT-4, Claude, LLaMA). Used for cognitive planning in VLA systems.

**Locomotion**: Movement of a robot from one location to another (walking, wheeled, flying).

---

## M

**Manipulation**: Grasping and moving objects with a robot arm/hand.

**MDP (Markov Decision Process)**: Mathematical framework for sequential decision-making (states, actions, rewards, transitions).

**Mermaid.js**: JavaScript library for creating diagrams from text syntax (flowcharts, sequence diagrams, etc.).

**Mesh**: 3D model composed of vertices, edges, and faces. Visual mesh (high detail for rendering) vs. collision mesh (low detail for physics).

**MPPI (Model Predictive Path Integral)**: Sampling-based control algorithm for trajectory optimization.

---

## N

**Nav2 (Navigation2)**: ROS 2 navigation stack for autonomous mobile robot path planning and obstacle avoidance.

**Node** (ROS 2): Executable process that performs computation (e.g., sensor driver, controller, planner). Nodes communicate via topics/services/actions.

**NVIDIA Jetson**: Embedded GPU platform for edge AI (Jetson Nano, Orin Nano, AGX Orin).

---

## O

**Observation Space**: Set of all possible sensor readings a robot can receive (e.g., camera images, joint angles).

**Odometry**: Estimating robot motion from wheel encoders or visual features. Prone to drift over time.

**Omniverse**: NVIDIA's platform for 3D simulation and collaboration (includes Isaac Sim).

---

## P

**PaLM-E (Pathways Language Model - Embodied)**: Google's multimodal LLM that grounds language in robot sensor data.

**Path Planning**: Computing a collision-free path from start to goal. Algorithms: A*, RRT, Dijkstra.

**PID (Proportional-Integral-Derivative)**: Classic control algorithm for tracking setpoints (e.g., joint angle control).

**Point Cloud**: Set of 3D points representing object surfaces (from LiDAR or depth cameras).

**Pose**: Position + orientation (6D: x, y, z, roll, pitch, yaw).

**PPO (Proximal Policy Optimization)**: State-of-the-art reinforcement learning algorithm (used to train robot policies).

**Publisher** (ROS 2): Node that sends messages on a topic. Subscribers receive the messages.

---

## Q

**QoS (Quality of Service)**: ROS 2 settings for reliability (best-effort vs. reliable), durability (transient vs. volatile), and latency.

**Quaternion**: Mathematical representation of 3D rotation using 4 numbers (avoids gimbal lock, unlike Euler angles).

---

## R

**RealSense**: Intel's family of depth cameras (D435, D455) using stereo vision and structured light.

**RL (Reinforcement Learning)**: Training robots through trial-and-error with reward signals (e.g., PPO, SAC, TD3).

**ROS 2 (Robot Operating System 2)**: Middleware for robot software development. Provides communication (topics, services, actions), tools, and libraries.

**RT-1 (Robotics Transformer 1)**: Google's vision-language-action model trained on 130k robot demonstrations.

**RTX**: NVIDIA's ray tracing technology for realistic lighting and reflections (requires RTX-series GPUs).

---

## S

**SAC (Soft Actor-Critic)**: Reinforcement learning algorithm optimized for continuous action spaces.

**SDF (Simulation Description Format)**: XML format for Gazebo worlds/models (alternative to URDF).

**Semantic Segmentation**: Classifying each pixel in an image by object category (person, car, road, etc.).

**Service** (ROS 2): Synchronous request-reply communication pattern (client sends request, server responds). Used for short operations.

**Sim-to-Real Gap**: Difference between simulated and real-world robot performance due to modeling errors, sensor noise, etc.

**SLAM (Simultaneous Localization and Mapping)**: Building a map while estimating robot position within it. Variants: visual SLAM (cameras), LiDAR SLAM.

**Subscriber** (ROS 2): Node that receives messages published on a topic.

---

## T

**Tensor**: Multi-dimensional array (0D = scalar, 1D = vector, 2D = matrix, 3D+ = tensor). Core data structure in PyTorch/TensorFlow.

**TF (Transform)**: ROS 2 library for tracking coordinate frame relationships (e.g., camera → base_link → world).

**TikZ**: LaTeX package for creating vector graphics and diagrams programmatically.

**Topic** (ROS 2): Named bus for message passing (publish-subscribe). Many-to-many communication (multiple publishers/subscribers).

**Trajectory**: Sequence of states over time (e.g., joint angles at each timestep for a robot motion).

**Transformer** (ML): Neural network architecture using self-attention (basis for GPT, BERT, CLIP).

---

## U

**Unity**: Game engine adapted for robotics simulation and visualization (high-fidelity rendering, physics).

**URDF (Unified Robot Description Format)**: XML format for robot models (links, joints, sensors, visual/collision meshes).

---

## V

**Visual SLAM**: SLAM using cameras instead of LiDAR (ORB-SLAM, Kimera, VINS).

**VLA (Vision-Language-Action)**: AI models that map language commands + visual observations → robot actions (RT-1, RT-2, PaLM-E).

**VRAM**: Video RAM on GPU (used for textures, framebuffers, neural network weights).

**VSLAM**: See Visual SLAM.

---

## W

**Whisper**: OpenAI's speech recognition model (multilingual, robust to noise).

**Workspace**:
1. ROS 2: Directory containing `src/`, `build/`, `install/` for packages (built with `colcon build`).
2. Robot: Reachable volume of end-effector positions.

---

## X

**XACRO (XML Macros)**: Extension of URDF allowing variables, loops, and macros (reduces repetition in robot descriptions).

---

## Y

**YOLO (You Only Look Once)**: Real-time object detection algorithm (YOLOv5, YOLOv8).

---

## Z

**Zero-Shot Learning**: Performing tasks without task-specific training examples (e.g., CLIP recognizing objects never seen during training).

**Zotero**: Open-source reference manager for citations and bibliographies.

---

## Acronym Quick Reference

| Acronym | Full Term | Category |
|---------|-----------|----------|
| ALOHA | A Low-cost Open-source Hardware System for Bimanual Teleoperation | Hardware |
| CLIP | Contrastive Language-Image Pre-training | ML Model |
| DDS | Data Distribution Service | ROS 2 |
| DINO | Self-Distillation with No Labels | ML Model |
| DOF | Degrees of Freedom | Robotics |
| DQN | Deep Q-Network | RL |
| EKF | Extended Kalman Filter | Estimation |
| HRI | Human-Robot Interaction | Field |
| IK | Inverse Kinematics | Kinematics |
| IMU | Inertial Measurement Unit | Sensor |
| LiDAR | Light Detection and Ranging | Sensor |
| LLM | Large Language Model | ML Model |
| MDP | Markov Decision Process | RL |
| MPPI | Model Predictive Path Integral | Control |
| Nav2 | Navigation2 | ROS 2 Stack |
| PID | Proportional-Integral-Derivative | Control |
| PPO | Proximal Policy Optimization | RL |
| QoS | Quality of Service | ROS 2 |
| RL | Reinforcement Learning | ML |
| ROS | Robot Operating System | Middleware |
| RT-1 | Robotics Transformer 1 | ML Model |
| RTX | Ray Tracing (NVIDIA) | Graphics |
| SAC | Soft Actor-Critic | RL |
| SDF | Simulation Description Format | File Format |
| SLAM | Simultaneous Localization and Mapping | Robotics |
| TF | Transform | ROS 2 Library |
| URDF | Unified Robot Description Format | File Format |
| VLA | Vision-Language-Action | ML Model Type |
| VRAM | Video RAM | Hardware |
| VSLAM | Visual SLAM | Robotics |
| XACRO | XML Macros | File Format |
| YOLO | You Only Look Once | ML Model |

---

**Previous**: [← Appendix C: Troubleshooting](./troubleshooting.md)
**Next**: [Appendix E: Resources →](./resources.md)
