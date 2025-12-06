# Appendix E: Additional Resources

Curated collection of documentation, tutorials, papers, and communities for continued learning.

---

## Official Documentation

### ROS 2
- **ROS 2 Humble Docs**: [https://docs.ros.org/en/humble/](https://docs.ros.org/en/humble/)
- **ROS 2 Tutorials**: [https://docs.ros.org/en/humble/Tutorials.html](https://docs.ros.org/en/humble/Tutorials.html)
- **ROS 2 Design**: [https://design.ros2.org/](https://design.ros2.org/)
- **rclpy API Reference**: [https://docs.ros2.org/humble/api/rclpy/](https://docs.ros2.org/humble/api/rclpy/)

### Simulation
- **Gazebo Documentation**: [https://gazebosim.org/docs](https://gazebosim.org/docs)
- **Unity Robotics Hub**: [https://github.com/Unity-Technologies/Unity-Robotics-Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- **Isaac Sim Manual**: [https://docs.omniverse.nvidia.com/isaacsim/latest/](https://docs.omniverse.nvidia.com/isaacsim/latest/)
- **Isaac ROS**: [https://nvidia-isaac-ros.github.io/](https://nvidia-isaac-ros.github.io/)

### Navigation & Perception
- **Nav2 Documentation**: [https://navigation.ros.org/](https://navigation.ros.org/)
- **MoveIt 2 (Motion Planning)**: [https://moveit.ros.org/](https://moveit.ros.org/)
- **ORB-SLAM3**: [https://github.com/UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)

### AI & ML
- **OpenAI Whisper**: [https://github.com/openai/whisper](https://github.com/openai/whisper)
- **Hugging Face Transformers**: [https://huggingface.co/docs/transformers](https://huggingface.co/docs/transformers)
- **PyTorch**: [https://pytorch.org/docs/](https://pytorch.org/docs/)
- **RT-1 Paper**: [https://robotics-transformer.github.io/](https://robotics-transformer.github.io/)

---

## Online Courses & Tutorials

### ROS 2
1. **The Construct (ROS 2 Basics)**
   [https://www.theconstructsim.com/](https://www.theconstructsim.com/)
   Interactive browser-based ROS 2 tutorials with simulated robots.

2. **Articulated Robotics (YouTube)**
   [https://www.youtube.com/@ArticulatedRobotics](https://www.youtube.com/@ArticulatedRobotics)
   Excellent video series on ROS 2, URDF, and robot bringup.

3. **ROS 2 For Beginners (Udemy)**
   Comprehensive course covering nodes, topics, services, and launch files.

### Robotics Fundamentals
1. **Modern Robotics (Northwestern University)**
   Free textbook + Coursera course on kinematics, dynamics, and control.
   [http://modernrobotics.org/](http://modernrobotics.org/)

2. **Underactuated Robotics (MIT)**
   Russ Tedrake's course on dynamics and control (includes humanoid locomotion).
   [https://underactuated.mit.edu/](https://underactuated.mit.edu/)

3. **Introduction to Robotics (Stanford CS223A)**
   Classic course on kinematics, dynamics, and planning.

### Machine Learning for Robotics
1. **Deep Reinforcement Learning (Berkeley CS285)**
   State-of-the-art RL course covering PPO, SAC, model-based RL.
   [https://rail.eecs.berkeley.edu/deeprlcourse/](https://rail.eecs.berkeley.edu/deeprlcourse/)

2. **Imitation Learning (Stanford CS326)**
   Behavioral cloning, inverse RL, and learning from demonstrations.

3. **Vision-Language Models (Hugging Face Course)**
   CLIP, BLIP, and multimodal transformers for robotics.
   [https://huggingface.co/learn](https://huggingface.co/learn)

---

## Research Papers (Essential Reading)

### Simulation & Sim-to-Real
1. **"Sim-to-Real Transfer in Robotics: An Overview"** (2021)
   Comprehensive survey of domain randomization and sim-to-real techniques.

2. **"Learning Dexterous In-Hand Manipulation"** (OpenAI, 2019)
   Training robot hand in simulation with massive domain randomization.

3. **"Isaac Gym: High Performance GPU-Based Physics Simulation"** (NVIDIA, 2021)
   GPU-accelerated parallel simulation for reinforcement learning.

### Vision-Language-Action
1. **"RT-1: Robotics Transformer for Real-World Control at Scale"** (Google, 2022)
   Vision-language-action model trained on 130k demonstrations.

2. **"PaLM-E: An Embodied Multimodal Language Model"** (Google, 2023)
   LLM grounded in robot sensor data for cognitive planning.

3. **"Open X-Embodiment: Robotic Learning Datasets and RT-X Models"** (2023)
   Largest public robot dataset (1M+ trajectories, 22 robot types).

### Humanoid Robotics
1. **"Learning to Walk in Minutes Using Massively Parallel Deep RL"** (ETH Zurich, 2022)
   Training quadruped/bipedal locomotion in Isaac Gym.

2. **"Whole-Body MPC for Humanoid Robots"** (MIT, 2019)
   Model predictive control for Atlas robot.

3. **"Digit: A Bipedal Robot for Logistics"** (Agility Robotics, 2021)
   Commercial humanoid design and deployment insights.

---

## Books

### Robotics Fundamentals
1. **"Modern Robotics: Mechanics, Planning, and Control"**
   Kevin Lynch & Frank Park (2017)
   Comprehensive textbook with code examples (Python, MATLAB).

2. **"Introduction to Autonomous Mobile Robots"**
   Siegwart, Nourbakhsh, Scaramuzza (2011)
   Classic reference on wheeled robots, localization, and mapping.

3. **"Probabilistic Robotics"**
   Thrun, Burgard, Fox (2005)
   The "bible" of robot perception and SLAM.

### Deep Learning & Reinforcement Learning
1. **"Deep Learning"**
   Goodfellow, Bengio, Courville (2016)
   Foundational textbook on neural networks and transformers.

2. **"Reinforcement Learning: An Introduction"**
   Sutton & Barto (2nd edition, 2018)
   Free online textbook covering MDP, Q-learning, policy gradients.

3. **"Grokking Deep Reinforcement Learning"**
   Miguel Morales (2020)
   Practical guide to implementing RL algorithms in PyTorch.

---

## Datasets

### Robot Manipulation
1. **Open X-Embodiment**
   [https://robotics-transformer.github.io/datasets.html](https://robotics-transformer.github.io/datasets.html)
   1M+ robot trajectories across 22 robot types (ALOHA, Franka, UR5).

2. **RLBench**
   [https://github.com/stepjam/RLBench](https://github.com/stepjam/RLBench)
   100 manipulation tasks in simulation for RL/imitation learning.

### Perception
1. **COCO (Common Objects in Context)**
   [https://cocodataset.org/](https://cocodataset.org/)
   330k images with object detection and segmentation annotations.

2. **ImageNet**
   [https://www.image-net.org/](https://www.image-net.org/)
   14M images across 20k categories (used to pretrain vision models).

3. **KITTI Vision Benchmark**
   [https://www.cvlibs.net/datasets/kitti/](https://www.cvlibs.net/datasets/kitti/)
   Autonomous driving dataset with stereo images, LiDAR, GPS.

---

## Communities & Forums

### ROS 2
- **ROS Discourse**: [https://discourse.ros.org/](https://discourse.ros.org/)
  Official forum for ROS 2 questions and announcements.
- **ROS Answers**: [https://answers.ros.org/](https://answers.ros.org/)
  Q&A site for ROS 1/2 (StackOverflow-style).
- **ROS Discord**: [https://discord.gg/ros](https://discord.gg/ros)
  Real-time chat with ROS community.

### Simulation
- **Gazebo Community**: [https://community.gazebosim.org/](https://community.gazebosim.org/)
  Official forum for Gazebo Classic and new Gazebo (Garden, Harmonic).
- **NVIDIA Isaac Sim Forums**: [https://forums.developer.nvidia.com/c/simulation/isaac-sim/](https://forums.developer.nvidia.com/c/simulation/isaac-sim/)
  Official support forum for Isaac Sim and Isaac ROS.

### Machine Learning
- **Hugging Face Discord**: [https://discord.gg/hugging-face](https://discord.gg/hugging-face)
  Discuss transformers, CLIP, and multimodal models.
- **OpenAI Community**: [https://community.openai.com/](https://community.openai.com/)
  Support for Whisper, GPT-4, and API usage.

### Robotics Research
- **r/robotics (Reddit)**: [https://www.reddit.com/r/robotics/](https://www.reddit.com/r/robotics/)
  Active community for hobbyists and professionals.
- **Robotics Stack Exchange**: [https://robotics.stackexchange.com/](https://robotics.stackexchange.com/)
  Q&A for robotics algorithms and hardware.

---

## Tools & Libraries

### Visualization
- **RViz2**: ROS 2's 3D visualization tool (included with `ros-humble-desktop`).
- **Foxglove Studio**: Modern alternative to RViz with web interface.
  [https://foxglove.dev/](https://foxglove.dev/)
- **PlotJuggler**: Real-time plotting for ROS 2 topics.
  [https://github.com/facontidavide/PlotJuggler](https://github.com/facontidavide/PlotJuggler)

### Robot Modeling
- **Blender**: Free 3D modeling software (export to URDF with plugins).
  [https://www.blender.org/](https://www.blender.org/)
- **Onshape**: Cloud-based CAD with URDF export.
  [https://www.onshape.com/](https://www.onshape.com/)
- **sw2urdf**: SolidWorks to URDF exporter.
  [http://wiki.ros.org/sw_urdf_exporter](http://wiki.ros.org/sw_urdf_exporter)

### Motion Planning
- **MoveIt 2**: ROS 2 framework for manipulation planning.
  [https://moveit.ros.org/](https://moveit.ros.org/)
- **OMPL (Open Motion Planning Library)**: State-of-the-art path planning algorithms.
  [https://ompl.kavrakilab.org/](https://ompl.kavrakilab.org/)

### Machine Learning
- **Stable-Baselines3**: RL algorithms (PPO, SAC, TD3) in PyTorch.
  [https://stable-baselines3.readthedocs.io/](https://stable-baselines3.readthedocs.io/)
- **OpenCV**: Computer vision library for Python/C++.
  [https://opencv.org/](https://opencv.org/)
- **PyBullet**: Lightweight physics simulator for RL.
  [https://pybullet.org/](https://pybullet.org/)

---

## YouTube Channels

1. **Boston Dynamics**
   [https://www.youtube.com/@BostonDynamics](https://www.youtube.com/@BostonDynamics)
   Official channel showcasing Atlas, Spot, and humanoid research.

2. **Articulated Robotics**
   [https://www.youtube.com/@ArticulatedRobotics](https://www.youtube.com/@ArticulatedRobotics)
   ROS 2 tutorials, URDF modeling, robot bringup.

3. **The Construct**
   [https://www.youtube.com/@TheConstruct](https://www.youtube.com/@TheConstruct)
   ROS 2 courses, simulation, and project walkthroughs.

4. **Two Minute Papers**
   [https://www.youtube.com/@TwoMinutePapers](https://www.youtube.com/@TwoMinutePapers)
   Quick summaries of recent AI/robotics research papers.

5. **NVIDIA Developer**
   [https://www.youtube.com/@NVIDIADeveloper](https://www.youtube.com/@NVIDIADeveloper)
   Isaac Sim tutorials, Omniverse updates, and GPU-accelerated robotics.

---

## Conferences & Journals

### Top Robotics Conferences
- **ICRA (International Conference on Robotics and Automation)**: Annual, IEEE.
- **IROS (International Conference on Intelligent Robots and Systems)**: Annual, IEEE.
- **RSS (Robotics: Science and Systems)**: Annual, academic.
- **CoRL (Conference on Robot Learning)**: Annual, focus on ML for robotics.

### Top Journals
- **IEEE Transactions on Robotics (T-RO)**: Premier robotics journal.
- **International Journal of Robotics Research (IJRR)**: High-impact theoretical work.
- **Science Robotics**: Multidisciplinary, high visibility.
- **Autonomous Robots**: Springer journal on autonomous systems.

### Where to Find Papers
- **arXiv.org (cs.RO)**: Preprints before peer review.
  [https://arxiv.org/list/cs.RO/recent](https://arxiv.org/list/cs.RO/recent)
- **Google Scholar**: Search citations and track research trends.
- **Papers With Code**: ML papers with open-source implementations.
  [https://paperswithcode.com/](https://paperswithcode.com/)

---

## Companion Code Repository

All tutorial code, examples, and verification scripts:

**GitHub**: [https://github.com/nizam/physical-ai-code](https://github.com/nizam/physical-ai-code)

Contents:
- `tutorials/`: Hands-on exercises for each module
- `examples/`: Standalone demos (URDF models, launch files, Python nodes)
- `scripts/`: Environment verification and testing utilities
- `.github/workflows/`: CI/CD for automated testing

---

## How to Stay Updated

### Follow These Organizations
- **Open Robotics (ROS)**: [https://www.openrobotics.org/](https://www.openrobotics.org/)
- **NVIDIA Robotics**: [https://developer.nvidia.com/robotics](https://developer.nvidia.com/robotics)
- **Google DeepMind Robotics**: [https://www.deepmind.com/research/highlighted-research/robotics](https://www.deepmind.com/research/highlighted-research/robotics)

### Newsletters
- **The Robot Report**: Weekly robotics industry news.
- **Import AI**: Jack Clark's AI research roundup (includes robotics).
- **Arxiv Sanity**: Custom alerts for new arXiv papers matching keywords.

### Podcasts
- **Robot Talk Podcast**: Interviews with robotics researchers.
- **The Robot Brains Podcast**: Conversations on AI and embodied intelligence.

---

**Previous**: [← Appendix D: Glossary](./glossary.md)
**Home**: [Return to Introduction](../intro.md)
