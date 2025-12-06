# Module 2: Digital Twin (Simulation)

## Overview

Welcome to Module 2! Building on your ROS 2 foundation, you'll now learn to create **digital twins**—virtual replicas of physical robots that enable safe, cost-effective development and testing. This module covers physics simulation (Gazebo), high-fidelity rendering (Unity), and sensor simulation for humanoid robotics.

## Learning Objectives

By the end of this module, you will be able to:

- Understand the role of simulation in the robot development cycle
- Set up and configure Gazebo physics simulation for humanoid robots
- Integrate Unity for photorealistic rendering and human-robot interaction visualization
- Simulate sensors (cameras, LiDAR, IMU) and validate their outputs
- Build a complete simulated humanoid robot with locomotion capabilities

## Why Simulation?

Simulation is critical for robotics development because it enables:

**Risk-Free Testing**: Test dangerous scenarios (falls, collisions) without hardware damage
**Rapid Iteration**: Deploy code changes in seconds, not hours
**Cost Savings**: Develop and validate before expensive hardware builds
**Reproducibility**: Create identical test conditions for algorithm comparison
**Parallel Development**: Multiple developers can work on the same robot simultaneously

**Industry Standard Tools**:
- **Gazebo**: Used by NASA, Boston Dynamics, and academic institutions worldwide
- **Unity**: Industry-leading game engine adapted for robotics visualization
- Combined approach provides both physics accuracy and visual realism

## Module Structure

This module consists of 4 chapters and 1 hands-on tutorial:

### Chapters

1. **Simulation Basics**: Digital twins, sim-to-real gap, and when to use simulation
2. **Gazebo Physics**: Setting up physics engines, contact dynamics, and stability
3. **Unity Rendering**: Photorealistic visualization, animation, and HRI scenarios
4. **Sensor Simulation**: Camera, depth, LiDAR, IMU models with realistic noise

### Tutorial

- **Tutorial 2: Gazebo Humanoid**: Build a simulated humanoid with balance control

## Prerequisites

Before starting this module, you should have:

- Completed Module 1 (ROS 2 fundamentals)
- Understanding of ROS 2 nodes, topics, and URDF
- Basic 3D geometry concepts (coordinate frames, transformations)
- Familiarity with Linux terminal and Python

## Time Commitment

- **Reading**: 8-10 hours total
- **Tutorial**: 1-2 hours
- **Practice**: 3-5 hours recommended

## Hardware Requirements

For this module, you need:
- Computer with Ubuntu 22.04 LTS
- 8GB RAM minimum, 16GB recommended
- Dedicated GPU recommended (NVIDIA preferred for Unity)
- ~5GB disk space for Gazebo models and Unity assets

## Getting Started

Ready to begin? The first chapter will cover simulation basics, digital twins, and the sim-to-real challenge.

## Additional Resources

- [Gazebo Documentation](https://gazebosim.org/docs)
- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [ROS 2 Gazebo Integration](https://github.com/ros-simulation/gazebo_ros2_control)
- [Companion Code Repository](https://github.com/nizam/physical-ai-code)

---

**Previous**: [← Module 1: ROS 2 Middleware](../module-1-ros2/index.md)
**Note**: Chapter content will be added progressively as the book is developed.
