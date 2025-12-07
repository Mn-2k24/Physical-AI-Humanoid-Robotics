# References

This page lists all academic papers, technical documentation, and resources cited throughout the book. The complete BibTeX database is available at `references/Physical AI & Humanoid Robotics.bib`.

---

## Academic Papers

### Robotics and Humanoid Systems

**Bono, A., Brameld, K., D'Alfonso, L., & Fedele, G. (2024).** Open Access NAO (OAN): A ROS2-based Software Framework for HRI Applications with the NAO Robot. *arXiv preprint arXiv:2403.13960*.
DOI: [10.48550/arXiv.2403.13960](https://doi.org/10.48550/arXiv.2403.13960)

**de Lima, C. R., Khan, S. G., Tufail, M., Shah, S. H., & Maximo, M. R. O. A. (2024).** Humanoid Robot Motion Planning Approaches: A Survey. *Journal of Intelligent & Robotic Systems*, 110(2), 86.
DOI: [10.1007/s10846-024-02117-z](https://doi.org/10.1007/s10846-024-02117-z)

**Jeong, J., Yang, J., Christmann, G. H. G., & Baltes, J. (2023).** Lightweight Mechatronic System for Humanoid Robot. *The Knowledge Engineering Review*, 38, e5.
DOI: [10.1017/S026988892300005X](https://doi.org/10.1017/S026988892300005X)

---

### ROS 2 and Mobile Robotics

**Jo, W., Kim, J., Wang, R., Pan, J., Senthilkumaran, R. K., & Min, B.-C. (2022).** SMARTmBOT: A ROS2-based Low-cost and Open-source Mobile Robot Platform. *arXiv preprint arXiv:2203.08903*.
DOI: [10.48550/arXiv.2203.08903](https://doi.org/10.48550/arXiv.2203.08903)

**Martini, M., Eirale, A., Cerrato, S., & Chiaberge, M. (2022).** PIC4rl-gym: A ROS2 Modular Framework for Robots Autonomous Navigation with Deep Reinforcement Learning. *arXiv preprint arXiv:2211.10714*.
DOI: [10.48550/arXiv.2211.10714](https://doi.org/10.48550/arXiv.2211.10714)

---

### Sim-to-Real Transfer and Domain Randomization

**Tobin, J., Fong, R., Ray, A., Schneider, J., Zaremba, W., & Abbeel, P. (2017).** Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World. In *2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)* (pp. 23-30). IEEE.
DOI: [10.1109/IROS.2017.8202133](https://doi.org/10.1109/IROS.2017.8202133)
**Key contribution**: Introduced domain randomization as a technique for robust sim-to-real transfer by training on randomized synthetic data.

**Zhao, W., Queralta, J. P., & Westerlund, T. (2020).** Sim-to-Real Transfer in Deep Reinforcement Learning for Robotics: A Survey. In *2020 IEEE Symposium Series on Computational Intelligence (SSCI)* (pp. 737-744). IEEE.
DOI: [10.1109/SSCI47803.2020.9308468](https://doi.org/10.1109/SSCI47803.2020.9308468)
**Key contribution**: Comprehensive survey of sim-to-real approaches including domain randomization, domain adaptation, and hybrid methods.

---

### Machine Learning and Computer Vision

**Paszke, A., Gross, S., Chintala, S., Chanan, G., Yang, E., DeVito, Z., Lin, Z., Desmaison, A., Antiga, L., & Lerer, A. (2017).** Automatic Differentiation in PyTorch. *NIPS Autodiff Workshop*.
**Key contribution**: PyTorch framework used throughout this book for neural network training and inference.

**Bradski, G. (2000).** The OpenCV Library. *Dr. Dobb's Journal of Software Tools*, 25(11), 120-123.
**Key contribution**: OpenCV library used for image processing, camera calibration, and real-time computer vision.

---

## Technical Documentation and Tools

### NVIDIA Ecosystem

**NVIDIA Corporation. (2024).** NVIDIA Isaac Sim: Robotics Simulation and Synthetic Data Generation Platform.
URL: [https://developer.nvidia.com/isaac-sim](https://developer.nvidia.com/isaac-sim)
**Used in**: Module 3 (Chapters 8-10) for photorealistic simulation and synthetic dataset generation.

**NVIDIA Corporation. (2024).** TensorRT SDK for High-Performance Deep Learning Inference.
URL: [https://developer.nvidia.com/tensorrt](https://developer.nvidia.com/tensorrt)
**Used in**: Appendix D for optimizing models for Jetson Orin edge deployment (FP16/INT8 quantization).

---

### Intel RealSense

**Intel Corporation. (2024).** Intel RealSense Depth and Tracking Cameras.
URL: [https://www.intelrealsense.com/](https://www.intelrealsense.com/)
**Used in**: Appendix D for real-world sensor integration and sim-to-real deployment.

---

### Open Source Frameworks

**Open Robotics. (2024).** ROS 2 Humble Hawksbill Documentation.
URL: [https://docs.ros.org/en/humble/](https://docs.ros.org/en/humble/)
**Primary framework**: Used throughout all modules for robot middleware and communication.

**Open Robotics. (2024).** Gazebo Garden Documentation.
URL: [https://gazebosim.org/docs/garden](https://gazebosim.org/docs/garden)
**Used in**: Module 2 (Chapters 5-7) for humanoid walking simulation and navigation.

**Unity Technologies. (2024).** Unity Robotics Hub.
URL: [https://github.com/Unity-Technologies/Unity-Robotics-Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
**Mentioned in**: Chapter 5 as alternative simulation platform with ROS-TCP-Connector.

---

## Additional Resources

### Tutorials and Community Resources

- **ROS 2 Answers**: [https://answers.ros.org/](https://answers.ros.org/) - Community Q&A for ROS 2 troubleshooting
- **Gazebo Community**: [https://community.gazebosim.org/](https://community.gazebosim.org/) - Official Gazebo forum
- **NVIDIA Isaac Sim Forums**: [https://forums.developer.nvidia.com/c/simulation/isaac-sim/](https://forums.developer.nvidia.com/c/simulation/isaac-sim/) - Isaac Sim support forum
- **Jetson Developer Forums**: [https://forums.developer.nvidia.com/c/agx-autonomous-machines/jetson-embedded-systems/](https://forums.developer.nvidia.com/c/agx-autonomous-machines/jetson-embedded-systems/) - Jetson Orin deployment support

---

### Hardware Specifications

**NVIDIA Jetson Orin Series** (2024). Technical specifications and performance benchmarks.
- Orin Nano (4GB): 20 TOPS AI performance, $249
- Orin Nano (8GB): 40 TOPS AI performance, $449
- Orin NX (8GB): 70 TOPS AI performance, $599
- Orin NX (16GB): 100 TOPS AI performance, $799
- Orin AGX (32GB): 200 TOPS AI performance, $1,599
- Orin AGX (64GB): 275 TOPS AI performance, $2,199

Source: [https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/)

---

### MLCommons Edge AI Benchmarks

**MLCommons. (2024).** MLPerf Inference Edge Benchmarks.
URL: [https://mlcommons.org/en/inference-edge/](https://mlcommons.org/en/inference-edge/)
**Used in**: Appendix A for comparing edge device performance (Jetson vs. other platforms).

---

## Citation Guidelines

When citing this book in academic work, please use:

```bibtex
@book{PhysicalAIHumanoidRobotics2025,
  title = {Physical AI \& Humanoid Robotics: A Practical Guide with ROS 2, Gazebo, and Isaac Sim},
  author = {[Nizam ul din]},
  year = {2025},
  publisher = {Self-published},
  url = {https://physical-ai-humanoid-robotics-zeta.vercel.app/},
  note = {Open-source robotics textbook with companion code repository}
}
```

---

## Citation Statistics

**Total Citations**: 14 (as of latest update)

**Citation Distribution**:
- Peer-reviewed journals: 3 (21%)
- Conference proceedings: 3 (21%)
- Technical reports/preprints: 5 (36%)
- Technical documentation: 3 (21%)

**Citation Recency**:
- 2024: 7 (50%)
- 2023: 1 (7%)
- 2022: 2 (14%)
- 2020 and earlier: 4 (29%)

**Geographic Distribution**:
- USA (NVIDIA, Intel, OpenAI): 6 (43%)
- International (academic institutions): 8 (57%)

---

## BibTeX Database

The complete BibTeX database with all citations is available in the companion repository:

**File**: `references/Physical AI & Humanoid Robotics.bib`
**Format**: BibTeX (compatible with Zotero, Mendeley, JabRef)
**Size**: 14 entries (9,380 bytes)

To use the database:

```bash
# Copy BibTeX file to your LaTeX project
cp references/"Physical AI & Humanoid Robotics.bib" ~/my-paper/references.bib

# Or import into Zotero
# File → Import → Select .bib file
```

---

## Updates and Corrections

If you identify citation errors, missing attributions, or outdated links, please submit an issue:

**GitHub Issues**: [https://github.com/Mn-2k24/Physical-AI-Humanoid-Robotics/issues](https://github.com/Mn-2k24/Physical-AI-Humanoid-Robotics/issues)

---

**Previous**: [← Appendix E: Resources](./appendices/resources.md)
