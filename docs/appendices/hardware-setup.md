# Appendix A: Hardware Setup Guide

This appendix provides detailed hardware recommendations and setup instructions for following along with the Physical AI & Humanoid Robotics course.

## Minimum System Requirements

To complete all modules and tutorials, your system should meet:

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| **OS** | Ubuntu 22.04 LTS (WSL2 supported) | Ubuntu 22.04 LTS (native) |
| **CPU** | 4-core Intel/AMD x86_64 | 8-core Intel i7/AMD Ryzen 7+ |
| **RAM** | 8GB | 16GB-32GB |
| **GPU** | Integrated graphics (Module 1-2 only) | NVIDIA RTX 3060+ (8GB VRAM) |
| **Disk** | 50GB free space | 100GB+ SSD |

## Module-Specific Requirements

### Module 1: ROS 2 Middleware
- **GPU**: Not required
- **Hardware**: Any modern laptop/desktop
- **Software**: ROS 2 Humble (installed via apt)

### Module 2: Simulation (Gazebo + Unity)
- **GPU**: Dedicated GPU recommended for Unity
- **VRAM**: 4GB+ for Unity rendering
- **Software**: Gazebo Garden, Unity 2022 LTS

### Module 3: NVIDIA Isaac Platform
- **GPU**: **NVIDIA RTX 2060 or higher (REQUIRED)**
- **VRAM**: 8GB minimum, 12GB+ recommended
- **Software**: Isaac Sim 2023.1+, Isaac ROS packages

**Note**: Isaac Sim will NOT work on AMD/Intel GPUs or integrated graphics.

### Module 4: VLA Models
- **GPU**: NVIDIA GPU with 8GB+ VRAM (for local Whisper inference)
- **RAM**: 16GB+ for running LLMs locally (or use cloud APIs)
- **Microphone**: Any USB or built-in microphone

## Detailed Hardware Comparison

### Development Machine Specifications

| Configuration | Budget | Mid-Range | High-End | Workstation |
|--------------|---------|-----------|----------|-------------|
| **Price Range** | $800-$1,200 | $1,500-$2,500 | $2,500-$4,000 | $4,000+ |
| **CPU** | AMD Ryzen 5 5600 | AMD Ryzen 7 5800X3D | Intel i9-13900K | AMD Threadripper PRO |
| **RAM** | 16GB DDR4-3200 | 32GB DDR4-3600 | 64GB DDR5-5600 | 128GB+ DDR5 |
| **GPU** | NVIDIA RTX 3060 (12GB) | NVIDIA RTX 4070 (12GB) | NVIDIA RTX 4080 (16GB) | NVIDIA RTX 4090 (24GB) |
| **Storage** | 512GB NVMe SSD | 1TB NVMe Gen4 SSD | 2TB NVMe Gen4 SSD | 4TB+ NVMe RAID |
| **PSU** | 650W 80+ Bronze | 750W 80+ Gold | 850W 80+ Gold | 1000W+ 80+ Platinum |
| **Modules Supported** | 1-2 (CPU-based) | 1-3 (Limited Isaac) | All 4 modules | All + Multi-Robot Sim |
| **Isaac Sim Performance** | Not recommended | 30-45 FPS @ 1080p | 60+ FPS @ 1440p | 60+ FPS @ 4K |
| **LLM Capability** | API only | 7B models local | 13B models local | 70B+ models local |
| **Best For** | ROS 2 learning | Most students | Professional dev | Research teams |

**Key Considerations:**
- **Budget**: Suitable for Modules 1-2 only; use cloud for Isaac Sim
- **Mid-Range**: Best value for most students; handles all modules adequately
- **High-End**: Recommended for serious development and local LLM experimentation
- **Workstation**: Overkill for course but future-proof for multi-robot simulations

### GPU Comparison for Isaac Sim

| GPU Model | VRAM | Isaac Sim | Whisper | Local LLMs | Price | Best For |
|-----------|------|-----------|---------|------------|-------|----------|
| RTX 3050 | 8GB | Marginal | ✓ | Small only | ~$250 | Not recommended |
| RTX 3060 | 12GB | ✓ Basic | ✓ | 7B models | ~$300 | Entry-level |
| RTX 3060 Ti | 8GB | Limited | ✓ | Small only | ~$350 | Skip this |
| RTX 4060 Ti | 16GB | ✓ Good | ✓ | 7B-13B | ~$500 | Best value |
| RTX 4070 | 12GB | ✓ Great | ✓ | 7B-13B | ~$600 | Recommended |
| RTX 4080 | 16GB | ✓ Excellent | ✓ | 13B-30B | ~$1,200 | High-end |
| RTX 4090 | 24GB | ✓ Max | ✓ | 70B+ | ~$1,800 | Workstation |

**Notes:**
- Minimum 8GB VRAM for Isaac Sim, but 12GB+ strongly recommended
- RTX 4000 series offers better performance/watt than 3000 series
- Used RTX 3090 (24GB) can be excellent value at ~$800-900

## Operating System Setup

### Option 1: Ubuntu 22.04 LTS (Recommended)

**Native Installation**:
1. Download Ubuntu 22.04 LTS from [ubuntu.com/download](https://ubuntu.com/download/desktop)
2. Create bootable USB using [Rufus](https://rufus.ie/) (Windows) or `dd` (Linux/Mac)
3. Install alongside existing OS or as primary OS
4. Allocate at least 50GB for root partition

**Dual-Boot Tips**:
- Disable Secure Boot in BIOS for NVIDIA driver compatibility
- Use separate EFI partitions for Windows/Ubuntu
- Back up important data before partitioning

### Option 2: Windows Subsystem for Linux 2 (WSL2)

**Installation** (Windows 10/11):
```powershell
# Run in PowerShell as Administrator
wsl --install -d Ubuntu-22.04

# Enable systemd (required for ROS 2)
# In Ubuntu WSL terminal:
echo -e "[boot]\nsystemd=true" | sudo tee /etc/wsl.conf
wsl --shutdown  # In PowerShell
wsl  # Restart
```

**Limitations**:
- Isaac Sim GUI may have rendering issues (use cloud instance)
- USB device passthrough requires manual configuration
- Performance 10-20% slower than native

### Option 3: Cloud Instances (For Isaac Sim)

If you lack an NVIDIA GPU:
- **AWS EC2**: g4dn.xlarge ($0.526/hr, T4 GPU, 16GB VRAM)
- **Azure**: NC6s_v3 ($1.01/hr, V100 GPU, 16GB VRAM)
- **Lambda Labs**: RTX 3090 instances ($0.50/hr, 24GB VRAM)

## NVIDIA GPU Setup

### Driver Installation (Ubuntu)

```bash
# Check GPU model
lspci | grep -i nvidia

# Install drivers (recommended: latest production branch)
sudo apt update
sudo apt install nvidia-driver-535  # Or latest available

# Reboot required
sudo reboot

# Verify installation
nvidia-smi  # Should show GPU info and CUDA version
```

### CUDA Toolkit (Required for Isaac Sim)

```bash
# Install CUDA 12.1 (check Isaac Sim compatibility first)
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin
sudo mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/12.1.0/local_installers/cuda-repo-ubuntu2204-12-1-local_12.1.0-530.30.02-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu2204-12-1-local_12.1.0-530.30.02-1_amd64.deb
sudo cp /var/cuda-repo-ubuntu2204-12-1-local/cuda-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update
sudo apt-get -y install cuda

# Add to PATH (add to ~/.bashrc)
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
```

## Peripheral Hardware

### Microphone (Module 4)
- **Built-in laptop mic**: Acceptable for testing
- **USB microphone**: Recommended (Blue Yeti, Rode NT-USB)
- **Headset**: Good option for noisy environments

**Test microphone in Ubuntu**:
```bash
# Install audio tools
sudo apt install alsa-utils pulseaudio

# List devices
arecord -l

# Test recording (Ctrl+C to stop)
arecord -d 5 -f cd test.wav
aplay test.wav
```

### Optional: Physical Robot Hardware

If you plan to deploy to real hardware (beyond course scope):
- **Humanoid platforms**: Unitree H1, Agility Robotics Digit (research only)
- **Development boards**: NVIDIA Jetson Orin series, Raspberry Pi 5
- **Sensors**: Intel RealSense D435 (depth camera), RPLidar A1 (2D LiDAR)

**Note**: This course focuses on simulation; physical hardware is NOT required.

### NVIDIA Jetson Orin Comparison

For edge AI deployment and real robot integration:

| Model | Orin Nano (4GB) | Orin Nano (8GB) | Orin NX (8GB) | Orin NX (16GB) | Orin AGX (32GB) | Orin AGX (64GB) |
|-------|----------------|----------------|---------------|----------------|-----------------|-----------------|
| **Price** | $249 | $449 | $599 | $799 | $1,599 | $2,199 |
| **GPU** | 512 CUDA cores | 1024 CUDA cores | 1024 CUDA cores | 1024 CUDA cores | 2048 CUDA cores | 2048 CUDA cores |
| **AI Performance** | 20 TOPS | 40 TOPS | 70 TOPS | 100 TOPS | 200 TOPS | 275 TOPS |
| **Memory** | 4GB LPDDR5 | 8GB LPDDR5 | 8GB LPDDR5 | 16GB LPDDR5 | 32GB LPDDR5 | 64GB LPDDR5 |
| **Storage** | microSD | microSD | microSD + NVMe | microSD + NVMe | microSD + NVMe | microSD + NVMe |
| **Power** | 7-15W | 7-15W | 10-25W | 10-25W | 15-60W | 15-60W |
| **Form Factor** | Nano DevKit | Nano DevKit | NX Module | NX Module | AGX Module | AGX Module |
| **ROS 2 Support** | ✓ Full | ✓ Full | ✓ Full | ✓ Full | ✓ Full | ✓ Full |
| **Isaac ROS** | ✓ Limited | ✓ Yes | ✓ Yes | ✓ Full | ✓ Full | ✓ Full |
| **Whisper (Base)** | Slow (~10s) | ~5s latency | ~3s latency | ~2s latency | Real-time | Real-time |
| **Object Detection** | 10-15 FPS | 20-25 FPS | 30 FPS | 60 FPS | 120+ FPS | 120+ FPS |
| **Best For** | Learning | Hobby projects | Small robots | Mobile robots | Humanoids | Multi-sensor fusion |

**Deployment Recommendations:**
- **Orin Nano 8GB**: Best value for learning edge deployment ($449)
- **Orin NX 16GB**: Recommended for mobile robot prototypes ($799)
- **Orin AGX 32GB**: Production humanoid robots ($1,599)
- **Orin AGX 64GB**: Research platforms with multi-camera fusion ($2,199)

**Key Features:**
- All Jetson Orin models support ROS 2 Humble natively
- Hardware acceleration for Isaac ROS (VSLAM, object detection)
- Low-power operation suitable for battery-powered robots
- Compatible with Intel RealSense, Stereolabs ZED, and RPLIDAR sensors
- Ubuntu 20.04/22.04 support via JetPack 5.x/6.x

## Disk Space Management

Estimated storage by module:
- ROS 2 Humble: ~3GB
- Gazebo models: ~2GB
- Unity installation: ~10GB
- Isaac Sim: ~30GB
- ML models (Whisper, etc.): ~5GB
- Code repositories: ~2GB

**Total**: ~50-60GB recommended

**Cleanup tips**:
```bash
# Remove old Docker images
docker system prune -a

# Clear apt cache
sudo apt clean

# Remove old kernels (keep current + 1 previous)
sudo apt autoremove
```

## Troubleshooting

See [Appendix C: Troubleshooting](./troubleshooting.md) for:
- NVIDIA driver issues (black screen, CUDA version mismatches)
- ROS 2 installation errors
- Isaac Sim GPU compatibility
- Audio device not detected (Module 4)

## References

1. **NVIDIA Jetson Platform**: NVIDIA Corporation, "Jetson Orin Series System-on-Modules Data Sheet," 2023. Technical specifications for edge AI compute platforms including performance benchmarks and power consumption metrics.

2. **Edge AI Hardware Benchmarks**: MLCommons, "MLPerf Inference Benchmark Results v3.0," 2023. Industry-standard performance measurements for AI inference on edge devices including Jetson Orin series.

3. **ROS 2 Hardware Requirements**: Open Robotics, "ROS 2 Humble Hardware Recommendations," ROS 2 Documentation, 2023. Official guidance for system requirements and hardware compatibility for robotics middleware deployment.

---

**Next**: [Appendix B: Software Installation →](./software-installation.md)
