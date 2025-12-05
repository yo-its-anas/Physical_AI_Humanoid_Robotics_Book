---
id: requirements
title: "Software and Hardware Requirements"
sidebar_label: "Requirements Overview"
sidebar_position: 5
description: "Complete hardware and software requirements for the Physical AI & Humanoid Robotics course"
keywords:
  - requirements
  - hardware specs
  - software dependencies
tags:
  - preface
  - setup
---

# Software and Hardware Requirements

This page details all software and hardware needed to complete the course. Requirements are organized by learning pathway.

## Learning Pathways

Choose your path based on goals and budget:

### Path 1: Simulation Only (Minimum)

**Goal**: Learn concepts without physical hardware
**Cost**: $1,500-2,500
**Chapters**: All chapters, simulation-based labs only

### Path 2: Simulation + Edge (Recommended)

**Goal**: Full learning experience including edge deployment
**Cost**: $2,500-4,000
**Chapters**: All chapters, deploy to Jetson

### Path 3: Complete Lab (Maximum)

**Goal**: Physical robot testing and sim-to-real transfer
**Cost**: $5,000-15,000
**Chapters**: All chapters including real robot capstone

## Software Requirements

### Operating System

**Required**: Ubuntu 22.04 LTS (Jammy Jellyfish)

**Why Ubuntu 22.04**:
- ROS 2 Humble official support
- Isaac Sim compatibility
- Most robotics tools target Ubuntu

**Alternatives**:
- Ubuntu 20.04 LTS (for ROS 2 Foxy, older)
- Docker containers (for non-Linux users)
- WSL2 on Windows (limited GPU support)

**Installation**:
```bash
# Download from https://ubuntu.com/download/desktop
# Recommended: Clean install on dedicated partition
# Minimum: Dual boot or VM (with GPU passthrough)
```

### ROS 2 Humble

**Version**: ROS 2 Humble Hawksbill (LTS release)

**Installation**:
```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop-full
sudo apt install ros-humble-ros-base
sudo apt install ros-dev-tools

# Environment setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Verification**:
```bash
ros2 --version
# Expected output: ros2 cli version X.X.X
```

### Gazebo Simulator

**Version**: Gazebo Garden or Fortress

**Installation**:
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-gazebo-plugins
```

**Verification**:
```bash
gazebo --version
# Expected: Gazebo multi-robot simulator, version 11.x or higher
```

### NVIDIA Isaac Sim

**Version**: Isaac Sim 2023.1.0 or later

**Requirements**:
- NVIDIA RTX GPU (see hardware section)
- Ubuntu 22.04
- NVIDIA Driver 525.60.11 or newer

**Installation**:
1. Install Omniverse Launcher:
```bash
# Download from https://www.nvidia.com/en-us/omniverse/download/
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage
```

2. Through Launcher:
   - Install Nucleus (local server)
   - Install Isaac Sim 2023.1.0+
   - Install Cache

**Verification**:
```bash
# Launch Isaac Sim
~/.local/share/ov/pkg/isaac_sim-*/isaac-sim.sh
```

### Python Development Environment

**Required Packages**:

```bash
# Python 3.10 (comes with Ubuntu 22.04)
sudo apt install python3-pip python3-venv

# ROS 2 Python dependencies
pip3 install rclpy std_msgs geometry_msgs sensor_msgs

# Machine Learning (for VLA chapters)
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip3 install transformers openai-whisper

# Development tools
pip3 install black flake8 pytest
```

### Additional Tools

**Code Editor/IDE**:
- VS Code (recommended): `snap install code --classic`
- PyCharm Professional
- vim/emacs (for terminal users)

**Version Control**:
```bash
sudo apt install git
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

**Build Tools**:
```bash
sudo apt install build-essential cmake
```

## Hardware Requirements

### Path 1: Simulation Workstation (Minimum)

**Purpose**: Run Isaac Sim, Gazebo, ROS 2 development

**Specifications**:

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| **CPU** | Intel i7-10700 / AMD Ryzen 7 3700X | Intel i9-12900K / AMD Ryzen 9 5900X |
| **GPU** | NVIDIA RTX 3060 (12GB VRAM) | NVIDIA RTX 4070 or higher |
| **RAM** | 32 GB DDR4 | 64 GB DDR4/DDR5 |
| **Storage** | 500 GB NVMe SSD | 1 TB NVMe SSD + 2TB HDD |
| **OS** | Ubuntu 22.04 LTS | Ubuntu 22.04 LTS |

**GPU Critical**:
- Isaac Sim requires RTX series (not GTX)
- Ray tracing cores needed for photorealistic rendering
- Minimum 8GB VRAM, 12GB+ recommended

**Cost Estimate**: $1,500-2,500 (desktop) / $2,000-3,500 (laptop)

**Alternatives**:
- Cloud workstation (AWS g5.xlarge or g5.2xlarge)
- Colab Pro+ with GPU (limited Isaac Sim support)

### Path 2: + Jetson Edge Device (Recommended)

**Add to Simulation Workstation**:

**Jetson Orin Nano Developer Kit** ($499):
- 1024-core NVIDIA Ampere GPU
- 8GB RAM
- Ubuntu 20.04 (JetPack 5.1+)
- For edge deployment and real-time inference

**Or Jetson AGX Orin** ($1,999):
- 2048-core GPU
- 32GB or 64GB RAM
- Higher performance for complex VLA models

**Sensors** (for Jetson):
- Intel RealSense D435i ($329): RGB-D camera
- IMU (MPU6050 or similar) ($10-50)
- Microphone (for voice recognition) ($20-100)

**Total Addition**: $850-2,500

### Path 3: + Robot Platform (Complete)

**Add to Simulation + Edge**:

**Option A: Proxy Quadruped** ($1,600-2,700):
- Unitree Go2 ($1,600): Affordable quadruped for testing
- Not humanoid, but tests locomotion and navigation

**Option B: Mini-Humanoid** ($3,000-8,000):
- Robotis OP3 (~$6,000): Research-grade humanoid
- Hiwonder XiaoR Humanoid (~$3,000): Educational platform

**Option C: Full Humanoid** ($16,000-90,000):
- Unitree G1 (~$16,000): Commercial humanoid
- Agility Digit ($250,000+): Industrial-grade (university labs)

**Recommended for Course**: Option A (Unitree Go2) or simulation only

**Total for Complete Lab**: $5,000-15,000 (workstation + Jetson + mini-humanoid)

## Cloud Computing Alternative

For students without local hardware:

### AWS EC2 Instances

**For Isaac Sim**:
- Instance type: g5.xlarge ($1.006/hour on-demand)
- GPU: NVIDIA A10G (24GB VRAM)
- vCPUs: 4
- RAM: 16 GB

**For Basic ROS 2**:
- Instance type: t3.large ($0.0832/hour)
- No GPU needed

**Cost Estimate**:
- 10 hours/week for semester: ~$150-200
- AWS Educate credits may cover this

### Google Colab Pro+

**For Jupyter-based Learning**:
- Cost: $49.99/month
- GPU access (limited hours)
- Not suitable for full Isaac Sim workflow

## Network Requirements

**Bandwidth**:
- Minimum: 10 Mbps download (for packages, updates)
- Recommended: 50+ Mbps (for cloud workflows, video tutorials)

**Storage for Downloads**:
- ROS 2 packages: ~5 GB
- Isaac Sim: ~30 GB
- Code examples and datasets: ~10 GB
- **Total: 50+ GB free space**

## Budget Summary

| Pathway | Initial Cost | Ongoing Costs |
|---------|-------------|---------------|
| **Simulation Only** | $1,500-2,500 | Electricity, internet |
| **Sim + Edge** | $2,500-4,000 | Same as above |
| **Complete Lab** | $5,000-15,000 | Same + hardware maintenance |
| **Cloud-Based** | $0-500 (hardware) | $50-200/month |

## Verification Checklist

Before starting Chapter 1:

**Software**:
- [ ] Ubuntu 22.04 installed
- [ ] ROS 2 Humble installed and sourced
- [ ] Gazebo launches successfully
- [ ] Isaac Sim opens (if using)
- [ ] Python 3.10 with pip available
- [ ] Git configured

**Hardware**:
- [ ] Workstation meets minimum specs
- [ ] GPU drivers installed (NVIDIA)
- [ ] Jetson setup complete (if using)
- [ ] Sensors connected and recognized (if using)

**Network**:
- [ ] Internet connectivity stable
- [ ] Can download ROS packages
- [ ] GitHub access verified

**Verification Commands**:
```bash
# Check ROS 2
ros2 topic list

# Check GPU
nvidia-smi

# Check Python
python3 --version
pip3 --version

# Check Git
git --version
```

## Getting Help

**If you don't meet requirements**:
- **Budget constraints**: Start with cloud-based or simulation-only
- **Hardware limitations**: Use lower-quality settings in Isaac Sim
- **OS issues**: Try Docker containers or WSL2

**See Also**:
- [Chapter 20: Workstation Setup](../06-hardware-lab/20-workstations.md)
- [Chapter 21: Jetson Configuration](../06-hardware-lab/21-jetson.md)
- [Appendix D: Hardware Checklists](../08-appendix/d-hardware-checklists.md)

---

*Requirements reviewed as of December 2025. Check official documentation for latest versions.*
