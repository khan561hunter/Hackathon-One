---
id: hardware-setup
title: ہارڈویئر سیٹ اپ اور لیب کی تشکیلات
sidebar_label: ہارڈویئر سیٹ اپ
sidebar_position: 8
---

# ہارڈویئر سیٹ اپ اور لیب کی تشکیلات

## خلاصہ

یہ guide Physical AI اور humanoid robotics کی development کے لیے hardware configurations کی تفصیل فراہم کرتا ہے، budget-friendly setups سے لے کر professional research labs تک۔

## Configuration کے اختیارات

ہم تین hardware tiers فراہم کرتے ہیں جو مختلف use cases اور budgets کے لیے optimized ہیں:

| Configuration | قیمت کی حد | استعمال کا معاملہ | کارکردگی |
|---------------|------------|----------|-------------|
| **Digital Twin Workstation** | $2,000-$5,000 | Simulation، RL training، development | اعلیٰ |
| **Physical AI Edge Kit** | $1,500-$3,000 | Edge deployment، حقیقی robot control | درمیانہ |
| **Cloud Robotics Lab** | $100-$500/ماہ | Scalable، کوئی upfront cost نہیں | متغیر |

---

## آپشن 1: Digital Twin Workstation

### مقصد

اعلیٰ کارکردگی والا workstation برائے:
- Isaac Sim GPU-accelerated simulation
- Parallel RL training (512+ environments)
- Gazebo/Unity simulation
- Documentation development اور deployment

### تجویز کردہ Specifications

#### Budget Build ($2,000-$2,500)

**CPU**: AMD Ryzen 7 5800X یا Intel i7-12700K
- کم از کم 8 cores / 16 threads
- Base clock: 3.8+ GHz
- مقصد: Physics simulation، ROS 2 nodes

**GPU**: NVIDIA RTX 3060 (12GB VRAM)
- CUDA compute capability: 8.6
- AI inference کے لیے tensor cores
- Isaac Sim کے لیے ray tracing
- **اہم**: Isaac Sim کے لیے کم از کم 12GB VRAM

**RAM**: 32GB DDR4-3200
- مطلق کم از کم 16GB
- متعدد simulations کے لیے 32GB تجویز کردہ
- بڑے پیمانے پر RL training کے لیے 64GB ideal

**Storage**:
- 1TB NVMe SSD (primary)
- 2TB HDD (secondary، datasets/logs)

**Motherboard**: PCIe 4.0 کے ساتھ ATX
- متعدد M.2 slots
- USB 3.2 Gen2 ports

**Power Supply**: 650W 80+ Gold
- GPU upgrades کے لیے headroom

**Case**: اچھے airflow کے ساتھ mid-tower

#### Performance Build ($4,000-$5,000)

**CPU**: AMD Ryzen 9 7950X یا Intel i9-13900K
- 16 cores / 32 threads
- 5.0+ GHz boost
- مقصد: بڑے پیمانے پر parallel simulation

**GPU**: NVIDIA RTX 4080 (16GB) یا RTX 4090 (24GB)
- CUDA compute capability: 8.9
- RTX 3060 سے 2-3x تیز
- پیچیدہ scenes کے لیے زیادہ VRAM

**RAM**: 64GB DDR5-5600
- Dual-channel configuration
- Stability کے لیے ECC اختیاری

**Storage**:
- 2TB NVMe Gen4 SSD (primary)
- 4TB NVMe SSD (secondary)
- 8TB HDD (backup/datasets)

**Cooling**: AIO liquid cooler (280mm+)

**Network**: 10GbE NIC (اختیاری، dataset transfer کے لیے)

### Software Installation

**Operating System**: Ubuntu 22.04 LTS

```bash
# Download Ubuntu 22.04 LTS
wget https://releases.ubuntu.com/22.04/ubuntu-22.04.3-desktop-amd64.iso

# Create bootable USB
sudo dd if=ubuntu-22.04.3-desktop-amd64.iso of=/dev/sdX bs=4M status=progress
```

**NVIDIA Drivers**:
```bash
# Add NVIDIA driver repository
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update

# Install latest driver
sudo apt install nvidia-driver-535

# Verify installation
nvidia-smi
```

**ROS 2 Humble**:
```bash
# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl

# Install ROS 2 Humble
sudo apt install ros-humble-desktop

# Source setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Isaac Sim** (اختیاری، NVIDIA GPU کی ضرورت):
```bash
# Download from NVIDIA Omniverse
# https://developer.nvidia.com/isaac-sim

# Install dependencies
sudo apt install libvulkan1 vulkan-utils
```

## آپشن 2: Physical AI Edge Kit

### مقصد

Embedded deployment برائے:
- حقیقی humanoid robot control
- Edge AI inference
- Mobile robotics
- کم power consumption

### تجویز کردہ Hardware

**Main Board**: NVIDIA Jetson Orin Nano/NX
- 8-core ARM CPU
- NVIDIA Ampere GPU (1024-2048 CUDA cores)
- 8-16GB RAM
- 40-100 TOPS AI performance

**قیمت**: $499-$899

**Additional Components**:
- Camera module: $50-$150
- LiDAR sensor: $100-$1,000
- IMU: $20-$100
- Power supply: $30-$80
- Storage (microSD/NVMe): $50-$200

### Software Setup

```bash
# Flash JetPack OS (Ubuntu 20.04-based)
# Use NVIDIA SDK Manager

# Install ROS 2 Humble (ARM64)
sudo apt update
sudo apt install ros-humble-ros-base

# Install perception packages
sudo apt install ros-humble-vision-msgs
sudo apt install ros-humble-image-transport
```

## آپشن 3: Cloud Robotics Lab

### مقصد

Cloud-based development برائے:
- کوئی upfront hardware cost نہیں
- Scalable compute resources
- کہیں سے بھی access
- Team collaboration

### تجویز کردہ Platforms

**AWS RoboMaker**:
- ROS/ROS 2 simulations
- Managed Gazebo instances
- قیمت: $0.50-$2.00/گھنٹہ

**Google Cloud Platform**:
- GPU instances (NVIDIA T4، V100، A100)
- قیمت: $0.35-$3.00/گھنٹہ

**Microsoft Azure**:
- Azure Kinect integration
- قیمت: $0.40-$2.50/گھنٹہ

### Setup Example (AWS)

```bash
# Launch EC2 instance with GPU
aws ec2 run-instances \
  --image-id ami-ubuntu-22.04-gpu \
  --instance-type g4dn.xlarge \
  --key-name my-key-pair

# SSH into instance
ssh -i my-key-pair.pem ubuntu@<instance-ip>

# Install ROS 2
sudo apt update
sudo apt install ros-humble-desktop
```

## Sensors اور Actuators

### Vision Sensors

**Intel RealSense D435**:
- Depth camera
- قیمت: ~$200
- RGB + Depth streams
- ROS 2 drivers دستیاب

**Stereolabs ZED 2**:
- Stereo camera
- قیمت: ~$450
- Spatial AI
- SDK included

### LiDAR Sensors

**RPLidar A1**:
- 2D LiDAR
- قیمت: ~$100
- 360° scanning
- 12m range

**Velodyne VLP-16**:
- 3D LiDAR
- قیمت: ~$4,000
- 16 channels
- 100m range

### Actuators

**Dynamixel Servos**:
- XL330: $20-$30 (چھوٹے joints)
- MX-28: $200-$250 (درمیانے joints)
- XM540: $300-$400 (بڑے joints)

## Networking Setup

### Lab Network

```bash
# Configure static IP for robot
sudo nano /etc/netplan/01-netcfg.yaml
```

```yaml
network:
  version: 2
  ethernets:
    eth0:
      addresses: [192.168.1.100/24]
      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
```

```bash
# Apply configuration
sudo netplan apply
```

### ROS 2 Multi-Machine Setup

```bash
# On robot (Jetson)
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# On workstation
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# Verify connection
ros2 node list
```

## Safety Considerations

1. **Emergency Stop**: Hardware e-stop button ضروری
2. **Power Management**: Surge protection
3. **Workspace**: کم از کم 2m × 2m clear space
4. **Ventilation**: GPU workstations کے لیے مناسب cooling
5. **Backup Power**: UPS (battery) systems

## Troubleshooting

### GPU نہیں مل رہا

```bash
# Check GPU
nvidia-smi

# Reinstall drivers if needed
sudo apt purge nvidia-*
sudo apt install nvidia-driver-535
sudo reboot
```

### ROS 2 Network Issues

```bash
# Check ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID

# Disable firewall temporarily for testing
sudo ufw disable

# Re-enable after testing
sudo ufw enable
```

## Budget Planning

### Starter Setup (~$2,500)
- Digital Twin Workstation (budget build)
- Single camera
- Basic sensors

### Professional Setup (~$8,000)
- Performance workstation
- Jetson Orin NX
- Multiple sensors
- Physical robot platform

### Research Lab (~$25,000+)
- متعدد workstations
- High-end sensors (Velodyne LiDAR)
- Professional robot platforms
- Dedicated server infrastructure

## اگلے قدم

اپنا hardware setup complete کرنے کے بعد:

[اگلا: Capstone Project →](capstone-vla-manipulation.md)

## حوالہ جات

- NVIDIA Jetson Documentation: https://developer.nvidia.com/embedded/jetson
- ROS 2 Hardware Acceleration: https://ros.org/reps/rep-2008.html
- Isaac Sim System Requirements: https://docs.omniverse.nvidia.com/app_isaacsim/
