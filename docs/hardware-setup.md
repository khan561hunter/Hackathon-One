---
id: hardware-setup
title: Hardware Setup & Lab Configurations
sidebar_label: Hardware Setup
sidebar_position: 8
---

# Hardware Setup & Lab Configurations

## Overview

This guide details the hardware configurations for Physical AI and humanoid robotics development, from budget-friendly setups to professional research labs.

## Configuration Options

We provide three hardware tiers optimized for different use cases and budgets:

| Configuration | Cost Range | Use Case | Performance |
|---------------|------------|----------|-------------|
| **Digital Twin Workstation** | $2,000-$5,000 | Simulation, RL training, development | High |
| **Physical AI Edge Kit** | $1,500-$3,000 | Edge deployment, real robot control | Medium |
| **Cloud Robotics Lab** | $100-$500/month | Scalable, no upfront cost | Variable |

---

## Option 1: Digital Twin Workstation

### Purpose

High-performance workstation for:
- Isaac Sim GPU-accelerated simulation
- Parallel RL training (512+ environments)
- Gazebo/Unity simulation
- Docusaurus development and deployment

### Recommended Specifications

#### Budget Build ($2,000-$2,500)

**CPU**: AMD Ryzen 7 5800X or Intel i7-12700K
- 8 cores / 16 threads minimum
- Base clock: 3.8+ GHz
- Purpose: Physics simulation, ROS 2 nodes

**GPU**: NVIDIA RTX 3060 (12GB VRAM)
- CUDA compute capability: 8.6
- Tensor cores for AI inference
- Ray tracing for Isaac Sim
- **Critical**: 12GB VRAM minimum for Isaac Sim

**RAM**: 32GB DDR4-3200
- 16GB absolute minimum
- 32GB recommended for multiple simulations
- 64GB ideal for large-scale RL training

**Storage**:
- 1TB NVMe SSD (primary)
- 2TB HDD (secondary, datasets/logs)

**Motherboard**: ATX with PCIe 4.0
- Multiple M.2 slots
- USB 3.2 Gen2 ports

**Power Supply**: 650W 80+ Gold
- Headroom for GPU upgrades

**Case**: Mid-tower with good airflow

#### Performance Build ($4,000-$5,000)

**CPU**: AMD Ryzen 9 7950X or Intel i9-13900K
- 16 cores / 32 threads
- 5.0+ GHz boost
- Purpose: Massive parallel simulation

**GPU**: NVIDIA RTX 4080 (16GB) or RTX 4090 (24GB)
- CUDA compute capability: 8.9
- 2-3x faster than RTX 3060
- Larger VRAM for complex scenes

**RAM**: 64GB DDR5-5600
- Dual-channel configuration
- ECC optional for stability

**Storage**:
- 2TB NVMe Gen4 SSD (primary)
- 4TB NVMe SSD (secondary)
- 8TB HDD (backup/datasets)

**Cooling**: AIO liquid cooler (280mm+)

**Network**: 10GbE NIC (optional, for dataset transfer)

### Software Installation

**Operating System**: Ubuntu 22.04 LTS

```bash
# Download Ubuntu 22.04 LTS
wget https://releases.ubuntu.com/22.04/ubuntu-22.04.3-desktop-amd64.iso

# Create bootable USB (on Linux)
sudo dd if=ubuntu-22.04.3-desktop-amd64.iso of=/dev/sdX bs=4M status=progress

# Boot from USB and install
```

**NVIDIA Drivers**:

```bash
# Add NVIDIA PPA
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update

# Install latest driver (525+)
sudo apt install nvidia-driver-535

# Reboot
sudo reboot

# Verify installation
nvidia-smi
```

**CUDA Toolkit** (for Isaac Sim):

```bash
# Install CUDA 12.x
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin
sudo mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600

wget https://developer.download.nvidia.com/compute/cuda/12.3.0/local_installers/cuda-repo-ubuntu2204-12-3-local_12.3.0-545.23.06-1_amd64.deb

sudo dpkg -i cuda-repo-ubuntu2204-12-3-local_12.3.0-545.23.06-1_amd64.deb
sudo cp /var/cuda-repo-ubuntu2204-12-3-local/cuda-*-keyring.gpg /usr/share/keyrings/

sudo apt update
sudo apt install cuda

# Add to PATH
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc

# Verify
nvcc --version
```

**ROS 2 Humble**:

```bash
# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop

# Install development tools
sudo apt install python3-colcon-common-extensions python3-rosdep

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

**Gazebo**:

```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

**Isaac Sim** (see User Story 3 for detailed installation)

**Docker** (for containerized development):

```bash
# Install Docker
sudo apt install docker.io
sudo systemctl start docker
sudo systemctl enable docker

# Add user to docker group
sudo usermod -aG docker $USER

# Install NVIDIA Container Toolkit
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/libnvidia-container/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt update
sudo apt install nvidia-container-toolkit

# Restart Docker
sudo systemctl restart docker
```

### Performance Tuning

**CPU Governor** (maximize performance):

```bash
# Set CPU to performance mode
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Make persistent
sudo apt install cpufrequtils
echo 'GOVERNOR="performance"' | sudo tee /etc/default/cpufrequtils
```

**GPU Power Limit**:

```bash
# Set maximum power limit (RTX 4090 example)
sudo nvidia-smi -pl 450  # 450W

# Monitor GPU usage
watch -n 1 nvidia-smi
```

**Swap Configuration** (for 32GB RAM systems):

```bash
# Create 32GB swap file
sudo fallocate -l 32G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Make persistent
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

---

## Option 2: Physical AI Edge Kit

### Purpose

Embedded system for:
- Real-time robot control
- On-device AI inference
- Edge deployment
- Sensor integration

### Recommended Hardware

#### NVIDIA Jetson Orin Nano ($499)

**Specifications**:
- GPU: 1024-core NVIDIA Ampere
- CPU: 6-core Arm Cortex-A78AE
- Memory: 8GB LPDDR5
- Storage: microSD (64GB+)
- Power: 7-15W

**Included Kit**:
- Jetson Orin Nano Developer Kit
- Carrier board
- Power supply
- Wi-Fi/Bluetooth module

#### NVIDIA Jetson Orin NX ($899)

**Specifications** (16GB version):
- GPU: 1024-core NVIDIA Ampere
- CPU: 8-core Arm Cortex-A78AE
- Memory: 16GB LPDDR5
- Storage: NVMe SSD support
- Power: 10-25W

### Sensor Suite

**LiDAR**: RPLiDAR A1 ($99)
- Range: 12m
- Sample rate: 8000/sec
- Interface: USB
- ROS 2 driver: `rplidar_ros`

```bash
sudo apt install ros-humble-rplidar-ros
```

**Depth Camera**: Intel RealSense D435i ($299)
- Depth: Stereo 1280x720
- RGB: 1920x1080
- IMU: BMI055
- Interface: USB 3.0
- ROS 2 driver: `realsense2_camera`

```bash
sudo apt install ros-humble-realsense2-camera
```

**IMU**: Adafruit BNO055 ($35)
- 9-DOF (accel, gyro, mag)
- Interface: I2C/UART
- Accuracy: ±1° orientation

**Alternative**: Stereolabs ZED 2 ($449)
- Stereo depth camera
- Neural depth engine
- Built-in IMU
- SDK with ROS 2 support

### Actuators & Controllers

**Servo Motors**: Dynamixel AX-12A ($45 each)
- Torque: 1.5 Nm
- Speed: 59 RPM
- Interface: TTL (half-duplex UART)
- Position feedback
- Quantity: 6-12 for humanoid

**Motor Controller**: U2D2 ($35)
- USB to Dynamixel TTL
- Compatible with all Dynamixel servos

**Alternative**: FEETECH SCS Servos ($25-40 each)
- Budget-friendly
- Similar protocol
- Lower precision

### Robot Platforms

**Option A: TurtleBot 4** ($1,499)
- iRobot Create 3 base
- RPLiDAR A1
- RealSense D435i
- Complete ROS 2 stack

**Option B: Custom Build** ($800-1,200)
- Chassis: Aluminum frame ($100-200)
- Motors: Dynamixel servos ($270-540)
- Sensors: LiDAR + Camera ($398)
- Jetson Orin Nano ($499)

**Option C: Humanoid Kit**: Unitree H1 ($90,000+)
- Professional humanoid platform
- Research/commercial use
- Full SDK and support

### Jetson Setup

**Flash JetPack** (Ubuntu 20.04 + CUDA + cuDNN):

```bash
# On host PC:
# Download NVIDIA SDK Manager
# https://developer.nvidia.com/nvidia-sdk-manager

# Flash Jetson Orin Nano
# Follow SDK Manager wizard

# On Jetson (after first boot):
# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-ros-base

# Install Isaac ROS
sudo apt install ros-humble-isaac-ros-base

# Enable all Jetson Orin cores
sudo nvpmodel -m 0
sudo jetson_clocks
```

**Optimize for Real-Time Control**:

```bash
# Set CPU to performance
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Disable GUI (headless operation)
sudo systemctl set-default multi-user.target

# Enable CAN bus (for motor controllers)
sudo modprobe can
sudo modprobe can_raw
```

### Power System

**Battery**: Tattu 6S LiPo 10000mAh ($120)
- Voltage: 22.2V
- Capacity: 10Ah
- Weight: 1.1kg
- Runtime: 2-4 hours

**Voltage Regulator**:
- 12V rail for Jetson
- 5V rail for sensors
- UBEC or custom regulator

**Battery Monitor**: Adafruit INA219 ($10)
- Current/voltage sensing
- I2C interface
- ROS 2 node for monitoring

### Networking

**Wi-Fi**: Built-in or USB adapter
- 802.11ac minimum
- 5GHz band for video streaming

**Ethernet**: Direct connection (optional)
- 1Gbps for development/debugging

---

## Option 3: Cloud Robotics Lab (Ether Lab Concept)

### Purpose

Cloud-based development without local hardware:
- Remote simulation
- Scalable compute
- Collaborative development
- GPU acceleration on-demand

### Platform Options

#### AWS RoboMaker

**Specifications**:
- EC2 G4dn instances (NVIDIA T4 GPUs)
- Pre-configured ROS 2 environments
- Gazebo simulation
- S3 storage for datasets

**Pricing** (on-demand):
- g4dn.xlarge: $0.526/hour (4 vCPU, 16GB RAM, T4 GPU)
- g4dn.2xlarge: $0.752/hour (8 vCPU, 32GB RAM, T4 GPU)

**Monthly estimate** (100 hours): $75-$200

```bash
# Launch ROS 2 development environment
aws robomaker create-simulation-job \
  --simulation-application arn:aws:robomaker:... \
  --robot-application arn:aws:robomaker:... \
  --max-job-duration-in-seconds 3600
```

#### Google Cloud Robotics

**Specifications**:
- Compute Engine with T4/V100 GPUs
- Kubernetes for containerized workflows
- Cloud Storage for datasets

**Pricing**:
- n1-standard-4 with T4: $0.48/hour
- Preemptible pricing: $0.11/hour

**Monthly estimate**: $50-$150

#### Microsoft Azure IoT Hub + VM

**Specifications**:
- NC-series VMs (V100 GPUs)
- Azure IoT Hub for device management
- Blob storage

**Pricing**:
- NC6s_v3: $3.06/hour (V100 GPU)
- IoT Hub Basic: $10/month

**Monthly estimate**: $100-$500

### Development Workflow

**Remote Development with VS Code**:

```bash
# Install VS Code Remote SSH extension
# Connect to cloud instance

# On cloud instance:
# Install ROS 2
sudo apt update && sudo apt install ros-humble-desktop

# Clone your workspace
git clone https://github.com/your-org/robot_workspace.git

# Build and run
cd robot_workspace
colcon build
source install/setup.bash
ros2 launch ...
```

**X11 Forwarding** (for Gazebo GUI):

```bash
# SSH with X11 forwarding
ssh -X user@cloud-instance-ip

# Run Gazebo
gazebo
```

**VNC** (for better performance):

```bash
# On cloud instance
sudo apt install xfce4 xfce4-goodies tightvncserver

# Start VNC server
vncserver :1 -geometry 1920x1080 -depth 24

# On local machine
ssh -L 5901:localhost:5901 user@cloud-instance-ip

# Connect VNC client to localhost:5901
```

### Cost Optimization

**Spot/Preemptible Instances**:
- 60-90% discount
- Risk: Can be terminated
- Best for: Training, batch processing

**Reserved Instances**:
- 30-50% discount
- Commitment: 1-3 years
- Best for: Long-term development

**Auto-shutdown**:

```bash
# Create shutdown script
cat > ~/auto_shutdown.sh << 'EOF'
#!/bin/bash
# Shutdown if idle for 30 minutes

IDLE_TIME=$(who -s | awk '{print $1}' | wc -l)

if [ $IDLE_TIME -eq 0 ]; then
    sudo shutdown -h now
fi
EOF

chmod +x ~/auto_shutdown.sh

# Add to crontab (check every 30 minutes)
echo "*/30 * * * * ~/auto_shutdown.sh" | crontab -
```

---

## Network Configuration

### Lab Network Setup

**Router**: Wi-Fi 6 (802.11ax)
- Dual-band (2.4GHz + 5GHz)
- MU-MIMO support
- Quality of Service (QoS)

**Recommended**: ASUS RT-AX86U ($250)

**Switch**: Gigabit Ethernet (8-16 ports)
- Unmanaged for simplicity
- NETGEAR GS108 ($30)

### ROS 2 Networking

**Configure DDS Discovery**:

```bash
# Set ROS_DOMAIN_ID (0-101)
export ROS_DOMAIN_ID=42

# Limit discovery to local network
export ROS_LOCALHOST_ONLY=1

# Or specify discovery peers
export ROS_DISCOVERY_SERVER=192.168.1.100:11811
```

**Multi-Machine Setup**:

```bash
# On workstation (192.168.1.100)
export ROS_DOMAIN_ID=42
ros2 run demo_nodes_cpp talker

# On robot (192.168.1.101)
export ROS_DOMAIN_ID=42
ros2 run demo_nodes_cpp listener
```

---

## Safety & Workspace Setup

### Workspace Requirements

**Physical Space**:
- Minimum: 3m × 3m (10ft × 10ft)
- Recommended: 5m × 5m (16ft × 16ft)
- Height: 2.5m+ for overhead fixtures

**Floor**:
- Level surface
- Non-slip mat
- Marked safety boundaries

**Lighting**:
- Bright, even illumination
- No strong shadows (affects vision)

### Safety Equipment

**Emergency Stop**:
- Big red button
- Wired connection (not wireless)
- Kill all power to actuators

**Barriers**:
- Physical fencing (optional)
- Warning signs
- Controlled access

**Fire Safety**:
- LiPo battery fire extinguisher (Class D)
- Fire blanket
- Smoke detector

### LiPo Battery Safety

**Charging**:
- Use LiPo-specific charger
- Never exceed 1C charge rate
- Charge in fireproof bag
- Monitor temperature

**Storage**:
- Storage voltage: 3.8V/cell
- Fireproof container
- Cool, dry location

**Disposal**:
- Discharge to 0V (slowly)
- Salt water bath (24 hours)
- Recycle at proper facility

---

## Bill of Materials (BOM)

### Digital Twin Workstation (Budget)

| Component | Model | Price |
|-----------|-------|-------|
| CPU | AMD Ryzen 7 5800X | $300 |
| GPU | NVIDIA RTX 3060 12GB | $400 |
| Motherboard | MSI B550-A PRO | $140 |
| RAM | 32GB DDR4-3200 | $100 |
| SSD | 1TB NVMe Gen3 | $80 |
| PSU | EVGA 650W Gold | $90 |
| Case | Fractal Design Meshify C | $100 |
| Cooling | Cooler Master Hyper 212 | $40 |
| **Total** | | **$1,250** |

### Physical AI Edge Kit (Complete)

| Component | Model | Price |
|-----------|-------|-------|
| Compute | Jetson Orin Nano 8GB | $499 |
| LiDAR | RPLiDAR A1 | $99 |
| Camera | Intel RealSense D435i | $299 |
| Servos | 6× Dynamixel AX-12A | $270 |
| Controller | U2D2 | $35 |
| Chassis | Custom aluminum frame | $150 |
| Battery | 6S 10Ah LiPo | $120 |
| Misc | Cables, mounts, etc. | $100 |
| **Total** | | **$1,572** |

### Cloud Robotics (Monthly)

| Service | Configuration | Price/Month |
|---------|---------------|-------------|
| Compute | AWS g4dn.xlarge (100 hours) | $75 |
| Storage | 1TB S3 | $23 |
| Network | Data transfer | $10 |
| **Total** | | **$108/month** |

---

## Procurement Guide

### Where to Buy

**Computer Components**:
- Newegg.com
- Amazon.com
- Micro Center (US retail)

**Robotics Hardware**:
- RobotShop.com
- TrossenRobotics.com
- Adafruit.com
- SparkFun.com

**NVIDIA Products**:
- NVIDIA Developer Store
- Authorized distributors

**Sensors**:
- IntelRealSense.com (official)
- SLAMTEC.com (RPLiDAR official)

### Educational Discounts

**NVIDIA**:
- Academic pricing on Jetson kits
- 30-40% discount
- Requires .edu email

**AWS**:
- AWS Educate credits
- $100-300 free credits
- Apply at aws.amazon.com/education

---

## Maintenance & Upgrades

### Workstation Maintenance

**Quarterly**:
- Clean dust filters
- Reapply thermal paste (yearly)
- Update NVIDIA drivers
- Ubuntu system updates

### Robot Maintenance

**Monthly**:
- Inspect servo gears
- Tighten loose screws
- Clean sensors (camera lens, LiDAR window)
- Check battery health

**Servo Calibration**:

```bash
# Using Dynamixel Wizard
# 1. Connect U2D2 to PC
# 2. Launch Dynamixel Wizard
# 3. Scan for servos
# 4. Test each servo
# 5. Update firmware if available
```

### Upgrade Path

**Year 1**: Budget workstation + simulation
**Year 2**: Add Jetson Edge Kit + basic robot
**Year 3**: Upgrade to RTX 4080 + advanced sensors
**Year 4**: Humanoid platform or research robot

---

## Troubleshooting

### GPU Not Detected

```bash
# Check if GPU is visible
lspci | grep -i nvidia

# Reinstall drivers
sudo apt purge nvidia-*
sudo apt install nvidia-driver-535
sudo reboot
```

### Jetson Won't Boot

1. Check power supply (5V 4A minimum)
2. Reflash with SDK Manager
3. Verify microSD card (Class 10+)

### ROS 2 Communication Issues

```bash
# Check network
ping <other_machine_ip>

# Verify ROS_DOMAIN_ID matches
echo $ROS_DOMAIN_ID

# List nodes
ros2 node list

# Check topic traffic
ros2 topic hz /topic_name
```

---

## Key Takeaways

1. **Start Simple**: Begin with simulation, add hardware incrementally
2. **GPU is Critical**: RTX 3060+ for Isaac Sim, Jetson for edge deployment
3. **Scalability**: Cloud provides flexibility without upfront cost
4. **Safety First**: Proper workspace setup prevents accidents
5. **Maintenance**: Regular upkeep extends hardware lifespan

## Next Steps

With your hardware configured, you're ready to:
- Deploy trained models to edge devices
- Run full VLA pipeline on real robots
- Scale experiments with cloud resources
- Build custom robotic platforms

---

## Resources

- NVIDIA Developer Program: https://developer.nvidia.com/
- ROS 2 Hardware Guide: https://docs.ros.org/en/humble/
- Robotics Suppliers: https://www.robotshop.com/
- Safety Standards: ISO 10218 (Robot Safety)

---

**Hardware Safety Notice**: Always follow manufacturer guidelines for electrical components, batteries, and robotic systems. Improper handling can cause injury or fire.
