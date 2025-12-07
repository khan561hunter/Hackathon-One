---
id: isaac-locomotion-training
title: AI-Robot Brain Training for Locomotion
sidebar_label: Isaac Locomotion Training
sidebar_position: 6
---

# User Story 3: AI-Robot Brain Training for Locomotion

## Story Goal

Train a humanoid robot to walk and avoid obstacles using NVIDIA Isaac Sim, understanding AI perception and navigation systems powered by reinforcement learning.

## Learning Objectives

By the end of this tutorial, you will:
- Set up NVIDIA Isaac Sim for humanoid robot simulation
- Convert URDF models to USD format for Isaac
- Integrate Isaac ROS perception modules
- Design reinforcement learning environments for locomotion
- Train walking policies using RL frameworks
- Deploy trained policies to simulated robots

## Prerequisites

- **Hardware**: NVIDIA GPU with RTX support (RTX 3060 or better)
- **Software**:
  - Ubuntu 22.04 LTS
  - NVIDIA drivers (525+)
  - NVIDIA Omniverse Launcher
  - Isaac Sim 2023.1.1 or later
  - ROS 2 Humble
- **Knowledge**:
  - Completed User Stories 1 & 2
  - Basic understanding of reinforcement learning
  - Python programming experience

## Quick Start

```bash
# 1. Launch Isaac Sim
~/.local/share/ov/pkg/isaac_sim-*/isaac-sim.sh

# 2. In Isaac Sim Python environment
cd ~/isaac_sim_project
python3 train_locomotion_policy.py --num_envs 512 --headless

# 3. Test trained policy
python3 test_policy.py --checkpoint ./runs/latest/model.pth
```

## Part 1: NVIDIA Isaac Sim Setup

### What is Isaac Sim?

**NVIDIA Isaac Sim** is a GPU-accelerated robotics simulator built on NVIDIA Omniverse. It provides:

- **Photorealistic rendering** with RTX ray tracing
- **PhysX 5** for accurate physics simulation
- **Parallel environments** for faster RL training (1000+ simultaneous robots)
- **Isaac ROS integration** for perception pipelines
- **Synthetic data generation** for AI training

### Installation

**Step 1: Install Omniverse Launcher**

```bash
# Download from NVIDIA website
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# Make executable
chmod +x omniverse-launcher-linux.AppImage

# Run installer
./omniverse-launcher-linux.AppImage
```

**Step 2: Install Isaac Sim via Launcher**

1. Open Omniverse Launcher
2. Go to **Exchange** tab
3. Search for "Isaac Sim"
4. Click **Install** (choose 2023.1.1 or later)
5. Wait for installation (~15GB download)

**Step 3: Verify Installation**

```bash
# Find Isaac Sim installation
ls ~/.local/share/ov/pkg/isaac_sim-*

# Run Isaac Sim
~/.local/share/ov/pkg/isaac_sim-2023.1.1/isaac-sim.sh
```

You should see the Isaac Sim viewport with a default scene.

### Isaac Sim Python Environment

Isaac Sim includes a custom Python environment with pre-installed packages:

```bash
# Access Isaac Sim Python
~/.local/share/ov/pkg/isaac_sim-*/python.sh

# Install additional packages
~/.local/share/ov/pkg/isaac_sim-*/python.sh -m pip install torch stable-baselines3
```

## Part 2: Converting URDF to USD

### Understanding USD

**Universal Scene Description (USD)** is NVIDIA's format for 3D scenes. Isaac Sim uses USD instead of URDF.

### Conversion Process

**Method 1: Using Isaac Sim UI**

1. Open Isaac Sim
2. Go to **Isaac Utils** ’ **URDF Importer**
3. Select your URDF file: `robot_control_pkg/urdf/humanoid.urdf`
4. Configure import settings:
   - **Merge Fixed Joints**: 
   - **Import Inertia Tensor**: 
   - **Fix Base**:  (we want locomotion)
   - **Self Collision**: 
5. Click **Import**
6. Save as USD: **File** ’ **Save As** ’ `humanoid_isaac.usd`

**Method 2: Programmatic Conversion**

```python
from omni.isaac.urdf import _urdf
import carb

# URDF import configuration
urdf_interface = _urdf.acquire_urdf_interface()

# Import settings
import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = True
import_config.fix_base = False  # Allow movement
import_config.import_inertia_tensor = True
import_config.self_collision = True
import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION

# Perform import
status, stage = urdf_interface.parse_urdf(
    urdf_path="/path/to/humanoid.urdf",
    import_config=import_config
)

if status:
    print("URDF imported successfully!")
    # Save USD
    stage.Export("/path/to/humanoid_isaac.usd")
```

### Adjusting Physics Properties

After conversion, tune physics for RL training:

```python
from pxr import UsdPhysics, PhysxSchema

# Get robot prim
robot_prim = stage.GetPrimAtPath("/World/humanoid")

# Add articulation API
articulation_api = UsdPhysics.ArticulationRootAPI.Apply(robot_prim)

# Enable self-collision
articulation_api.CreateEnabledSelfCollisionsAttr(True)

# Set solver properties
physx_articulation = PhysxSchema.PhysxArticulationAPI.Apply(robot_prim)
physx_articulation.CreateSolverPositionIterationCountAttr(32)
physx_articulation.CreateSolverVelocityIterationCountAttr(16)
```

## Part 3: Isaac ROS Perception Integration

### Isaac ROS Overview

**Isaac ROS** provides GPU-accelerated perception algorithms:

- **Visual SLAM** (Simultaneous Localization and Mapping)
- **Object detection** (DOPE, DNN-based)
- **Depth processing** and point cloud generation
- **Apriltag detection**
- **Image segmentation**

### Setting Up Isaac ROS

**Installation:**

```bash
# Create workspace
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws/src

# Clone Isaac ROS repositories
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_inference.git

# Build
cd ~/isaac_ros_ws
colcon build --symlink-install
source install/setup.bash
```

### Visual SLAM Integration

**Purpose**: Track robot's position using camera data

```python
# isaac_ros_perception.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped

class IsaacROSPerceptionNode(Node):
    def __init__(self):
        super().__init__('isaac_ros_perception_node')

        # Subscribe to Isaac Sim camera
        self.image_sub = self.create_subscription(
            Image,
            '/humanoid/camera/image_raw',
            self.image_callback,
            10
        )

        # Subscribe to Visual SLAM output
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/visual_slam/tracking/vo_pose',
            self.pose_callback,
            10
        )

        self.current_pose = None

    def image_callback(self, msg):
        # Images automatically processed by Visual SLAM node
        pass

    def pose_callback(self, msg):
        self.current_pose = msg.pose
        self.get_logger().info(
            f'Robot position: x={msg.pose.position.x:.2f}, '
            f'y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f}'
        )

def main():
    rclpy.init()
    node = IsaacROSPerceptionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Object Detection with DOPE

**DOPE (Deep Object Pose Estimation)** detects 3D object poses from RGB images:

```bash
# Launch DOPE for object detection
ros2 launch isaac_ros_dope isaac_ros_dope_tensor_rt.launch.py \
  model_file_path:=/models/dope/soup_60k.onnx \
  object_name:=soup
```

**Integrate in Python:**

```python
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray

class ObjectDetectionIntegration(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.detection_callback,
            10
        )

    def detection_callback(self, msg):
        for detection in msg.detections:
            obj_id = detection.id
            pose = detection.pose.pose.pose

            self.get_logger().info(
                f'Detected object {obj_id} at '
                f'x={pose.position.x:.2f}, y={pose.position.y:.2f}'
            )
```

## Part 4: Reinforcement Learning Environment

### RL Environment Design

An RL environment defines:
- **Observations**: What the agent sees (joint angles, velocities, goal position)
- **Actions**: What the agent controls (joint torques/positions)
- **Rewards**: Feedback signal (distance to goal, staying upright, energy efficiency)

### Creating the Locomotion Environment

```python
# locomotion_rl_env.py
import torch
import numpy as np
from omni.isaac.gym.vec_env import VecEnvBase
from omni.isaac.core.utils.torch import torch_rand_float

class HumanoidLocomotionEnv(VecEnvBase):
    def __init__(self, cfg, rl_device, sim_device, graphics_device_id, headless):
        self.cfg = cfg
        self.num_obs = 48  # Joint positions/velocities + orientation
        self.num_actions = 6  # 6 joint torques

        # Reward weights
        self.rew_scales = {
            "lin_vel_xy": 1.0,      # Forward progress
            "ang_vel_z": -0.05,     # Penalize spinning
            "torque": -0.00001,     # Energy efficiency
            "termination": -2.0,    # Avoid falling
            "upright": 0.1,         # Stay upright
        }

        super().__init__(
            cfg, rl_device, sim_device, graphics_device_id, headless
        )

        # Buffers for RL
        self.obs_buf = torch.zeros(
            (self.num_envs, self.num_obs), device=self.device
        )
        self.rew_buf = torch.zeros(self.num_envs, device=self.device)
        self.reset_buf = torch.ones(self.num_envs, device=self.device, dtype=torch.long)

    def create_sim(self):
        """Create Isaac Sim scene with multiple robot instances"""
        self.sim = SimulationContext(
            stage_units_in_meters=1.0,
            physics_dt=self.dt,
            rendering_dt=self.dt,
            backend="torch",
            device=self.device,
        )

        # Create ground plane
        self._create_ground_plane()

        # Create robot instances
        self._create_humanoid_instances()

        return self.sim

    def _create_humanoid_instances(self):
        """Instantiate multiple robots for parallel training"""
        from omni.isaac.core.utils.stage import add_reference_to_stage

        num_per_row = int(np.sqrt(self.num_envs))

        for i in range(self.num_envs):
            row = i // num_per_row
            col = i % num_per_row

            # Position offset
            x = col * 2.0
            y = row * 2.0

            # Add robot instance
            prim_path = f"/World/envs/env_{i}/humanoid"
            add_reference_to_stage(
                usd_path="./humanoid_isaac.usd",
                prim_path=prim_path
            )

            # Set position
            from omni.isaac.core.utils.xforms import set_world_pose
            set_world_pose(prim_path, position=np.array([x, y, 1.0]))

    def get_observations(self):
        """Gather observations for all environments"""
        # Get joint positions and velocities
        dof_pos = self.humanoids.get_joint_positions()
        dof_vel = self.humanoids.get_joint_velocities()

        # Get base orientation
        root_states = self.humanoids.get_world_poses()
        orientations = root_states[1]  # Quaternions

        # Combine into observation
        self.obs_buf = torch.cat([
            dof_pos,
            dof_vel,
            orientations,
            self.commands,  # Target velocity
        ], dim=-1)

        return self.obs_buf

    def pre_physics_step(self, actions):
        """Apply actions before physics step"""
        # Scale actions to torque range
        torques = actions * self.max_torque

        # Apply torques to joints
        self.humanoids.set_joint_efforts(torques)

    def post_physics_step(self):
        """Calculate rewards and check for resets"""
        self.progress_buf += 1

        # Update observations
        self.get_observations()

        # Calculate rewards
        self.calculate_metrics()

        # Check for episode termination
        self.check_termination()

    def calculate_metrics(self):
        """Compute reward signal"""
        # Get robot velocities
        base_lin_vel = self.humanoids.get_linear_velocities()
        base_ang_vel = self.humanoids.get_angular_velocities()

        # Reward forward motion
        lin_vel_reward = torch.sum(
            base_lin_vel[:, :2] * self.commands[:, :2], dim=1
        )

        # Penalize spinning
        ang_vel_penalty = torch.sum(torch.abs(base_ang_vel[:, 2]))

        # Penalize torque usage
        torque_penalty = torch.sum(torch.abs(self.torques), dim=1)

        # Reward staying upright
        orientations = self.humanoids.get_world_poses()[1]
        up_vector = quat_rotate_inverse(orientations, torch.tensor([0, 0, 1]))
        upright_reward = up_vector[:, 2]  # Z component

        # Total reward
        self.rew_buf = (
            self.rew_scales["lin_vel_xy"] * lin_vel_reward
            + self.rew_scales["ang_vel_z"] * ang_vel_penalty
            + self.rew_scales["torque"] * torque_penalty
            + self.rew_scales["upright"] * upright_reward
        )

    def check_termination(self):
        """Determine if episode should end"""
        # Get robot heights
        root_positions = self.humanoids.get_world_poses()[0]
        heights = root_positions[:, 2]

        # Terminate if robot falls (height < 0.3m)
        self.reset_buf = torch.where(
            heights < 0.3,
            torch.ones_like(self.reset_buf),
            self.reset_buf
        )

        # Terminate if episode exceeds max length
        self.reset_buf = torch.where(
            self.progress_buf >= self.max_episode_length,
            torch.ones_like(self.reset_buf),
            self.reset_buf
        )

        # Apply termination penalty
        self.rew_buf = torch.where(
            self.reset_buf > 0,
            self.rew_buf + self.rew_scales["termination"],
            self.rew_buf
        )

    def reset_idx(self, env_ids):
        """Reset specific environments"""
        # Reset joint positions to default
        default_dof_pos = torch.zeros(len(env_ids), self.num_dof, device=self.device)
        self.humanoids.set_joint_positions(default_dof_pos, indices=env_ids)

        # Reset joint velocities
        default_dof_vel = torch.zeros(len(env_ids), self.num_dof, device=self.device)
        self.humanoids.set_joint_velocities(default_dof_vel, indices=env_ids)

        # Reset progress counter
        self.progress_buf[env_ids] = 0
        self.reset_buf[env_ids] = 0
```

## Part 5: Training with RSL-RL

### RSL-RL Overview

**RSL-RL** (Robotic Systems Lab - Reinforcement Learning) is a lightweight RL framework optimized for legged robots.

**Installation:**

```bash
pip install rsl_rl
```

### Training Script

```python
# train_locomotion_policy.py
import torch
from rsl_rl.runners import OnPolicyRunner
from locomotion_rl_env import HumanoidLocomotionEnv

def train():
    # Environment configuration
    env_cfg = {
        "num_envs": 512,
        "episode_length": 500,
        "control_freq_inv": 2,  # 50 Hz control
    }

    # Create environment
    env = HumanoidLocomotionEnv(
        cfg=env_cfg,
        rl_device="cuda:0",
        sim_device="cuda:0",
        graphics_device_id=0,
        headless=True  # No rendering during training
    )

    # Training configuration
    train_cfg = {
        "algorithm": {
            "clip_param": 0.2,
            "desired_kl": 0.01,
            "entropy_coef": 0.01,
            "gamma": 0.99,
            "lam": 0.95,
            "learning_rate": 1e-3,
            "max_grad_norm": 1.0,
            "num_learning_epochs": 5,
            "num_mini_batches": 4,
            "value_loss_coef": 1.0,
        },
        "runner": {
            "max_iterations": 3000,
            "save_interval": 100,
            "log_interval": 10,
        }
    }

    # Create trainer
    runner = OnPolicyRunner(env, train_cfg, log_dir="./runs")

    # Train!
    runner.learn(
        num_learning_iterations=train_cfg["runner"]["max_iterations"],
        init_at_random_ep_len=True
    )

    print("Training complete!")
    print(f"Model saved to: {runner.log_dir}")

if __name__ == "__main__":
    train()
```

### Training Process

**Start training:**

```bash
python3 train_locomotion_policy.py \
  --num_envs 512 \
  --headless \
  --max_iterations 3000
```

**Monitor training:**

```bash
# TensorBoard
tensorboard --logdir=./runs

# Open browser: http://localhost:6006
```

**Training metrics to watch:**
- **Mean Reward**: Should increase over time
- **Episode Length**: Should increase (robot survives longer)
- **Policy Loss**: Should decrease
- **Value Loss**: Should stabilize

**Expected training time:**
- **RTX 3060**: ~2-3 hours for 3000 iterations
- **RTX 4090**: ~45-60 minutes for 3000 iterations

## Part 6: Testing Trained Policy

### Loading and Testing

```python
# test_policy.py
import torch
from locomotion_rl_env import HumanoidLocomotionEnv
from rsl_rl.modules import ActorCritic

def test_policy(checkpoint_path):
    # Create environment (single instance for visualization)
    env_cfg = {"num_envs": 1}
    env = HumanoidLocomotionEnv(
        cfg=env_cfg,
        rl_device="cuda:0",
        sim_device="cuda:0",
        graphics_device_id=0,
        headless=False  # Enable rendering
    )

    # Load trained policy
    policy = ActorCritic(
        num_obs=env.num_obs,
        num_privileged_obs=0,
        num_actions=env.num_actions
    )

    checkpoint = torch.load(checkpoint_path)
    policy.load_state_dict(checkpoint['model_state_dict'])
    policy.eval()

    # Test loop
    obs = env.reset()

    for _ in range(1000):  # Run for 1000 steps
        with torch.no_grad():
            actions = policy.act_inference(obs)

        obs, rewards, dones, info = env.step(actions)

        if dones.any():
            obs = env.reset()

    print("Testing complete!")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=str, required=True)
    args = parser.parse_args()

    test_policy(args.checkpoint)
```

**Run test:**

```bash
python3 test_policy.py --checkpoint ./runs/latest/model.pth
```

### Deploying to ROS 2

Bridge trained policy to ROS 2 for real robot deployment:

```python
# policy_ros_bridge.py
import rclpy
from rclpy.node import Node
import torch
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class PolicyROSBridge(Node):
    def __init__(self, policy_path):
        super().__init__('policy_ros_bridge')

        # Load policy
        self.policy = self.load_policy(policy_path)

        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        # Publish joint commands
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_group_controller/commands',
            10
        )

    def load_policy(self, path):
        policy = ActorCritic(num_obs=48, num_actions=6)
        checkpoint = torch.load(path)
        policy.load_state_dict(checkpoint['model_state_dict'])
        policy.eval()
        return policy

    def joint_callback(self, msg):
        # Convert ROS message to observation tensor
        obs = self.create_observation(msg)

        # Infer action from policy
        with torch.no_grad():
            actions = self.policy.act_inference(obs)

        # Publish joint commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = actions.cpu().numpy().tolist()
        self.cmd_pub.publish(cmd_msg)

def main():
    rclpy.init()
    node = PolicyROSBridge('./runs/latest/model.pth')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Part 7: Advanced Topics

### Domain Randomization

Improve sim-to-real transfer by randomizing simulation parameters:

```python
def randomize_physics(self):
    # Randomize ground friction
    friction = torch_rand_float(0.5, 1.5, (self.num_envs, 1), device=self.device)
    self.gym.set_actor_ground_friction(friction)

    # Randomize robot mass
    mass_scale = torch_rand_float(0.8, 1.2, (self.num_envs, 1), device=self.device)
    self.gym.set_actor_mass_scale(mass_scale)

    # Randomize joint properties
    stiffness = torch_rand_float(0.8, 1.2, (self.num_envs, self.num_dof), device=self.device)
    damping = torch_rand_float(0.5, 1.5, (self.num_envs, self.num_dof), device=self.device)
    self.gym.set_dof_properties(stiffness, damping)
```

### Curriculum Learning

Gradually increase task difficulty:

```python
class CurriculumManager:
    def __init__(self):
        self.phase = 0
        self.phases = [
            {"terrain": "flat", "speed": 0.5},
            {"terrain": "flat", "speed": 1.0},
            {"terrain": "rough", "speed": 0.5},
            {"terrain": "rough", "speed": 1.0},
            {"terrain": "stairs", "speed": 0.5},
        ]

    def update_phase(self, success_rate):
        if success_rate > 0.9 and self.phase < len(self.phases) - 1:
            self.phase += 1
            print(f"Advanced to phase {self.phase}: {self.phases[self.phase]}")

    def get_current_config(self):
        return self.phases[self.phase]
```

## Part 8: Troubleshooting

### Isaac Sim Not Starting

**Error**: "Failed to initialize Omniverse"

**Solution:**
```bash
# Check NVIDIA drivers
nvidia-smi

# Reinstall Vulkan drivers
sudo apt install vulkan-utils

# Verify Vulkan
vulkaninfo
```

### Out of Memory During Training

**Error**: "CUDA out of memory"

**Solutions:**
1. Reduce `num_envs`: Try 256 instead of 512
2. Reduce observation/action space size
3. Use gradient accumulation
4. Enable mixed precision training:

```python
from torch.cuda.amp import autocast, GradScaler

scaler = GradScaler()

with autocast():
    loss = compute_loss(...)

scaler.scale(loss).backward()
scaler.step(optimizer)
scaler.update()
```

### Policy Not Learning

**Symptom**: Reward stays constant or decreases

**Checks:**
1. **Reward shaping**: Ensure rewards guide toward desired behavior
2. **Action scaling**: Actions should be in reasonable range
3. **Observation normalization**: Normalize observations to [-1, 1]
4. **Hyperparameters**: Try lower learning rate (1e-4)

## Key Takeaways

1. **Isaac Sim**: GPU-accelerated simulation enables massively parallel training
2. **RL Environment Design**: Careful reward shaping is critical for success
3. **Observation Space**: Include minimal but sufficient information for task
4. **Training Time**: Parallel environments dramatically reduce training time
5. **Sim-to-Real**: Domain randomization and robust policies bridge the gap

## Next Steps

With a trained locomotion policy, you're ready for:
- **Complex terrains**: Stairs, slopes, uneven surfaces
- **Object manipulation**: Pick and place with trained policies
- **Vision-Language-Action integration**: Voice-controlled behaviors
- **Real robot deployment**: Transfer policies to physical hardware

[Next: Capstone: Voice-Controlled Object Manipulation ’](capstone-vla-manipulation.md)

## References

- Makoviychuk, V., et al. (2021). Isaac Gym: High Performance GPU-Based Physics Simulation for Robot Learning. *NeurIPS*.
- Schulman, J., et al. (2017). Proximal Policy Optimization Algorithms. *arXiv:1707.06347*.
- NVIDIA Isaac Sim Documentation: https://docs.omniverse.nvidia.com/app_isaacsim/
- RSL-RL: https://github.com/leggedrobotics/rsl_rl

---

**Code Files Reference:**
- RL Environment: `isaac_sim_project/locomotion_rl_env.py`
- Training Script: `isaac_sim_project/train_locomotion_policy.py`
- Testing Script: `isaac_sim_project/test_policy.py`
- ROS Bridge: `isaac_sim_project/policy_ros_bridge.py`
