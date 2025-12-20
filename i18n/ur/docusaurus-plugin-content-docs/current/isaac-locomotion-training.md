---
id: isaac-locomotion-training
title: Locomotion کے لیے AI-Robot Brain Training
sidebar_label: Isaac Locomotion Training
sidebar_position: 6
---

# یوزر اسٹوری 3: Locomotion کے لیے AI-Robot Brain Training

## اسٹوری کا مقصد

NVIDIA Isaac Sim استعمال کرتے ہوئے humanoid robot کو چلنا اور obstacles سے بچنا سکھانے کے لیے reinforcement learning استعمال کریں۔

## سیکھنے کے مقاصد

اس tutorial کے اختتام تک، آپ:
- NVIDIA Isaac Sim environment سیٹ اپ کریں گے
- Humanoid locomotion task define کریں گے
- Reinforcement learning algorithm implement کریں گے (PPO)
- GPU-accelerated training چلائیں گے
- Trained policy کو evaluate کریں گے
- Trained model کو حقیقی robot میں deploy کریں گے

## پیشگی ضروریات

- User Stories 1 & 2 مکمل
- NVIDIA GPU (RTX 3060 یا بہتر، 12GB+ VRAM)
- Ubuntu 22.04 LTS
- Python 3.8+
- PyTorch انسٹال
- Isaac Sim انسٹال (NVIDIA Omniverse سے)

## Isaac Sim کیوں؟

**NVIDIA Isaac Sim** GPU-accelerated robotics simulator ہے جو مشہور ہے:
- **GPU Physics**: CPU-based simulators سے 10-100x تیز
- **Parallel Environments**: training کے لیے ایک ساتھ 1000+ robots
- **Photorealistic Rendering**: vision-based tasks کے لیے
- **Domain Randomization**: Sim-to-real transfer کے لیے
- **Isaac Gym**: RL training کے لیے built-in tools

## فوری شروعات

```bash
# Terminal 1: Isaac Sim launch کریں
~/.local/share/ov/pkg/isaac_sim-2023.1.0/isaac-sim.sh

# Terminal 2: Training script چلائیں
cd ~/isaac_training
python train_humanoid_walk.py --task=HumanoidLocomotion --num_envs=512
```

## Part 1: Environment Setup

### Isaac Sim Installation

```bash
# Omniverse Launcher download کریں
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# Executable بنائیں
chmod +x omniverse-launcher-linux.AppImage

# Launch اور Isaac Sim install کریں
./omniverse-launcher-linux.AppImage
```

### Python Environment

```bash
# Virtual environment بنائیں
python3 -m venv isaac_env
source isaac_env/bin/activate

# Dependencies install کریں
pip install torch torchvision
pip install numpy matplotlib tensorboard
pip install omniisaacgymenvs
```

## Part 2: Humanoid Locomotion Task

### State Space

Robot کی state (observation) میں شامل ہے:
- Joint positions (12 values)
- Joint velocities (12 values)
- Base orientation (4 values - quaternion)
- Base linear velocity (3 values)
- Base angular velocity (3 values)
- **Total**: 34-dimensional state vector

### Action Space

Robot کے actions (controls):
- 12 joint position targets (ہر leg میں 6 joints)
- Action range: [-1, 1] (normalized)
- Mapped to actual joint limits

### Reward Function

```python
def calculate_reward(self):
    reward = 0.0

    # Forward progress (آگے بڑھنے کی حوصلہ افزائی)
    reward += self.base_velocity_x * 2.0

    # Alive bonus (کھڑے رہنے کی حوصلہ افزائی)
    reward += 0.1

    # Energy penalty (efficiency کی حوصلہ افزائی)
    reward -= 0.01 * np.sum(np.square(self.joint_torques))

    # Orientation penalty (سیدھا کھڑا رہنے کی حوصلہ افزائی)
    reward -= 0.5 * abs(self.base_roll) + 0.5 * abs(self.base_pitch)

    # Termination (گرنے پر negative reward)
    if self.is_fallen():
        reward -= 10.0

    return reward
```

## Part 3: PPO Training Algorithm

### Proximal Policy Optimization (PPO)

PPO ایک مشہور RL algorithm ہے locomotion کے لیے:

```python
import torch
import torch.nn as nn
from torch.distributions import Normal

class ActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim):
        super().__init__()

        # Actor network
        self.actor = nn.Sequential(
            nn.Linear(state_dim, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, action_dim)
        )

        # Critic network
        self.critic = nn.Sequential(
            nn.Linear(state_dim, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, 1)
        )

    def forward(self, state):
        action_mean = self.actor(state)
        value = self.critic(state)
        return action_mean, value
```

### Training Loop

```bash
# Training شروع کریں (512 parallel environments)
python train_humanoid_walk.py \
    --task HumanoidLocomotion \
    --num_envs 512 \
    --max_iterations 10000 \
    --learning_rate 3e-4
```

**متوقع training time**:
- RTX 3060: ~6-8 گھنٹے
- RTX 4080: ~3-4 گھنٹے
- RTX 4090: ~2-3 گھنٹے

## Part 4: Domain Randomization

Sim-to-real transfer بہتر بنانے کے لیے physics parameters randomize کریں:

```python
class DomainRandomization:
    def randomize_physics(self):
        # Mass randomization
        mass_scale = np.random.uniform(0.8, 1.2)

        # Friction randomization
        friction = np.random.uniform(0.5, 1.5)

        # Motor strength randomization
        motor_strength = np.random.uniform(0.9, 1.1)

        return mass_scale, friction, motor_strength
```

## Part 5: Evaluation

### Trained Policy Test کریں

```bash
# Isaac Sim میں trained policy چلائیں
python evaluate_policy.py \
    --checkpoint runs/HumanoidLocomotion_512/model_5000.pth \
    --num_envs 16
```

### Metrics

- **Success Rate**: episodes کا % جو گرے بغیر goal تک پہنچے
- **Average Speed**: Forward velocity (m/s)
- **Energy Efficiency**: فی میٹر torque
- **Stability**: Orientation deviation

## Part 6: Deployment

### Trained Model Export کریں

```python
# ONNX format میں export کریں
import torch.onnx

dummy_input = torch.randn(1, 34)
torch.onnx.export(policy_network, dummy_input, "humanoid_walk.onnx")
```

### Real Robot میں Deploy کریں

```bash
# Jetson Orin پر model load کریں
python deploy_to_robot.py \
    --model humanoid_walk.onnx \
    --device cuda \
    --rate 50
```

## Troubleshooting

### Training Unstable ہے

**Symptoms**: Reward تیزی سے گرتا ہے، robot گرتا رہتا ہے

**Solutions**:
- Learning rate کم کریں: `3e-4` → `1e-4`
- Reward shaping بہتر بنائیں
- Domain randomization کم کریں

### GPU Memory کی کمی

**Solutions**:
```bash
# Environments کی تعداد کم کریں
python train.py --num_envs 256  # 512 کی بجائے

# Batch size کم کریں
python train.py --minibatch_size 128
```

## کلیدی سبق

1. **GPU Acceleration**: Parallel simulation training کو بڑے پیمانے پر scalable بناتی ہے
2. **Reward Engineering**: اچھی طرح designed rewards مطلوبہ behaviors سیکھنے کی کلید ہیں
3. **Domain Randomization**: Simulation سے real robot میں transfer بہتر بناتا ہے
4. **Curriculum Learning**: سادہ tasks سے پیچیدہ کی طرف بتدریج بڑھنا
5. **Monitoring**: TensorBoard training progress visualize کرنے کے لیے ضروری

## اگلے قدم

اب جب آپ locomotion policies تربیت دے سکتے ہیں:
- Object manipulation سیکھنے کے لیے grasping شامل کریں
- Vision-based navigation implement کریں
- Multi-task learning explore کریں

[اگلا: Voice-Controlled Object Manipulation (Capstone) →](capstone-vla-manipulation.md)

## حوالہ جات

- Schulman, J., et al. (2017). Proximal Policy Optimization Algorithms. *arXiv:1707.06347*.
- NVIDIA Isaac Sim Documentation: https://docs.omniverse.nvidia.com/app_isaacsim/
- Isaac Gym: https://developer.nvidia.com/isaac-gym
