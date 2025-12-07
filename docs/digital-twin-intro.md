---
id: digital-twin-intro
title: Introduction to Digital Twins
sidebar_label: Digital Twin Simulation
sidebar_position: 3
---

# Introduction to Digital Twins for Robotics

## What is a Digital Twin?

A **digital twin** is a virtual representation of a physical system that:
- Mirrors the physical robot's structure and behavior
- Simulates physics (gravity, friction, collisions)
- Provides a safe environment for testing and development
- Enables rapid prototyping without hardware

For robotics, digital twins allow you to:
- **Test algorithms** before deploying to real robots
- **Train AI models** in simulation
- **Visualize** robot behavior
- **Debug** problems without risking hardware damage

## Why Use Simulation?

### 1. Safety
- No risk of damaging expensive hardware
- Test dangerous scenarios (falling, collisions)
- Develop fail-safe behaviors

### 2. Speed
- Faster than real-time simulation (train AI quickly)
- Parallel testing (run multiple scenarios simultaneously)
- No waiting for hardware setup

### 3. Cost
- Reduce hardware wear and tear
- Test without purchasing all components upfront
- Develop software before hardware is ready

### 4. Reproducibility
- Exact same conditions every test
- Deterministic behavior for debugging
- Easy to share scenarios with team

## Popular Robotics Simulators

### Gazebo

**Gazebo** is the most widely-used open-source robot simulator, especially in the ROS ecosystem.

**Key Features:**
- High-fidelity physics simulation (ODE, Bullet, Simbody engines)
- Sensor simulation (cameras, LiDAR, IMU, force-torque)
- Seamless ROS 2 integration
- Plugin system for custom functionality
- Active community and extensive documentation

**Use Cases:**
- ROS-based robot development
- Outdoor environments
- Multi-robot systems

**Installation:**
```bash
# Install Gazebo Garden (recommended for ROS 2 Humble)
sudo apt-get update
sudo apt-get install ros-humble-gazebo-ros-pkgs
```

### Unity Robotics Hub

**Unity** is a powerful game engine adapted for robotics simulation.

**Key Features:**
- Photorealistic rendering (important for computer vision)
- Asset store with pre-made environments and models
- Unity ML-Agents for reinforcement learning
- ROS-Unity bridge for ROS 2 integration

**Use Cases:**
- Computer vision applications
- Human-robot interaction (realistic environments)
- Consumer/service robots

### NVIDIA Isaac Sim

**Isaac Sim** is NVIDIA's robotics simulator built on Omniverse.

**Key Features:**
- GPU-accelerated physics (PhysX 5)
- RTX ray tracing for photorealistic rendering
- Isaac ROS integration
- Synthetic data generation for AI training
- Optimized for NVIDIA hardware

**Use Cases:**
- AI/ML training (especially vision models)
- Warehouse and logistics robots
- Autonomous vehicles

## Digital Twin Workflow

```
1. Model Creation (URDF/USD)
         ↓
2. Import to Simulator (Gazebo/Unity/Isaac)
         ↓
3. Configure Sensors & Actuators
         ↓
4. Develop Control Software (ROS 2)
         ↓
5. Test in Simulation
         ↓
6. Deploy to Real Hardware
```

## URDF: Universal Robot Description Format

**URDF** is an XML format for describing robot models used in ROS.

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="my_robot">

  <!-- Link: Physical component -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0" ixz="0"
               iyy="1.0" iyz="0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Joint: Connection between links -->
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel"/>
    <origin xyz="0 0 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="wheel">
    <!-- Wheel geometry here -->
  </link>

</robot>
```

### URDF Components

**Links**: Physical parts of the robot
- Visual: What you see (appearance)
- Collision: For physics calculations
- Inertial: Mass and inertia properties

**Joints**: Connections between links
- Fixed: No movement
- Revolute: Rotation with limits
- Continuous: Unlimited rotation
- Prismatic: Linear sliding

## Simulating Sensors

### Camera Simulation

```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="camera1">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>800</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>camera</namespace>
        <remapping>image_raw:=image</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

### LiDAR Simulation

```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar_sensor">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.10</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>lidar</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>
```

## Physics Simulation

### Physics Engines

Gazebo supports multiple physics engines:

- **ODE (Open Dynamics Engine)**: Default, good for most cases
- **Bullet**: Fast, good for manipulation
- **DART**: Accurate, good for complex mechanisms
- **Simbody**: Biomechanics, accurate constraints

### Configuring Physics

```xml
<world name="default">
  <physics type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>
  </physics>

  <gravity>0 0 -9.81</gravity>
</world>
```

## Sim-to-Real Transfer

**Sim-to-real transfer** is the challenge of moving from simulation to real hardware. Common issues:

### 1. Physics Mismatch
- **Problem**: Simulated physics don't perfectly match reality
- **Solution**: Add randomization (domain randomization), tune physics parameters

### 2. Sensor Noise
- **Problem**: Real sensors are noisy; simulated sensors are perfect
- **Solution**: Add Gaussian noise to simulated sensor data

### 3. Latency
- **Problem**: Real systems have delays; simulation is instant
- **Solution**: Add artificial delays in simulation

### 4. Model Inaccuracies
- **Problem**: URDF model doesn't match real robot exactly
- **Solution**: Calibrate using real robot measurements

## Best Practices

1. **Start Simple**: Begin with basic shapes before adding detail
2. **Validate Early**: Test simple motions in sim before complex behaviors
3. **Add Noise**: Make simulation more realistic with sensor noise
4. **Use Realistic Physics**: Set mass, inertia, friction appropriately
5. **Monitor Performance**: Keep simulation running in real-time or faster

## Building Our Humanoid Digital Twin

In the next chapters, we'll:
1. Create a humanoid URDF model (already done!)
2. Spawn it in Gazebo
3. Add sensors (cameras, LiDAR, IMU)
4. Control joints through ROS 2
5. Implement navigation behaviors

Our humanoid model (`robot_control_pkg/urdf/humanoid.urdf`) includes:
- Base and torso
- Two arms with shoulder and elbow joints (7 DOF total)
- Proper mass and inertia properties
- Visual and collision geometries

## Next Steps

Ready to see your digital twin in action?

[Next: ROS 2 Joint Control →](ros2-joint-control.md)

## References

- Koenig, N., & Howard, A. (2004). Design and use paradigms for Gazebo, an open-source multi-robot simulator. *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 2149-2154.
- NVIDIA Isaac Sim Documentation: https://docs.omniverse.nvidia.com/app_isaacsim/
- Unity Robotics Hub: https://github.com/Unity-Technologies/Unity-Robotics-Hub
