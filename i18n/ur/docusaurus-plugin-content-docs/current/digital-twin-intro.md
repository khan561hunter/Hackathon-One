---
id: digital-twin-intro
title: ڈیجیٹل ٹوئن کا تعارف
sidebar_label: ڈیجیٹل ٹوئن سمیولیشن
sidebar_position: 3
---

# روبوٹکس کے لیے ڈیجیٹل ٹوئن کا تعارف

## ڈیجیٹل ٹوئن کیا ہے؟

**ڈیجیٹل ٹوئن** ایک جسمانی نظام کی virtual نمائندگی ہے جو:
- جسمانی robot کی ساخت اور رویے کی عکاسی کرتا ہے
- Physics کو simulate کرتا ہے (کشش ثقل، friction، ٹکراؤ)
- Testing اور development کے لیے محفوظ ماحول فراہم کرتا ہے
- Hardware کے بغیر تیز prototyping ممکن بناتا ہے

Robotics کے لیے، digital twins آپ کو اجازت دیتے ہیں:
- حقیقی robots پر deploy کرنے سے پہلے **algorithms test کریں**
- Simulation میں **AI models تربیت دیں**
- Robot کے رویے کو **visualize کریں**
- Hardware کو نقصان کے خطرے کے بغیر **debug کریں**

## Simulation کیوں استعمال کریں؟

### 1. حفاظت
- مہنگے hardware کو نقصان کا کوئی خطرہ نہیں
- خطرناک scenarios test کریں (گرنا، ٹکراؤ)
- Fail-safe رویے develop کریں

### 2. رفتار
- Real-time simulation سے تیز (AI کو جلدی تربیت دیں)
- Parallel testing (ایک ساتھ متعدد scenarios چلائیں)
- Hardware setup کے لیے انتظار نہیں

### 3. لاگت
- Hardware wear اور tear کم کریں
- تمام components پہلے سے خریدے بغیر test کریں
- Hardware تیار ہونے سے پہلے software develop کریں

### 4. Reproducibility
- ہر test میں بالکل ایک جیسے conditions
- Debugging کے لیے deterministic رویہ
- Team کے ساتھ scenarios share کرنا آسان

## مشہور Robotics Simulators

### Gazebo

**Gazebo** سب سے زیادہ استعمال ہونے والا open-source robot simulator ہے، خاص طور پر ROS ecosystem میں۔

**کلیدی خصوصیات:**
- High-fidelity physics simulation (ODE، Bullet، Simbody engines)
- Sensor simulation (cameras، LiDAR، IMU، force-torque)
- Seamless ROS 2 integration
- Custom functionality کے لیے plugin system
- فعال community اور وسیع documentation

**استعمال کے معاملات:**
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

**Unity** ایک طاقتور game engine ہے جو robotics simulation کے لیے موافق بنایا گیا ہے۔

**کلیدی خصوصیات:**
- Photorealistic rendering (computer vision کے لیے اہم)
- پہلے سے بنے ماحول اور models کے ساتھ asset store
- Reinforcement learning کے لیے Unity ML-Agents
- ROS 2 integration کے لیے ROS-Unity bridge

**استعمال کے معاملات:**
- Computer vision applications
- Human-robot interaction (realistic environments)
- Consumer/service robots

### NVIDIA Isaac Sim

**Isaac Sim** NVIDIA کا robotics simulator ہے جو Omniverse پر بنایا گیا ہے۔

**کلیدی خصوصیات:**
- GPU-accelerated physics (PhysX 5)
- Photorealistic rendering کے لیے RTX ray tracing
- Isaac ROS integration
- AI training کے لیے synthetic data generation
- NVIDIA hardware کے لیے optimized

**استعمال کے معاملات:**
- AI/ML training (خاص طور پر vision models)
- Warehouse اور logistics robots
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

**URDF** ایک XML format ہے جو ROS میں استعمال ہونے والے robot models کی تفصیل دینے کے لیے ہے۔

### بنیادی URDF ڈھانچہ

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

### URDF کے اجزاء

**Links**: Robot کے جسمانی حصے
- Visual: جو آپ دیکھتے ہیں (ظاہری شکل)
- Collision: Physics calculations کے لیے
- Inertial: Mass اور inertia properties

**Joints**: Links کے درمیان connections
- Fixed: کوئی حرکت نہیں
- Revolute: حدود کے ساتھ rotation
- Continuous: لامحدود rotation
- Prismatic: Linear sliding

## Sensors کو Simulate کرنا

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

Gazebo متعدد physics engines کی support کرتا ہے:

- **ODE (Open Dynamics Engine)**: Default، زیادہ تر cases کے لیے اچھا
- **Bullet**: تیز، manipulation کے لیے اچھا
- **DART**: درست، پیچیدہ mechanisms کے لیے اچھا
- **Simbody**: Biomechanics، درست constraints

### Physics کو Configure کرنا

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

**Sim-to-real transfer** simulation سے حقیقی hardware میں منتقلی کا چیلنج ہے۔ عام مسائل:

### 1. Physics Mismatch
- **مسئلہ**: Simulated physics بالکل حقیقت سے match نہیں ہوتی
- **حل**: Randomization شامل کریں (domain randomization)، physics parameters tune کریں

### 2. Sensor Noise
- **مسئلہ**: حقیقی sensors noisy ہوتے ہیں؛ simulated sensors perfect ہوتے ہیں
- **حل**: Simulated sensor data میں Gaussian noise شامل کریں

### 3. Latency
- **مسئلہ**: حقیقی systems میں delays ہوتی ہیں؛ simulation فوری ہے
- **حل**: Simulation میں artificial delays شامل کریں

### 4. Model کی غلطیاں
- **مسئلہ**: URDF model حقیقی robot سے بالکل match نہیں کرتا
- **حل**: حقیقی robot کی measurements استعمال کرتے ہوئے calibrate کریں

## بہترین طریقے

1. **سادگی سے شروع کریں**: Detail شامل کرنے سے پہلے basic shapes سے شروع کریں
2. **جلد Validate کریں**: پیچیدہ behaviors سے پہلے sim میں سادہ motions test کریں
3. **Noise شامل کریں**: Sensor noise کے ساتھ simulation کو زیادہ realistic بنائیں
4. **Realistic Physics استعمال کریں**: Mass، inertia، friction مناسب طریقے سے set کریں
5. **Performance کی نگرانی کریں**: Simulation کو real-time یا تیز چلائیں

## ہمارے Humanoid Digital Twin کی تعمیر

اگلے chapters میں، ہم:
1. ایک humanoid URDF model بنائیں گے (پہلے سے ہو گیا!)
2. اسے Gazebo میں spawn کریں گے
3. Sensors شامل کریں گے (cameras، LiDAR، IMU)
4. ROS 2 کے ذریعے joints control کریں گے
5. Navigation behaviors implement کریں گے

ہمارے humanoid model (`robot_control_pkg/urdf/humanoid.urdf`) میں شامل ہے:
- Base اور torso
- دونوں arms کے ساتھ shoulder اور elbow joints (7 DOF total)
- مناسب mass اور inertia properties
- Visual اور collision geometries

## اگلے قدم

اپنے digital twin کو action میں دیکھنے کے لیے تیار ہیں؟

[اگلا: ROS 2 Joint Control →](ros2-joint-control.md)

## حوالہ جات

- Koenig, N., & Howard, A. (2004). Design and use paradigms for Gazebo, an open-source multi-robot simulator. *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 2149-2154.
- NVIDIA Isaac Sim Documentation: https://docs.omniverse.nvidia.com/app_isaacsim/
- Unity Robotics Hub: https://github.com/Unity-Technologies/Unity-Robotics-Hub
