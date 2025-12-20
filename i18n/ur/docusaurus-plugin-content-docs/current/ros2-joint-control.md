---
id: ros2-joint-control
title: ROS 2 بنیادی جوائنٹ کنٹرول
sidebar_label: ROS 2 جوائنٹ کنٹرول
sidebar_position: 4
---

# یوزر اسٹوری 1: ROS 2 بنیادی جوائنٹ کنٹرول

## اسٹوری کا مقصد

ایک simulated humanoid joint کو کنٹرول کرکے ROS 2 nodes، topics، اور services کی بنیادی باتیں سمجھنا۔

## سیکھنے کے مقاصد

اس ٹیوٹوریل کے اختتام تک، آپ:
- ROS 2 Python nodes بنائیں گے اور چلائیں گے
- Standard message types استعمال کرتے ہوئے joint commands شائع کریں گے
- Robot controllers کو configure اور launch کریں گے
- Gazebo simulation میں robot کے رویے کا مشاہدہ کریں گے
- ROS 2 control framework کو سمجھیں گے

## پیشگی ضروریات

- ROS 2 Humble انسٹال ہے
- Gazebo simulator انسٹال ہے
- `robot_control_pkg` package بنایا گیا ہے
- بنیادی Python programming کا علم

## فن تعمیر کا جائزہ

```
┌──────────────────┐      /joint_group_controller/commands       ┌─────────────────┐
│ Joint Commander  │ ────────────────────────────────────────────> │   Controller    │
│      Node        │         (Float64MultiArray)                  │    Manager      │
└──────────────────┘                                               └─────────────────┘
                                                                            │
                                                                            │ Joint Commands
                                                                            ↓
                                                                   ┌─────────────────┐
                                                                   │ Gazebo Simulator│
                                                                   │  (Humanoid)     │
                                                                   └─────────────────┘
                                                                            │
                                                                            │ Joint States
                                                                            ↓
                                                                   ┌─────────────────┐
                                                                   │ Joint State     │
                                                                   │  Broadcaster    │
                                                                   └─────────────────┘
                                                                            │
                                                                            ↓
                                                              /joint_states (sensor_msgs/JointState)
```

## مرحلہ 1: Humanoid URDF کا جائزہ لیں

ہمارا humanoid robot model `robot_control_pkg/urdf/humanoid.urdf` میں define ہے:

- **Base Link**: Robot کی بنیاد
- **Torso**: مرکزی جسم
- **Left Arm**: Shoulder (yaw + pitch) + Elbow = 3 DOF
- **Right Arm**: Shoulder (yaw + pitch) + Elbow = 3 DOF
- **کل**: 6 actuated joints

### قابل کنٹرول Joints

| Joint کا نام | قسم | حد | تفصیل |
|------------|------|-------|-------------|
| `left_shoulder_joint` | Revolute | -1.57 to 1.57 rad | بائیں کندھے کا yaw (سائیڈ سے سائیڈ) |
| `left_shoulder_pitch_joint` | Revolute | -3.14 to 3.14 rad | بائیں کندھے کا pitch (اوپر نیچے) |
| `left_elbow_joint` | Revolute | 0 to 2.5 rad | بائیں کہنی کا flex |
| `right_shoulder_joint` | Revolute | -1.57 to 1.57 rad | دائیں کندھے کا yaw |
| `right_shoulder_pitch_joint` | Revolute | -3.14 to 3.14 rad | دائیں کندھے کا pitch |
| `right_elbow_joint` | Revolute | 0 to 2.5 rad | دائیں کہنی کا flex |

## مرحلہ 2: Controller Configuration کو سمجھنا

فائل `robot_control_pkg/config/humanoid_controllers.yaml` دو controllers define کرتی ہے:

### Joint State Broadcaster

تمام joints کی موجودہ حالت کو `/joint_states` topic پر شائع کرتا ہے۔

```yaml
joint_state_broadcaster:
  ros__parameters:
    joints:
      - left_shoulder_joint
      - left_shoulder_pitch_joint
      - left_elbow_joint
      - right_shoulder_joint
      - right_shoulder_pitch_joint
      - right_elbow_joint
```

### Joint Group Controller

تمام joints کے لیے position commands قبول کرتا ہے۔

```yaml
joint_group_controller:
  ros__parameters:
    joints:
      - left_shoulder_joint
      - left_shoulder_pitch_joint
      - left_elbow_joint
      - right_shoulder_joint
      - right_shoulder_pitch_joint
      - right_elbow_joint
    interface_name: position
```

**Command Topic**: `/joint_group_controller/commands`
**Message Type**: `std_msgs/msg/Float64MultiArray`

## مرحلہ 3: ROS 2 Package بنائیں

```bash
# Navigate to your ROS 2 workspace
cd ~/ros2_ws

# Copy robot_control_pkg to src/ (if not already there)
# cp -r /path/to/robot_control_pkg ~/ros2_ws/src/

# Build the package
colcon build --packages-select robot_control_pkg

# Source the workspace
source install/setup.bash
```

## مرحلہ 4: Simulation شروع کریں

ایک terminal کھولیں اور humanoid robot کے ساتھ Gazebo launch کریں:

```bash
ros2 launch robot_control_pkg humanoid_gazebo.launch.py
```

یہ command:
1. Gazebo simulator شروع کرتا ہے
2. Humanoid robot کو position (0, 0, 0.5) پر spawn کرتا ہے
3. Robot state publisher load کرتا ہے
4. Controller manager شروع کرتا ہے
5. Joint controllers کو activate کرتا ہے

**متوقع نتیجہ:**
- Gazebo window کھلتی ہے
- Humanoid robot ہوا میں معلق نظر آتا ہے
- Console controller initialization messages دکھاتا ہے

## مرحلہ 5: دستیاب Topics کا معائنہ کریں

نئے terminal میں، active ROS 2 topics کی فہرست بنائیں:

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic list
```

**اہم Topics:**
```
/joint_group_controller/commands  # Command input
/joint_states                     # Joint state output
/robot_description                # URDF model
/tf                               # Transform tree
```

Joint command topic check کریں:

```bash
ros2 topic info /joint_group_controller/commands
```

**نتیجہ:**
```
Type: std_msgs/msg/Float64MultiArray
Publisher count: 0
Subscription count: 1
```

## مرحلہ 6: Manual Joint Control (ٹیسٹنگ)

Automated node چلانے سے پہلے، manual control test کریں:

```bash
# Command all joints to zero position (neutral pose)
ros2 topic pub --once /joint_group_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

# Wave the left arm (left shoulder yaw = 1.0 rad)
ros2 topic pub --once /joint_group_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [1.0, -1.0, 0.5, 0.0, -1.0, 0.5]}"
```

Robot کے بازوؤں کو Gazebo میں حرکت کرتے ہوئے **مشاہدہ کریں**!

## مرحلہ 7: Joint Commander Node چلائیں

ہمارا custom node `joint_commander_node.py` coordinated motion patterns کے ساتھ joint control کو automate کرتا ہے۔

### Node Code کے اہم نکات

```python
class JointCommanderNode(Node):
    def __init__(self):
        super().__init__('joint_commander_node')

        # Create publisher
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/joint_group_controller/commands',
            10
        )

        # Timer for periodic publishing (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = Float64MultiArray()

        # Generate sinusoidal motion
        left_shoulder_yaw = 0.5 * math.sin(2 * math.pi * 0.5 * self.time)
        # ... (compute other joints)

        msg.data = [left_shoulder_yaw, ...]
        self.publisher_.publish(msg)
```

### Node چلائیں

```bash
# In a new terminal (keep Gazebo running)
source ~/ros2_ws/install/setup.bash
ros2 run robot_control_pkg joint_commander_node
```

**آپ کو کیا نظر آنا چاہیے:**
- Console logs شائع شدہ joint commands دکھاتے ہوئے
- Humanoid بازو coordinated sine-wave patterns میں لہرا رہے ہوں
- بائیں اور دائیں بازو opposite phases میں حرکت کر رہے ہوں

**مثال کے طور پر نتیجہ:**
```
[INFO] [joint_commander_node]: Joint Commander Node initialized
[INFO] [joint_commander_node]: Publishing joint commands (t=0.0s):
    L_shoulder=[0.00, -1.00], L_elbow=0.50, R_shoulder=[0.00, -1.00], R_elbow=0.50
[INFO] [joint_commander_node]: Publishing joint commands (t=2.0s):
    L_shoulder=[0.48, -0.85], L_elbow=0.65, R_shoulder=[-0.48, -0.70], R_elbow=0.35
```

## مرحلہ 8: Joint States کی نگرانی کریں

جب node چل رہا ہو، تو حقیقی joint positions کی نگرانی کریں:

```bash
ros2 topic echo /joint_states
```

**نمونہ نتیجہ:**
```yaml
header:
  stamp:
    sec: 1234567
    nanosec: 890000000
  frame_id: ''
name:
- left_shoulder_joint
- left_shoulder_pitch_joint
- left_elbow_joint
- right_shoulder_joint
- right_shoulder_pitch_joint
- right_elbow_joint
position: [0.487, -0.853, 0.647, -0.487, -0.701, 0.353]
velocity: [0.31, 0.15, 0.09, -0.31, 0.18, -0.09]
effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

## مرحلہ 9: RViz کے ساتھ Visualize کریں (اختیاری)

Robot model اور transforms کو visualize کرنے کے لیے RViz launch کریں:

```bash
rviz2
```

**Configuration:**
1. **Fixed Frame** کو `base_link` پر سیٹ کریں
2. **RobotModel** display شامل کریں
3. **TF** display شامل کریں تاکہ coordinate frames دیکھیں
4. Robot کو Gazebo کے ساتھ sync میں حرکت کرتے ہوئے observe کریں

## ROS 2 Control Framework کو سمجھنا

### ros2_control Architecture

```
User Node (Python/C++)
        ↓
  Controller Manager
        ↓
  Forward Command Controller ← Controller Config (YAML)
        ↓
  Hardware Interface (Gazebo Plugin)
        ↓
  Simulated Robot (Gazebo) / Real Hardware
```

### کلیدی اجزاء

1. **Controller Manager**: تمام controllers کو manage کرنے والا مرکزی hub
2. **Controllers**: Commands کو hardware interfaces میں translate کرتے ہیں (position، velocity، effort)
3. **Hardware Interface**: Sim/real hardware کے لیے abstraction layer
4. **Resource Manager**: Joint states اور commands کو manage کرتا ہے

## مسائل کا حل

### Controllers Load نہیں ہو رہے

**علامت:** Joint commands کا کوئی اثر نہیں۔

**حل:**
```bash
# Check controller status
ros2 control list_controllers

# Restart controllers
ros2 control load_controller --set-state active joint_group_controller
```

### Robot Gazebo میں گر رہا ہے

**علامت:** Robot فوری طور پر collapse ہو جاتا ہے۔

**حل:** یقینی بنائیں کہ spawn height (`z_pose`) کافی ہے:
```bash
ros2 launch robot_control_pkg humanoid_gazebo.launch.py z_pose:=1.0
```

### Joint Limits تجاوز ہو گئیں

**علامت:** Joint limits کے بارے میں warning messages۔

**حل:** `humanoid.urdf` میں joint limit definitions check کریں اور یقینی بنائیں کہ commands حدود میں رہیں۔

## مشقیں

### مشق 1: Custom Motion Pattern

`joint_commander_node.py` کو modify کریں تاکہ صرف دائیں بازو کے لیے "waving" motion بنائیں۔

**اشارہ:** بائیں بازو کے joints کو constant values پر سیٹ کریں، صرف دائیں بازو کو vary کریں۔

### مشق 2: مختلف Frequencies

Motion frequency کو تبدیل کریں تاکہ بازو تیز یا آہستہ لہرائیں۔

**تبدیلی:** `self.motion_frequency = 0.5` کو `self.motion_frequency = 2.0` میں بدلیں

### مشق 3: Manual Control Interface

Arrow keys استعمال کرتے ہوئے individual joints کو control کرنے کے لیے ایک سادہ keyboard interface بنائیں۔

**Packages تلاش کریں:** Python کے لیے `pynput` یا `keyboard`

## کلیدی نکات

1. **ROS 2 Topics** nodes کے درمیان publish-subscribe communication کو enable کرتے ہیں
2. **Controller Manager** معیاری انداز میں joint control کو orchestrate کرتا ہے
3. **URDF models** robot کی ساخت اور joint کی constraints کی تعریف کرتے ہیں
4. **Gazebo simulation** محفوظ testing ماحول فراہم کرتا ہے
5. **ros2_control** framework hardware کے فرق کو abstract کرتا ہے

## اگلے قدم

اب جب آپ individual joints کو control کر سکتے ہیں، آپ تیار ہیں:
- Robot میں sensors شامل کریں (cameras، LiDAR)
- Navigation behaviors implement کریں
- End-effector control کے لیے inverse kinematics دریافت کریں

[اگلا: Simulation میں Humanoid Navigation →](humanoid-navigation.md)

## حوالہ جات

- ROS 2 Control Documentation: https://control.ros.org/
- Gazebo-ROS Integration: https://github.com/ros-simulation/gazebo_ros_pkgs
- std_msgs Documentation: https://docs.ros2.org/latest/api/std_msgs/

---

**Code Files کا حوالہ:**
- Joint Commander Node: `robot_control_pkg/robot_control_pkg/joint_commander_node.py`
- Controller Config: `robot_control_pkg/config/humanoid_controllers.yaml`
- Launch File: `robot_control_pkg/launch/humanoid_gazebo.launch.py`
- URDF Model: `robot_control_pkg/urdf/humanoid.urdf`
