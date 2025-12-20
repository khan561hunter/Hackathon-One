---
id: ros2-intro
title: روبوٹ آپریٹنگ سسٹم 2 کا تعارف
sidebar_label: ROS 2 کی بنیادیں
sidebar_position: 2
---

# ROS 2 کا تعارف: روبوٹ آپریٹنگ سسٹم

## ROS 2 کیا ہے؟

**ROS 2 (روبوٹ آپریٹنگ سسٹم 2)** روبوٹ سافٹ ویئر لکھنے کے لیے ایک لچکدار فریم ورک ہے۔ اس کے نام کے باوجود، یہ روایتی معنیٰ میں آپریٹنگ سسٹم نہیں ہے، بلکہ ایک middleware ہے جو فراہم کرتا ہے:

- روبوٹ اجزاء کے درمیان **مواصلات کا بنیادی ڈھانچہ**
- سینسر ڈیٹا اور commands کے لیے **معیاری پیغام کی شکلیں**
- روبوٹ کی ترقی کے لیے **ٹولز اور لائبریریاں**
- ماڈیولر روبوٹکس سافٹ ویئر کے لیے **پیکج مینجمنٹ**

ROS 2، ROS کی اگلی نسل ہے، جسے ROS 1 کی محدودیات کو حل کرنے کے لیے بنیاد سے دوبارہ ڈیزائن کیا گیا ہے، جس میں توجہ ہے:
- ریئل ٹائم سسٹمز
- ملٹی روبوٹ سسٹمز
- چھوٹے embedded platforms
- پروڈکشن ماحول
- سیکیورٹی

## ROS 2 کے بنیادی تصورات

### 1. Nodes

ایک **node** ایک واحد مقصد والا قابل عمل پروگرام ہے جو ایک مخصوص کام انجام دیتا ہے۔ مثالیں:
- ایک node جو کیمرے سے ڈیٹا پڑھتا ہے
- ایک node جو تصاویر پروسیس کرتا ہے
- ایک node جو motors کنٹرول کرتا ہے
- ایک node جو paths کی منصوبہ بندی کرتا ہے

**ڈیزائن فلاسفی**: ایک بڑے monolithic پروگرام کی بجائے، ROS آپ کے روبوٹ کی فعالیت کو بہت سے چھوٹے، focused nodes میں تقسیم کرنے کی حوصلہ افزائی کرتا ہے جو ایک دوسرے کے ساتھ بات چیت کرتے ہیں۔

### 2. Topics

**Topics** نام والی buses ہیں جن پر nodes پیغامات کا تبادلہ کرتے ہیں۔ Topics ایک **publish-subscribe** پیٹرن لاگو کرتے ہیں:

- **Publishers** ایک topic کو پیغامات بھیجتے ہیں
- **Subscribers** ایک topic سے پیغامات وصول کرتے ہیں
- Many-to-many مواصلات (متعدد publishers، متعدد subscribers)

مثال:
```
┌─────────────┐      /camera/image      ┌──────────────┐
│   Camera    │ ──────────────────────> │    Image     │
│    Node     │      (publishes)        │  Processing  │
└─────────────┘                         │     Node     │
                                        └──────────────┘
```

### 3. Services

**Services** ایک **request-response** پیٹرن لاگو کرتی ہیں:
- ایک node سروس کو **request** بھیجتا ہے
- سروس اسے پروسیس کرتی ہے اور **response** واپس کرتی ہے
- Synchronous مواصلات (client response کا انتظار کرتا ہے)

مثال: ایک node inverse kinematics کے calculation کی درخواست کرتا ہے اور joint angles کے نتیجے کا انتظار کرتا ہے۔

### 4. Actions

**Actions** طویل المیعاد کاموں کے لیے ہیں جو فراہم کرتے ہیں:
- عمل درآمد کے دوران **Feedback**
- کام کو **منسوخ** کرنے کی صلاحیت
- مکمل ہونے پر **Result**

مثال: ایک navigation action جو robot کی پیشرفت پر متواتر updates فراہم کرتا ہے۔

### 5. Parameters

**Parameters** configuration کی قدریں ہیں جو ہو سکتی ہیں:
- Launch کے وقت سیٹ کی جائیں
- Runtime کے دوران dynamically تبدیل کی جائیں
- دوسرے nodes سے query کی جائیں

مثال: زیادہ سے زیادہ رفتار، PID controller gains، sensor calibration values۔

## ROS 2 کی تعمیر

### Data Distribution Service (DDS)

ROS 2 اپنی communication middleware کے طور پر **DDS (Data Distribution Service)** استعمال کرتا ہے۔ DDS فراہم کرتا ہے:

- Nodes کی **خودکار دریافت**
- قابل اعتماد یا best-effort مواصلات کے لیے **Quality of Service (QoS)** profiles
- **Real-time** صلاحیتیں
- **Security** فیچرز

### عام DDS Implementations

- **Fast DDS** (زیادہ تر distributions میں default)
- **Cyclone DDS**
- **RTI Connext DDS**

## ROS 2 کو سیٹ اپ کرنا

### تنصیب (Ubuntu 22.04)

```bash
# Add ROS 2 apt repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Source the setup script
source /opt/ros/humble/setup.bash

# Install development tools
sudo apt install python3-colcon-common-extensions python3-rosdep
```

### ROS 2 Workspace بنانا

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build the workspace (empty for now)
colcon build

# Source the workspace
source install/setup.bash
```

## آپ کا پہلا ROS 2 Node

آئیے Python میں ایک سادہ "Hello World" node بنائیں:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class HelloNode(Node):
    def __init__(self):
        super().__init__('hello_node')
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        self.get_logger().info(f'Hello ROS 2! Count: {self.counter}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = HelloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**یہ کیا کرتا ہے:**
1. 'hello_node' نام کا ایک node بناتا ہے
2. ایک timer سیٹ اپ کرتا ہے جو ہر 1 سیکنڈ میں fire ہوتا ہے
3. ہر بار جب timer fire ہو تو ایک message log کرتا ہے
4. ایک counter بڑھاتا ہے

### Node کو چلانا

```bash
# Make the script executable
chmod +x hello_node.py

# Run it
python3 hello_node.py
```

## ROS 2 کمانڈ لائن ٹولز

### Nodes کی فہرست

```bash
ros2 node list
```

### Topics کا معائنہ

```bash
# List all topics
ros2 topic list

# Show message type of a topic
ros2 topic info /camera/image

# Echo messages from a topic
ros2 topic echo /camera/image

# Publish to a topic manually
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
```

### Services کا معائنہ

```bash
# List services
ros2 service list

# Call a service
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
```

### Parameters

```bash
# List parameters of a node
ros2 param list

# Get a parameter value
ros2 param get /my_node my_parameter

# Set a parameter value
ros2 param set /my_node my_parameter 42
```

## ROS 2 Message کی اقسام

ROS 2 strongly-typed messages استعمال کرتا ہے جو `.msg` فائلوں میں define کیے جاتے ہیں۔ عام message packages:

### std_msgs
بنیادی اقسام: `String`, `Int32`, `Float64`, `Bool`

### geometry_msgs
Geometric primitives: `Point`, `Pose`, `Twist`, `Transform`

### sensor_msgs
Sensor ڈیٹا: `Image`, `LaserScan`, `Imu`, `JointState`

### trajectory_msgs
Robot کی حرکت: `JointTrajectory`, `JointTrajectoryPoint`

## Quality of Service (QoS)

ROS 2 QoS profiles کے ذریعے communication کی reliability اور performance پر fine-grained کنٹرول کی اجازت دیتا ہے:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Sensor data QoS (best effort, volatile)
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)

# Create subscriber with custom QoS
self.subscription = self.create_subscription(
    LaserScan,
    '/scan',
    self.listener_callback,
    sensor_qos
)
```

## بہترین طریقے

1. **ہر Node میں ایک مقصد**: ہر node کو ایک کام اچھی طرح کرنا چاہیے
2. **معنی خیز نام استعمال کریں**: Topic اور node کے نام ان کے مقصد کو بیان کریں
3. **مناسب QoS منتخب کریں**: QoS کو اپنے use case سے match کریں (reliability بمقابلہ latency)
4. **Shutdown کو خوبصورتی سے ہینڈل کریں**: Nodes بند ہونے پر resources کو صاف کریں
5. **مناسب طریقے سے Log کریں**: print کی بجائے ROS 2 logging (`get_logger().info()`) استعمال کریں

## اگلے قدم

اب جب آپ ROS 2 کی بنیادیں سمجھ گئے ہیں، آپ تیار ہیں:
1. اپنا پہلا joint control node بنائیں ([ROS 2 Joint Control](ros2-joint-control.md))
2. Digital twin simulation کو سمجھیں ([Digital Twin Introduction](digital-twin-intro.md))
3. Sensors اور actuators کو integrate کریں

## حوالہ جات

- Macenski, S., et al. (2020). The ROS 2 Project. *Robotics and Automation Letters*, 5(2), 512-519.
- Official ROS 2 Documentation: https://docs.ros.org/en/humble/

[اگلا: ROS 2 Joint Control →](ros2-joint-control.md)
