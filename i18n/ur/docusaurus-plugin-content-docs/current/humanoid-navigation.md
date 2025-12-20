---
id: humanoid-navigation
title: Simulation میں Humanoid Navigation
sidebar_label: Humanoid Navigation
sidebar_position: 5
---

# یوزر اسٹوری 2: Simulation میں Humanoid Navigation

## اسٹوری کا مقصد

رکاوٹوں والے کمرے میں robot کو navigate کرتے ہوئے physics simulation، collision detection، اور sensor integration کے بارے میں سیکھیں۔

## سیکھنے کے مقاصد

اس tutorial کے اختتام تک، آپ:
- Robot model میں LiDAR اور depth cameras شامل کریں گے
- Sensor data کو obstacle detection کے لیے استعمال کریں گے
- خودکار navigation behaviors implement کریں گے
- Wall-following اور obstacle avoidance algorithms استعمال کریں گے
- Sensor-based decision making سمجھیں گے

## پیشگی ضروریات

- User Story 1 (ROS 2 Joint Control) مکمل
- ROS 2 Humble انسٹال ہے
- Gazebo simulator انسٹال ہے
- Sensor data processing کی بنیادی سمجھ
- Python NumPy library (`pip install numpy`)

## فوری شروعات

```bash
# Terminal 1: Navigation world launch کریں
ros2 launch robot_control_pkg humanoid_gazebo.launch.py world:=navigation_world.world

# Terminal 2: Sensor processor شروع کریں
ros2 run robot_control_pkg sensor_data_processor_node

# Terminal 3: Navigation controller شروع کریں
ros2 run robot_control_pkg navigation_controller_node

# Terminal 4: Navigation hints monitor کریں
ros2 topic echo /navigation_hints
```

## Architecture کا جائزہ

ہمارا system LiDAR sensor، depth camera، sensor data processor node، navigation controller node اور robot base controller پر مشتمل ہے۔

## Part 1: Sensor Integration کو سمجھنا

### LiDAR Sensor

**LiDAR (Light Detection and Ranging)** laser pulses استعمال کرتے ہوئے objects تک فاصلہ ناپتا ہے، environment کی 2D تصویر بناتا ہے۔

#### URDF میں LiDAR Configuration

```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar_sensor">
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.10</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </ray>
  </sensor>
</gazebo>
```

**کلیدی Parameters:**
- **Samples**: Robot کے گرد 360 readings
- **Range**: 0.1m سے 10m detection range
- **Update Rate**: 10 Hz (فی سیکنڈ 10 scans)
- **Noise**: حقیقی sensor imperfections کی نقل کرتا Gaussian noise

#### LiDAR Data پڑھنا

```python
from sensor_msgs.msg import LaserScan

def lidar_callback(self, msg: LaserScan):
    num_readings = len(msg.ranges)
    front_width = int(num_readings * 30 / 180)
    front_ranges = msg.ranges[:front_width] + msg.ranges[-front_width:]
    min_distance = min(front_ranges)

    if min_distance < 0.5:
        self.get_logger().warn(f'Obstacle {min_distance:.2f}m ahead!')
```

### Depth Camera

**Depth cameras** RGB images فراہم کرتے ہیں depth information کے ساتھ، scene کا 3D point cloud بناتے ہیں۔

## Part 2: Navigation World

Navigation world (`navigation_world.world`) ایک 10m × 10m کا کمرہ ہے جس میں مختلف obstacles ہیں۔

## Part 3: Sensor Data Processing

### Sensor Data Processor Node

یہ node raw sensor data process کرتا ہے اور actionable navigation hints generate کرتا ہے۔

**1. Obstacle Detection**

```python
def lidar_callback(self, msg: LaserScan):
    ranges = np.array(msg.ranges)
    ranges = np.where(np.isinf(ranges), msg.range_max, ranges)

    if front_dist < self.obstacle_threshold:
        self.publish_obstacle_alert(front_dist)
```

**2. Navigation Hint Generation**

```python
def generate_navigation_hints(self, ranges, scan_msg):
    twist = Twist()
    if front_dist < self.safe_distance:
        if left_dist > right_dist:
            twist.angular.z = 0.5
            twist.linear.x = 0.1
        else:
            twist.angular.z = -0.5
            twist.linear.x = 0.1
    else:
        twist.linear.x = 0.3
        twist.angular.z = 0.0
    self.nav_hints_pub.publish(twist)
```

## Part 4: Navigation Controller

### Navigation Behaviors

Navigation controller تین distinct behaviors implement کرتا ہے:

#### 1. Exploration
کھلی جگہ میں آگے بڑھیں

#### 2. Obstacle Avoidance
فوری obstacles پر reactive response

#### 3. Wall Following
دیوار سے consistent فاصلہ برقرار رکھیں

### Navigation Modes

```bash
# Autonomous mode
ros2 run robot_control_pkg navigation_controller_node

# Hint-based mode
ros2 run robot_control_pkg navigation_controller_node --ros-args -p navigation_mode:=hint_based

# Wall-follow mode
ros2 run robot_control_pkg navigation_controller_node --ros-args -p navigation_mode:=wall_follow
```

## کلیدی سبق

1. **Sensor Fusion**: LiDAR اور camera تکمیلی معلومات فراہم کرتے ہیں
2. **Behavior Orchestration**: State machines متعدد navigation behaviors organize کرتی ہیں
3. **Reactive Control**: تیز sensor processing real-time obstacle avoidance enable کرتی ہے
4. **Parameter Tuning**: Navigation quality اچھی طرح tuned thresholds پر منحصر ہے
5. **Modularity**: Sensing، processing، اور control کو الگ کرنا debugging آسان بناتا ہے

## اگلے قدم

اب جب آپ خودکار navigation کر سکتے ہیں:
- Reinforcement learning کے ساتھ AI-powered navigation policies تربیت دیں
- SLAM implement کریں
- Semantic navigation کے لیے object recognition شامل کریں

[اگلا: AI-Robot Brain Training for Locomotion →](isaac-locomotion-training.md)

## حوالہ جات

- Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press.
- LaValle, S. M. (2006). *Planning Algorithms*. Cambridge University Press.
- ROS 2 Navigation Stack: https://navigation.ros.org/
- Gazebo Sensors: https://gazebosim.org/docs/garden/sensors
