---
id: ros2-intro
title: Introduction to ROS 2
sidebar_label: ROS 2 Fundamentals
sidebar_position: 2
---

# Introduction to ROS 2: The Robot Operating System

## What is ROS 2?

**ROS 2 (Robot Operating System 2)** is a flexible framework for writing robot software. Despite its name, it's not an operating system in the traditional sense, but rather a middleware that provides:

- **Communication infrastructure** between robot components
- **Standard message formats** for sensor data and commands
- **Tools and libraries** for robot development
- **Package management** for modular robotics software

ROS 2 is the next generation of ROS, redesigned from the ground up to address the limitations of ROS 1, with focus on:
- Real-time systems
- Multi-robot systems
- Small embedded platforms
- Production environments
- Security

## Core ROS 2 Concepts

### 1. Nodes

A **node** is a single-purpose executable program that performs one specific task. Examples:
- A node that reads data from a camera
- A node that processes images
- A node that controls motors
- A node that plans paths

**Design Philosophy**: Rather than having one monolithic program, ROS encourages breaking your robot's functionality into many small, focused nodes that communicate with each other.

### 2. Topics

**Topics** are named buses over which nodes exchange messages. Topics implement a **publish-subscribe** pattern:

- **Publishers** send messages to a topic
- **Subscribers** receive messages from a topic
- Many-to-many communication (multiple publishers, multiple subscribers)

Example:
```
┌─────────────┐      /camera/image      ┌──────────────┐
│   Camera    │ ──────────────────────> │    Image     │
│    Node     │      (publishes)        │  Processing  │
└─────────────┘                         │     Node     │
                                        └──────────────┘
```

### 3. Services

**Services** implement a **request-response** pattern:
- A node sends a **request** to a service
- The service processes it and returns a **response**
- Synchronous communication (client waits for response)

Example: A node requests inverse kinematics calculation and waits for the joint angles result.

### 4. Actions

**Actions** are for long-running tasks that provide:
- **Feedback** during execution
- Ability to **cancel** the task
- **Result** when completed

Example: A navigation action that provides periodic updates on the robot's progress toward a goal.

### 5. Parameters

**Parameters** are configuration values that can be:
- Set at launch time
- Changed dynamically during runtime
- Queried by other nodes

Example: Maximum speed, PID controller gains, sensor calibration values.

## ROS 2 Architecture

### Data Distribution Service (DDS)

ROS 2 uses **DDS (Data Distribution Service)** as its communication middleware. DDS provides:

- **Automatic discovery** of nodes
- **Quality of Service (QoS)** profiles for reliable or best-effort communication
- **Real-time** capabilities
- **Security** features

### Common DDS Implementations

- **Fast DDS** (default in most distributions)
- **Cyclone DDS**
- **RTI Connext DDS**

## Setting Up ROS 2

### Installation (Ubuntu 22.04)

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

### Creating a ROS 2 Workspace

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build the workspace (empty for now)
colcon build

# Source the workspace
source install/setup.bash
```

## Your First ROS 2 Node

Let's create a simple "Hello World" node in Python:

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

**What this does:**
1. Creates a node named 'hello_node'
2. Sets up a timer that fires every 1 second
3. Logs a message each time the timer fires
4. Increments a counter

### Running the Node

```bash
# Make the script executable
chmod +x hello_node.py

# Run it
python3 hello_node.py
```

## ROS 2 Command-Line Tools

### Listing Nodes

```bash
ros2 node list
```

### Examining Topics

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

### Examining Services

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

## ROS 2 Message Types

ROS 2 uses strongly-typed messages defined in `.msg` files. Common message packages:

### std_msgs
Basic types: `String`, `Int32`, `Float64`, `Bool`

### geometry_msgs
Geometric primitives: `Point`, `Pose`, `Twist`, `Transform`

### sensor_msgs
Sensor data: `Image`, `LaserScan`, `Imu`, `JointState`

### trajectory_msgs
Robot motion: `JointTrajectory`, `JointTrajectoryPoint`

## Quality of Service (QoS)

ROS 2 allows fine-grained control over communication reliability and performance through QoS profiles:

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

## Best Practices

1. **One Purpose Per Node**: Each node should do one thing well
2. **Use Meaningful Names**: Topic and node names should describe their purpose
3. **Choose Appropriate QoS**: Match QoS to your use case (reliability vs. latency)
4. **Handle Shutdown Gracefully**: Clean up resources when nodes exit
5. **Log Appropriately**: Use ROS 2 logging (`get_logger().info()`) instead of print

## Next Steps

Now that you understand ROS 2 fundamentals, you're ready to:
1. Build your first joint control node ([ROS 2 Joint Control](ros2-joint-control.md))
2. Understand digital twin simulation ([Digital Twin Introduction](digital-twin-intro.md))
3. Integrate sensors and actuators

## References

- Macenski, S., et al. (2020). The ROS 2 Project. *Robotics and Automation Letters*, 5(2), 512-519.
- Official ROS 2 Documentation: https://docs.ros.org/en/humble/

[Next: ROS 2 Joint Control →](ros2-joint-control.md)
