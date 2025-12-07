# Robot Control Package

ROS 2 package for humanoid robot control and simulation.

## Overview

This package provides the core functionality for controlling a humanoid robot in simulation and on real hardware. It includes nodes for joint control, sensor processing, navigation, and manipulation.

## Package Structure

```
robot_control_pkg/
├── config/              # Configuration files (YAML)
├── launch/              # Launch files
├── robot_control_pkg/   # Python source code
│   ├── __init__.py
│   ├── joint_commander_node.py
│   ├── sensor_data_processor_node.py
│   ├── navigation_controller_node.py
│   └── manipulation_action_server.py
├── resource/            # ROS 2 resource files
├── urdf/                # Robot URDF models
├── worlds/              # Gazebo world files
├── package.xml          # Package dependencies
├── setup.py             # Python package setup
└── setup.cfg            # Setup configuration
```

## Dependencies

- ROS 2 (Humble Hawksbill or later)
- Python 3.8+
- Gazebo (Garden or later)
- rclpy
- std_msgs
- sensor_msgs
- geometry_msgs
- trajectory_msgs

## Building

From your ROS 2 workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select robot_control_pkg
source install/setup.bash
```

## Usage

### Joint Commander Node

Control individual joints of the robot:

```bash
ros2 run robot_control_pkg joint_commander_node
```

### Sensor Data Processor

Process sensor data from LiDAR and cameras:

```bash
ros2 run robot_control_pkg sensor_data_processor_node
```

### Navigation Controller

Execute navigation commands:

```bash
ros2 run robot_control_pkg navigation_controller_node
```

### Manipulation Action Server

Handle object manipulation actions:

```bash
ros2 run robot_control_pkg manipulation_action_server
```

## Launching Simulations

Launch the humanoid in Gazebo:

```bash
ros2 launch robot_control_pkg humanoid_gazebo.launch.py
```

## Development Status

This package is under active development as part of the "Physical AI & Humanoid Robotics" book project.

## License

MIT
