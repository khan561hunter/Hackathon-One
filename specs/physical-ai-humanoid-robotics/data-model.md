# Data Models for Physical AI & Humanoid Robotics Book

This document outlines the key data schemas and structures used throughout the "Physical AI & Humanoid Robotics: Embodied Intelligence in the Real World" book. It defines the format of data exchanged between different modules, components, and systems, including robot configurations, ROS 2 messages, VLA pipeline data, and Docusaurus content metadata.

---

## 1. Humanoid Robot Configuration Model

This model defines the structural and kinematic parameters of a generic humanoid robot, adaptable for various simulation environments (Gazebo, Unity, Isaac Sim) and hardware platforms.

```json
{
  "robot_name": "string",
  "model_file": "string",
  "description_format": "URDF | MJCF | USD",
  "kinematic_tree": {
    "root_link": "string",
    "joints": [
      {
        "name": "string",
        "type": "revolute | prismatic | fixed",
        "parent": "string",
        "child": "string",
        "axis": {"x": "float", "y": "float", "z": "float"},
        "limits": {"lower": "float", "upper": "float"}
      }
    ],
    "links": [
      {
        "name": "string",
        "mass": "float",
        "inertia": {"ixx": "float", "iyy": "float", "izz": "float", "ixy": "float", "ixz": "float", "iyz": "float"},
        "visuals": [
          {
            "geometry_type": "mesh | box | cylinder | sphere",
            "geometry_file": "string",
            "material": {"color": "rgba(float, float, float, float)"}
          }
        ],
        "collisions": [
          {
            "geometry_type": "mesh | box | cylinder | sphere",
            "geometry_file": "string"
          }
        ]
      }
    ]
  },
  "sensors": [
    {
      "name": "string",
      "type": "camera | lidar | imu | force_torque",
      "pose": {"x": "float", "y": "float", "z": "float", "roll": "float", "pitch": "float", "yaw": "float"},
      "parameters": {
        "resolution_x": "integer",
        "resolution_y": "integer",
        "fov": "float",
        "min_range": "float",
        "max_range": "float"
      }
    }
  ],
  "actuators": [
    {
      "name": "string",
      "joint_name": "string",
      "control_type": "position | velocity | effort",
      "parameters": {
        "kp": "float",
        "kv": "float",
        "max_effort": "float"
      }
    }
  ]
}
```

---

## 2. ROS 2 Message Formats

Standard and custom ROS 2 message definitions for inter-node communication, including sensor data, joint commands, and navigation goals.

### Sensor Data (e.g., IMU, Camera, Lidar)

#### `sensor_msgs/msg/Imu`
```yaml
# Standard ROS 2 IMU message
std_msgs/Header header

geometry_msgs/Quaternion orientation
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance # Row major about x, y, z axes
```

#### `sensor_msgs/msg/Image`
```yaml
# Standard ROS 2 Camera Image message
std_msgs/Header header

uint32 height
uint32 width
string encoding # E.g., "rgb8", "mono8"
uint8 is_bigendian
uint32 step # Full row length in bytes
uint8[] data
```

### Joint Commands

#### `std_msgs/msg/Float64MultiArray` (for simple joint position/velocity/effort commands)
```yaml
# Generic multi-array for joint commands
std_msgs/MultiArrayLayout layout
float64[] data # Array of target joint values
```

#### `trajectory_msgs/msg/JointTrajectory` (for complex trajectory execution)
```yaml
# Standard ROS 2 Joint Trajectory message
std_msgs/Header header
string[] joint_names
trajectory_msgs/JointTrajectoryPoint[] points
```

### Navigation Goal

#### `geometry_msgs/msg/PoseStamped` (for simple goal poses)
```yaml
# Standard ROS 2 Pose Stamped message
std_msgs/Header header
geometry_msgs/Pose pose
```

---

## 3. VLA Pipeline Data Structures

Data formats used in the Vision-Language-Action (VLA) pipeline, from voice input to interpreted actions and simulation commands.

### Voice Command Input

```json
{
  "timestamp": "ISO 8601 string",
  "audio_data_base64": "string", # Base64 encoded audio byte array
  "language_code": "string" # E.g., "en-US"
}
```

### Transcribed Text (Whisper Output)

```json
{
  "timestamp": "ISO 8601 string",
  "transcribed_text": "string",
  "confidence": "float"
}
```

### Interpreted Action Sequence (GPT Output)

```json
{
  "timestamp": "ISO 8601 string",
  "raw_command": "string", # Original transcribed text
  "interpreted_actions": [
    {
      "action_type": "move_arm | grasp | speak | navigate_to",
      "target": {
        "object_name": "string",
        "coordinates": {"x": "float", "y": "float", "z": "float"},
        "predefined_pose": "string"
      },
      "parameters": {
        "speed": "float",
        "force": "float",
        "phrase": "string"
      },
      "constraints": [
        {"type": "collision_avoidance", "value": "true"}
      ]
    }
  ],
  "confidence": "float",
  "model_version": "string"
}
```

---

## 4. Docusaurus Structured Content Metadata

Metadata embedded within Docusaurus Markdown files to facilitate content organization, search, and dynamic rendering.

### Docusaurus Page Front Matter

```yaml
---
id: "chapter-ros2-intro"
title: "Introduction to ROS 2"
sidebar_label: "ROS 2 Fundamentals"
sidebar_position: 1
slug: /ros2/intro
authors: ["Claude Code", "Human Author"]
tags: ["ROS2", "Robotics", "Fundamentals"]
---
```

### Code Snippet Metadata

```json
{
  "language": "python | cpp | javascript",
  "file_path": "string", # Path to external code example
  "start_line": "integer",
  "end_line": "integer",
  "description": "string",
  "dependencies": ["string"]
}
```

### Simulation Demo Metadata

```json
{
  "simulator": "Gazebo | Unity | Isaac Sim",
  "video_file": "string", # Path to video in /static
  "description": "string",
  "duration_seconds": "integer",
  "code_example_ref": "string" # Link to relevant code snippet
}
```

---

## 5. Digital Twin Data Pipeline

Data flow and structure for capturing and processing information from digital twin simulations.

```json
{
  "simulation_id": "string",
  "timestamp": "ISO 8601 string",
  "robot_state": {
    "joint_positions": {"joint_name": "float"},
    "joint_velocities": {"joint_name": "float"},
    "link_poses": {"link_name": {"x": "float", "y": "float", "z": "float", "qx": "float", "qy": "float", "qz": "float", "qw": "float"}}
  },
  "sensor_readings": {
    "camera": [
      {
        "sensor_name": "string",
        "image_data_base64": "string",
        "encoding": "string"
      }
    ],
    "lidar": [
      {
        "sensor_name": "string",
        "point_cloud_data": "array of floats", # E.g., [x1, y1, z1, x2, y2, z2, ...]
        "min_range": "float",
        "max_range": "float"
      }
    ]
  },
  "environmental_data": {
    "lighting": "float",
    "gravity": "float",
    "obstacles": [
      {"id": "string", "pose": {"x": "float", "y": "float", "z": "float", "roll": "float", "pitch": "float", "yaw": "float"}}
    ]
  }
}
```

---

## 6. Isaac Sim Observation/Action Format

Data structures for observations received by an AI agent from Isaac Sim and actions sent to control the simulated robot.

### Observation Space

```json
{
  "robot_joint_positions": "array of floats",
  "robot_joint_velocities": "array of floats",
  "end_effector_pose": {"x": "float", "y": "float", "z": "float", "qx": "float", "qy": "float", "qz": "float", "qw": "float"},
  "camera_rgb": "array of integers", # Flattened image array
  "camera_depth": "array of floats", # Flattened depth image array
  "lidar_points": "array of floats", # Flattened point cloud array
  "object_detections": [
    {
      "class_id": "integer",
      "bounding_box": ["x_min", "y_min", "x_max", "y_max"],
      "confidence": "float",
      "pose": {"x": "float", "y": "float", "z": "float", "qx": "float", "qy": "float", "qz": "float", "qw": "float"}
    }
  ],
  "goal_position": {"x": "float", "y": "float", "z": "float"}
}
```

### Action Space

```json
{
  "joint_position_targets": "array of floats", # Target positions for each joint
  "joint_velocity_targets": "array of floats", # Target velocities for each joint
  "end_effector_delta_pose": {"dx": "float", "dy": "float", "dz": "float", "droll": "float", "dpitch": "float", "dyaw": "float"}, # Relative end-effector movement
  "gripper_command": "open | close | float (width)"
}
```
