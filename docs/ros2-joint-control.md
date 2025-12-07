---
id: ros2-joint-control
title: ROS 2 Basic Joint Control
sidebar_label: ROS 2 Joint Control
sidebar_position: 4
---

# User Story 1: ROS 2 Basic Joint Control

## Story Goal

Understand the fundamentals of ROS 2 nodes, topics, and services by controlling a simulated humanoid joint.

## Learning Objectives

By the end of this tutorial, you will:
- Create and run ROS 2 Python nodes
- Publish joint commands using standard message types
- Configure and launch robot controllers
- Observe robot behavior in Gazebo simulation
- Understand the ROS 2 control framework

## Prerequisites

- ROS 2 Humble installed
- Gazebo simulator installed
- `robot_control_pkg` package built
- Basic Python programming knowledge

## Architecture Overview

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

## Step 1: Review the Humanoid URDF

Our humanoid robot model is defined in `robot_control_pkg/urdf/humanoid.urdf` with:

- **Base Link**: Foundation of the robot
- **Torso**: Main body
- **Left Arm**: Shoulder (yaw + pitch) + Elbow = 3 DOF
- **Right Arm**: Shoulder (yaw + pitch) + Elbow = 3 DOF
- **Total**: 6 actuated joints

### Controllable Joints

| Joint Name | Type | Range | Description |
|------------|------|-------|-------------|
| `left_shoulder_joint` | Revolute | -1.57 to 1.57 rad | Left shoulder yaw (side-to-side) |
| `left_shoulder_pitch_joint` | Revolute | -3.14 to 3.14 rad | Left shoulder pitch (up-down) |
| `left_elbow_joint` | Revolute | 0 to 2.5 rad | Left elbow flex |
| `right_shoulder_joint` | Revolute | -1.57 to 1.57 rad | Right shoulder yaw |
| `right_shoulder_pitch_joint` | Revolute | -3.14 to 3.14 rad | Right shoulder pitch |
| `right_elbow_joint` | Revolute | 0 to 2.5 rad | Right elbow flex |

## Step 2: Understanding the Controller Configuration

The file `robot_control_pkg/config/humanoid_controllers.yaml` defines two controllers:

### Joint State Broadcaster

Publishes the current state of all joints to the `/joint_states` topic.

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

Accepts position commands for all joints.

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

## Step 3: Build the ROS 2 Package

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

## Step 4: Launch the Simulation

Open a terminal and launch Gazebo with the humanoid robot:

```bash
ros2 launch robot_control_pkg humanoid_gazebo.launch.py
```

This command:
1. Starts Gazebo simulator
2. Spawns the humanoid robot at position (0, 0, 0.5)
3. Loads the robot state publisher
4. Starts the controller manager
5. Activates joint controllers

**Expected Output:**
- Gazebo window opens
- Humanoid robot appears suspended in the air
- Console shows controller initialization messages

## Step 5: Examine Available Topics

In a new terminal, list active ROS 2 topics:

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic list
```

**Key Topics:**
```
/joint_group_controller/commands  # Command input
/joint_states                     # Joint state output
/robot_description                # URDF model
/tf                               # Transform tree
```

Check the joint command topic:

```bash
ros2 topic info /joint_group_controller/commands
```

**Output:**
```
Type: std_msgs/msg/Float64MultiArray
Publisher count: 0
Subscription count: 1
```

## Step 6: Manual Joint Control (Testing)

Before running the automated node, test manual control:

```bash
# Command all joints to zero position (neutral pose)
ros2 topic pub --once /joint_group_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

# Wave the left arm (left shoulder yaw = 1.0 rad)
ros2 topic pub --once /joint_group_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [1.0, -1.0, 0.5, 0.0, -1.0, 0.5]}"
```

**Observe** the robot's arms moving in Gazebo!

## Step 7: Run the Joint Commander Node

Our custom node `joint_commander_node.py` automates joint control with coordinated motion patterns.

### Node Code Highlights

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

### Run the Node

```bash
# In a new terminal (keep Gazebo running)
source ~/ros2_ws/install/setup.bash
ros2 run robot_control_pkg joint_commander_node
```

**What You Should See:**
- Console logs showing published joint commands
- Humanoid arms waving in coordinated sine-wave patterns
- Left and right arms moving in opposite phases

**Example Output:**
```
[INFO] [joint_commander_node]: Joint Commander Node initialized
[INFO] [joint_commander_node]: Publishing joint commands (t=0.0s):
    L_shoulder=[0.00, -1.00], L_elbow=0.50, R_shoulder=[0.00, -1.00], R_elbow=0.50
[INFO] [joint_commander_node]: Publishing joint commands (t=2.0s):
    L_shoulder=[0.48, -0.85], L_elbow=0.65, R_shoulder=[-0.48, -0.70], R_elbow=0.35
```

## Step 8: Monitor Joint States

While the node is running, monitor the actual joint positions:

```bash
ros2 topic echo /joint_states
```

**Sample Output:**
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

## Step 9: Visualize with RViz (Optional)

Launch RViz to visualize the robot model and transforms:

```bash
rviz2
```

**Configuration:**
1. Set **Fixed Frame** to `base_link`
2. Add **RobotModel** display
3. Add **TF** display to see coordinate frames
4. Observe the robot moving in sync with Gazebo

## Understanding the ROS 2 Control Framework

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

### Key Components

1. **Controller Manager**: Central hub managing all controllers
2. **Controllers**: Translate commands to hardware interfaces (position, velocity, effort)
3. **Hardware Interface**: Abstraction layer for sim/real hardware
4. **Resource Manager**: Manages joint states and commands

## Troubleshooting

### Controllers Not Loading

**Symptom:** Joint commands have no effect.

**Solution:**
```bash
# Check controller status
ros2 control list_controllers

# Restart controllers
ros2 control load_controller --set-state active joint_group_controller
```

### Robot Falls in Gazebo

**Symptom:** Robot collapses immediately.

**Solution:** Ensure the spawn height (`z_pose`) is sufficient:
```bash
ros2 launch robot_control_pkg humanoid_gazebo.launch.py z_pose:=1.0
```

### Joint Limits Exceeded

**Symptom:** Warning messages about joint limits.

**Solution:** Check `humanoid.urdf` for joint limit definitions and ensure commands stay within bounds.

## Exercises

### Exercise 1: Custom Motion Pattern

Modify `joint_commander_node.py` to create a "waving" motion for just the right arm.

**Hint:** Set left arm joints to constant values, vary only right arm.

### Exercise 2: Different Frequencies

Change the motion frequency to make arms wave faster or slower.

**Modify:** `self.motion_frequency = 0.5` to `self.motion_frequency = 2.0`

### Exercise 3: Manual Control Interface

Create a simple keyboard interface to control individual joints using arrow keys.

**Packages to explore:** `pynput` or `keyboard` for Python

## Key Takeaways

1. **ROS 2 Topics** enable publish-subscribe communication between nodes
2. **Controller Manager** orchestrates joint control in a standardized way
3. **URDF models** define robot structure and joint constraints
4. **Gazebo simulation** provides a safe testing environment
5. **ros2_control** framework abstracts hardware differences

## Next Steps

Now that you can control individual joints, you're ready to:
- Add sensors to the robot (cameras, LiDAR)
- Implement navigation behaviors
- Explore inverse kinematics for end-effector control

[Next: Humanoid Navigation in Simulation →](humanoid-navigation.md)

## References

- ROS 2 Control Documentation: https://control.ros.org/
- Gazebo-ROS Integration: https://github.com/ros-simulation/gazebo_ros_pkgs
- std_msgs Documentation: https://docs.ros2.org/latest/api/std_msgs/

---

**Code Files Reference:**
- Joint Commander Node: `robot_control_pkg/robot_control_pkg/joint_commander_node.py`
- Controller Config: `robot_control_pkg/config/humanoid_controllers.yaml`
- Launch File: `robot_control_pkg/launch/humanoid_gazebo.launch.py`
- URDF Model: `robot_control_pkg/urdf/humanoid.urdf`
