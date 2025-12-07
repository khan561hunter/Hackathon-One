#!/usr/bin/env python3
"""
Joint Commander Node

This ROS 2 node publishes joint position commands to control the humanoid robot's joints.
It demonstrates basic joint control by commanding the robot's arms to move through
a sequence of positions.

Usage:
    ros2 run robot_control_pkg joint_commander_node

Topics:
    Publishers:
        /joint_group_controller/commands (std_msgs/msg/Float64MultiArray):
            Joint position commands for all controllable joints
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math


class JointCommanderNode(Node):
    """
    ROS 2 node for commanding joint positions of the humanoid robot.

    This node publishes joint commands at a regular interval, moving the robot's
    arms through a simple motion pattern to demonstrate joint control.
    """

    def __init__(self):
        super().__init__('joint_commander_node')

        # Create publisher for joint commands
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/joint_group_controller/commands',
            10
        )

        # Control loop timer (10 Hz)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Joint names in order (matching the URDF)
        self.joint_names = [
            'left_shoulder_joint',
            'left_shoulder_pitch_joint',
            'left_elbow_joint',
            'right_shoulder_joint',
            'right_shoulder_pitch_joint',
            'right_elbow_joint'
        ]

        # Motion state
        self.time = 0.0
        self.motion_amplitude = 0.5  # radians
        self.motion_frequency = 0.5  # Hz

        self.get_logger().info('Joint Commander Node initialized')
        self.get_logger().info(f'Publishing to: /joint_group_controller/commands')
        self.get_logger().info(f'Controlling {len(self.joint_names)} joints')

    def timer_callback(self):
        """
        Timer callback that publishes joint commands.

        Creates a sinusoidal motion pattern for the robot's arms to demonstrate
        coordinated joint control.
        """
        msg = Float64MultiArray()

        # Generate sinusoidal motion for demonstration
        # Left shoulder joints (wave pattern)
        left_shoulder_yaw = self.motion_amplitude * math.sin(2 * math.pi * self.motion_frequency * self.time)
        left_shoulder_pitch = -1.0 + 0.3 * math.sin(2 * math.pi * self.motion_frequency * self.time)
        left_elbow = 0.5 + 0.3 * math.sin(2 * math.pi * self.motion_frequency * self.time)

        # Right shoulder joints (opposite phase wave)
        right_shoulder_yaw = -self.motion_amplitude * math.sin(2 * math.pi * self.motion_frequency * self.time)
        right_shoulder_pitch = -1.0 + 0.3 * math.cos(2 * math.pi * self.motion_frequency * self.time)
        right_elbow = 0.5 + 0.3 * math.cos(2 * math.pi * self.motion_frequency * self.time)

        # Pack joint commands
        msg.data = [
            left_shoulder_yaw,
            left_shoulder_pitch,
            left_elbow,
            right_shoulder_yaw,
            right_shoulder_pitch,
            right_elbow
        ]

        # Publish commands
        self.publisher_.publish(msg)

        # Log every 2 seconds
        if int(self.time) % 2 == 0 and (self.time - int(self.time)) < self.timer_period:
            self.get_logger().info(
                f'Publishing joint commands (t={self.time:.1f}s): '
                f'L_shoulder=[{left_shoulder_yaw:.2f}, {left_shoulder_pitch:.2f}], '
                f'L_elbow={left_elbow:.2f}, '
                f'R_shoulder=[{right_shoulder_yaw:.2f}, {right_shoulder_pitch:.2f}], '
                f'R_elbow={right_elbow:.2f}'
            )

        # Update time
        self.time += self.timer_period


def main(args=None):
    """
    Main entry point for the joint commander node.

    Args:
        args: Command line arguments (default: None)
    """
    rclpy.init(args=args)

    try:
        node = JointCommanderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        node.get_logger().info('Shutting down Joint Commander Node')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
