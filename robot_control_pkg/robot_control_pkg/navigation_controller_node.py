#!/usr/bin/env python3
"""
Navigation Controller Node

This ROS 2 node implements basic navigation behavior for the humanoid robot,
using obstacle avoidance and wall-following strategies.

Usage:
    ros2 run robot_control_pkg navigation_controller_node

Topics:
    Subscribers:
        /navigation_hints (geometry_msgs/msg/Twist): Suggested navigation velocities
        /humanoid/scan (sensor_msgs/msg/LaserScan): LiDAR scan data

    Publishers:
        /cmd_vel (geometry_msgs/msg/Twist): Velocity commands to move the robot
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import math


class NavigationControllerNode(Node):
    """
    ROS 2 node for autonomous navigation with obstacle avoidance.

    Implements wall-following and obstacle avoidance behaviors based on
    LiDAR sensor data and navigation hints.
    """

    def __init__(self):
        super().__init__('navigation_controller_node')

        # Declare parameters
        self.declare_parameter('navigation_mode', 'autonomous')
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)

        # Get parameters
        self.nav_mode = self.get_parameter('navigation_mode').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value

        # Subscribe to navigation hints
        self.hints_subscription = self.create_subscription(
            Twist,
            '/navigation_hints',
            self.hints_callback,
            10
        )

        # Subscribe to LiDAR for direct control
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/humanoid/scan',
            self.lidar_callback,
            10
        )

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control timer (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)

        # State
        self.current_hint = Twist()
        self.latest_scan = None
        self.navigation_state = 'exploring'  # exploring, avoiding, wall_following

        # Obstacle avoidance parameters
        self.obstacle_threshold = 0.6  # meters
        self.wall_follow_distance = 0.8  # meters

        self.get_logger().info(f'Navigation Controller Node initialized')
        self.get_logger().info(f'Mode: {self.nav_mode}')
        self.get_logger().info(f'Max speeds: linear={self.max_linear}, angular={self.max_angular}')

    def hints_callback(self, msg: Twist):
        """
        Receive navigation hints from sensor processor.

        Args:
            msg: Twist message with suggested velocities
        """
        self.current_hint = msg

    def lidar_callback(self, msg: LaserScan):
        """
        Store latest LiDAR data for navigation decisions.

        Args:
            msg: LaserScan message
        """
        self.latest_scan = msg

    def control_loop(self):
        """
        Main control loop that executes navigation behavior.

        Runs at 20 Hz and publishes velocity commands based on current state
        and sensor data.
        """
        if self.latest_scan is None:
            return

        # Process scan data
        ranges = np.array(self.latest_scan.ranges)
        ranges = np.where(np.isinf(ranges), self.latest_scan.range_max, ranges)

        # Determine navigation behavior
        cmd = Twist()

        if self.nav_mode == 'autonomous':
            cmd = self.autonomous_navigation(ranges)
        elif self.nav_mode == 'hint_based':
            cmd = self.current_hint
        elif self.nav_mode == 'wall_follow':
            cmd = self.wall_following_behavior(ranges)

        # Apply speed limits
        cmd.linear.x = max(-self.max_linear, min(self.max_linear, cmd.linear.x))
        cmd.angular.z = max(-self.max_angular, min(self.max_angular, cmd.angular.z))

        # Publish command
        self.cmd_vel_pub.publish(cmd)

    def autonomous_navigation(self, ranges: np.ndarray) -> Twist:
        """
        Autonomous navigation behavior combining exploration and obstacle avoidance.

        Args:
            ranges: Array of LiDAR range measurements

        Returns:
            Twist: Velocity command
        """
        cmd = Twist()
        num_readings = len(ranges)

        # Divide into sectors
        front_width = int(num_readings * 0.15)  # 15% front sector
        front_center = num_readings // 2
        front_ranges = ranges[front_center - front_width:front_center + front_width]

        left_ranges = ranges[:num_readings // 3]
        right_ranges = ranges[2 * num_readings // 3:]

        # Calculate sector statistics
        front_min = np.min(front_ranges) if len(front_ranges) > 0 else self.latest_scan.range_max
        left_avg = np.mean(left_ranges[~np.isinf(left_ranges)]) if len(left_ranges[~np.isinf(left_ranges)]) > 0 else self.latest_scan.range_max
        right_avg = np.mean(right_ranges[~np.isinf(right_ranges)]) if len(right_ranges[~np.isinf(right_ranges)]) > 0 else self.latest_scan.range_max

        # State machine
        if front_min < self.obstacle_threshold:
            # AVOIDING: Obstacle directly ahead
            self.navigation_state = 'avoiding'
            cmd.linear.x = 0.0
            # Turn toward more open side
            if left_avg > right_avg:
                cmd.angular.z = 0.8  # Turn left
                self.get_logger().info(f'AVOIDING: Turning LEFT (front: {front_min:.2f}m)')
            else:
                cmd.angular.z = -0.8  # Turn right
                self.get_logger().info(f'AVOIDING: Turning RIGHT (front: {front_min:.2f}m)')

        elif left_avg < self.wall_follow_distance or right_avg < self.wall_follow_distance:
            # WALL_FOLLOWING: Near a wall
            self.navigation_state = 'wall_following'
            cmd = self.wall_following_behavior(ranges)

        else:
            # EXPLORING: Open space, move forward
            self.navigation_state = 'exploring'
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0

        return cmd

    def wall_following_behavior(self, ranges: np.ndarray) -> Twist:
        """
        Wall-following behavior to navigate along walls.

        Args:
            ranges: Array of LiDAR range measurements

        Returns:
            Twist: Velocity command
        """
        cmd = Twist()
        num_readings = len(ranges)

        # Get right side distances (for right-wall following)
        right_side_start = int(num_readings * 0.65)
        right_side_end = int(num_readings * 0.85)
        right_side = ranges[right_side_start:right_side_end]

        if len(right_side) > 0:
            right_side_avg = np.mean(right_side[~np.isinf(right_side)]) if len(right_side[~np.isinf(right_side)]) > 0 else self.latest_scan.range_max

            # PID-like control to maintain wall distance
            error = right_side_avg - self.wall_follow_distance
            kp = 1.0  # Proportional gain

            cmd.linear.x = 0.25
            cmd.angular.z = -kp * error  # Negative because right-hand rule

            if abs(error) > 0.1:
                self.get_logger().info(f'WALL_FOLLOW: distance={right_side_avg:.2f}m, correction={cmd.angular.z:.2f}')

        return cmd


def main(args=None):
    """
    Main entry point for the navigation controller node.

    Args:
        args: Command line arguments (default: None)
    """
    rclpy.init(args=args)

    try:
        node = NavigationControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot
        cmd = Twist()
        node.cmd_vel_pub.publish(cmd)
        node.get_logger().info('Shutting down Navigation Controller Node')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
