#!/usr/bin/env python3
"""
Sensor Data Processor Node

This ROS 2 node processes sensor data from LiDAR and depth camera to detect obstacles
and provide environmental awareness for navigation.

Usage:
    ros2 run robot_control_pkg sensor_data_processor_node

Topics:
    Subscribers:
        /humanoid/scan (sensor_msgs/msg/LaserScan): LiDAR scan data
        /humanoid/camera/depth/image_raw (sensor_msgs/msg/Image): Depth camera data

    Publishers:
        /obstacle_detection (std_msgs/msg/String): Obstacle detection results
        /navigation_hints (geometry_msgs/msg/Twist): Suggested velocity commands
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import numpy as np
import math


class SensorDataProcessorNode(Node):
    """
    ROS 2 node for processing sensor data and detecting obstacles.

    Processes LiDAR and depth camera data to identify obstacles and
    provide navigation hints for collision avoidance.
    """

    def __init__(self):
        super().__init__('sensor_data_processor_node')

        # Subscribe to LiDAR
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/humanoid/scan',
            self.lidar_callback,
            10
        )

        # Subscribe to Depth Camera
        self.depth_subscription = self.create_subscription(
            Image,
            '/humanoid/camera/depth/image_raw',
            self.depth_callback,
            10
        )

        # Publishers
        self.obstacle_pub = self.create_publisher(String, '/obstacle_detection', 10)
        self.nav_hints_pub = self.create_publisher(Twist, '/navigation_hints', 10)

        # Obstacle detection parameters
        self.obstacle_threshold = 0.5  # meters
        self.safe_distance = 1.0  # meters

        # State
        self.latest_lidar_data = None
        self.obstacles_detected = False

        self.get_logger().info('Sensor Data Processor Node initialized')
        self.get_logger().info('Listening to /humanoid/scan and /humanoid/camera/depth/image_raw')

    def lidar_callback(self, msg: LaserScan):
        """
        Process LiDAR scan data to detect obstacles.

        Args:
            msg: LaserScan message containing range measurements
        """
        self.latest_lidar_data = msg

        # Analyze scan data for obstacles
        ranges = np.array(msg.ranges)
        ranges = np.where(np.isinf(ranges), msg.range_max, ranges)  # Replace inf with max range

        # Find minimum distance in front sectors (front 60 degrees)
        num_readings = len(ranges)
        front_sector = 30  # degrees on each side
        front_indices = list(range(0, int(num_readings * front_sector / 180))) + \
                       list(range(int(num_readings * (180 - front_sector) / 180), num_readings))

        if front_indices:
            front_ranges = ranges[front_indices]
            min_front_distance = np.min(front_ranges)

            # Detect obstacles
            if min_front_distance < self.obstacle_threshold:
                self.obstacles_detected = True
                msg_out = String()
                msg_out.data = f'OBSTACLE DETECTED: {min_front_distance:.2f}m ahead (CRITICAL)'
                self.obstacle_pub.publish(msg_out)
                self.get_logger().warn(f'Obstacle detected at {min_front_distance:.2f}m!')
            elif min_front_distance < self.safe_distance:
                self.obstacles_detected = True
                msg_out = String()
                msg_out.data = f'OBSTACLE WARNING: {min_front_distance:.2f}m ahead'
                self.obstacle_pub.publish(msg_out)
            else:
                if self.obstacles_detected:
                    msg_out = String()
                    msg_out.data = 'Path clear'
                    self.obstacle_pub.publish(msg_out)
                self.obstacles_detected = False

            # Generate navigation hints based on obstacle positions
            self.generate_navigation_hints(ranges, msg)

    def generate_navigation_hints(self, ranges: np.ndarray, scan_msg: LaserScan):
        """
        Generate navigation velocity hints to avoid obstacles.

        Args:
            ranges: Array of range measurements
            scan_msg: Original LaserScan message for angle information
        """
        # Divide scan into left, front, and right sectors
        num_readings = len(ranges)
        sector_size = num_readings // 3

        left_sector = ranges[:sector_size]
        front_sector = ranges[sector_size:2*sector_size]
        right_sector = ranges[2*sector_size:]

        # Calculate average distances
        left_dist = np.mean(left_sector[~np.isinf(left_sector)]) if len(left_sector[~np.isinf(left_sector)]) > 0 else scan_msg.range_max
        front_dist = np.mean(front_sector[~np.isinf(front_sector)]) if len(front_sector[~np.isinf(front_sector)]) > 0 else scan_msg.range_max
        right_dist = np.mean(right_sector[~np.isinf(right_sector)]) if len(right_sector[~np.isinf(right_sector)]) > 0 else scan_msg.range_max

        # Generate velocity commands
        twist = Twist()

        if front_dist < self.safe_distance:
            # Obstacle in front, turn toward more open space
            if left_dist > right_dist:
                twist.angular.z = 0.5  # Turn left
                twist.linear.x = 0.1   # Slow forward
                self.get_logger().info('Hint: Turn LEFT (obstacle ahead)')
            else:
                twist.angular.z = -0.5  # Turn right
                twist.linear.x = 0.1    # Slow forward
                self.get_logger().info('Hint: Turn RIGHT (obstacle ahead)')
        else:
            # Path clear, move forward
            twist.linear.x = 0.3
            twist.angular.z = 0.0

        self.nav_hints_pub.publish(twist)

    def depth_callback(self, msg: Image):
        """
        Process depth camera data for obstacle detection.

        Args:
            msg: Image message containing depth data
        """
        # For this demo, we primarily use LiDAR
        # In a full implementation, depth camera data would complement LiDAR
        # for detecting obstacles at different heights

        # Log occasionally to show it's receiving data
        if self.get_clock().now().nanoseconds % 5000000000 < 50000000:  # ~Every 5 seconds
            self.get_logger().info(
                f'Depth camera data received: {msg.width}x{msg.height}, encoding: {msg.encoding}'
            )


def main(args=None):
    """
    Main entry point for the sensor data processor node.

    Args:
        args: Command line arguments (default: None)
    """
    rclpy.init(args=args)

    try:
        node = SensorDataProcessorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down Sensor Data Processor Node')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
