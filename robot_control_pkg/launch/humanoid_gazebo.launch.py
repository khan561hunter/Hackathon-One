#!/usr/bin/env python3
"""
Launch file to spawn the humanoid robot in Gazebo simulation.

This launch file:
1. Starts Gazebo with an empty world
2. Spawns the humanoid robot from the URDF file
3. Publishes robot state information
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('robot_control_pkg')

    # Path to the URDF file
    urdf_file = os.path.join(pkg_dir, 'urdf', 'humanoid.urdf')

    # Read the URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.5')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='X position of the robot in Gazebo'
    )

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Y position of the robot in Gazebo'
    )

    declare_z_position_cmd = DeclareLaunchArgument(
        'z_pose',
        default_value='0.5',
        description='Z position of the robot in Gazebo'
    )

    # Start Gazebo server
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Start Gazebo client
    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # Spawn the robot in Gazebo
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'humanoid',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ],
        output='screen'
    )

    # Robot State Publisher node
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )

    # Path to controller configuration
    controller_config_file = os.path.join(pkg_dir, 'config', 'humanoid_controllers.yaml')

    # Controller Manager (spawned by Gazebo plugin, configured here)
    controller_manager_cmd = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_desc},
            controller_config_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Spawn Joint State Broadcaster
    spawn_joint_state_broadcaster_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # Spawn Joint Group Controller
    spawn_joint_group_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_group_controller'],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_z_position_cmd)

    # Add nodes and processes
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(controller_manager_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(spawn_joint_state_broadcaster_cmd)
    ld.add_action(spawn_joint_group_controller_cmd)

    return ld
