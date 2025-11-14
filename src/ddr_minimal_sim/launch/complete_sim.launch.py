#!/usr/bin/env python3

"""
Complete Simulator Launch File

This launch file starts all components of the minimal differential drive simulator:
  - Simulator node (vehicle kinematics)
  - Environment node (obstacles and map)
  - Laser simulator node (lidar simulation)
  - Static TF publisher (map -> odom)
  - RViz (optional)

Usage:
  ros2 launch ddr_minimal_sim complete_sim.launch.py sim_env_config:=sim_env_obs.yaml
  ros2 launch ddr_minimal_sim complete_sim.launch.py sim_env_config:=sim_env_obs_exam.yaml rviz:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    sim_env_config_arg = DeclareLaunchArgument(
        'sim_env_config',
        default_value='sim_env_obs.yaml',
        description='Simulation environment configuration file name (in config/ directory)'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )

    # Create launch description
    ld = LaunchDescription([
        sim_env_config_arg,
        rviz_arg,
    ])

    # Add nodes via opaque function to resolve substitutions
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld


def launch_setup(context, *args, **kwargs):  # noqa: ARG001
    # Get launch configuration values
    sim_env_config = LaunchConfiguration('sim_env_config').perform(context)
    rviz_enabled = LaunchConfiguration('rviz').perform(context)

    # Get package directories
    pkg_share = get_package_share_directory('ddr_minimal_sim')
    config_file = os.path.join(pkg_share, 'config', sim_env_config)

    # Verify config file exists
    if not os.path.exists(config_file):
        raise FileNotFoundError(f"Configuration file not found: {config_file}")

    print(f"[complete_sim.launch.py] Using configuration file: {config_file}")

    nodes_to_launch = []

    # Static TF: map -> odom (identity transform)
    # This allows the simulator to work with navigation stacks that expect a map frame
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
        output='log'
    )
    nodes_to_launch.append(static_tf_map_odom)

    # Simulator Node (this publishes /clock, so it doesn't use sim_time)
    simulator_node = Node(
        package='ddr_minimal_sim',
        executable='simulator_node',
        name='simulator',
        output='screen',
        parameters=[config_file, {'use_sim_time': False}],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/odom', '/odom'),
        ]
    )
    nodes_to_launch.append(simulator_node)

    # Environment Node
    environment_node = Node(
        package='ddr_minimal_sim',
        executable='environment_node',
        name='environment',
        output='screen',
        parameters=[config_file, {'use_sim_time': True}]
    )
    nodes_to_launch.append(environment_node)

    # Laser Simulator Node
    laser_simulator_node = Node(
        package='ddr_minimal_sim',
        executable='laser_simulator_node',
        name='laser_simulator',
        output='screen',
        parameters=[config_file, {'use_sim_time': True}],
        remappings=[
            ('/scan', '/scan'),
        ]
    )
    nodes_to_launch.append(laser_simulator_node)

    # RViz (optional)
    if rviz_enabled.lower() == 'true':
        rviz_config_file = os.path.join(pkg_share, 'rviz', 'simulator.rviz')

        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else []
        )
        nodes_to_launch.append(rviz_node)

    return nodes_to_launch
