"""NeuPAN Complete Simulation System Launch File.

Launches the complete simulation environment including:
  - ddr_minimal_sim simulator (robot dynamics, environment, laser simulation)
  - NeuPAN planner node
  - RViz2 visualization (optional)

This integrates the external simulator with NeuPAN. For standalone NeuPAN testing
without the full simulator, use: ros2 launch neupan_ros2 simulation.launch.py

Usage:
  ros2 launch neupan_ros2 sim_complete.launch.py
  ros2 launch neupan_ros2 sim_complete.launch.py sim_env_config:=scenario_maze.yaml
  ros2 launch neupan_ros2 sim_complete.launch.py sim_env_config:=scenario_obstacles.yaml use_rviz:=true
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

logger = get_logger('sim_complete_launch')


def generate_launch_description() -> LaunchDescription:
    # ========== Launch Arguments ==========

    sim_env_config_arg = DeclareLaunchArgument(
        'sim_env_config',
        default_value='scenario_maze.yaml',
        description='Simulation environment configuration file (in ddr_minimal_sim/config/)'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )

    # ========== Configuration Files ==========

    # Use new robot-based configuration structure
    pkg_share = get_package_share_directory('neupan_ros2')
    robot_config_dir = os.path.join(pkg_share, 'config', 'robots', 'simulation')
    robot_config = os.path.join(robot_config_dir, 'robot.yaml')

    # RViz configuration
    rviz_config = os.path.join(pkg_share, 'rviz', 'neupan_sim.rviz')

    logger.info(f"Using NeuPAN config from: {robot_config_dir}")
    logger.info("Sim environment config will be passed to ddr_minimal_sim")

    # ========== Include ddr_minimal_sim complete launcher ==========

    # Include the complete simulator launch file from ddr_minimal_sim package
    # This will start: simulator_node, environment_node, laser_simulator_node, and static TF
    simulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ddr_minimal_sim'),
            '/launch/complete_sim.launch.py'
        ]),
        launch_arguments={
            'sim_env_config': LaunchConfiguration('sim_env_config'),
            'rviz': 'false'  # We'll launch RViz separately with NeuPAN config
        }.items()
    )

    # ========== NeuPAN Planner Node ==========

    neupan_node = Node(
        package='neupan_ros2',
        executable='neupan_node',
        name='neupan_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            robot_config,
            {'robot_config_dir': robot_config_dir}
        ],
        remappings=[
            ('/neupan_cmd_vel', '/cmd_vel')  # NeuPAN output to simulator input
        ]
    )

    # ========== Visualization ==========

    # Validate RViz configuration file existence
    if not os.path.exists(rviz_config):
        logger.warning(f"RViz config not found: {rviz_config}")
        logger.info("RViz will launch with default configuration")
        rviz_args = []
    else:
        logger.info(f"Using RViz config: {rviz_config}")
        rviz_args = ['-d', rviz_config]

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=rviz_args,
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    # ========== Launch Description ==========

    return LaunchDescription([
        # Arguments
        sim_env_config_arg,
        use_rviz_arg,

        # Include complete simulator (all simulator nodes + static TF)
        simulator_launch,

        # NeuPAN planner
        neupan_node,

        # Visualization
        rviz_node,
    ])

