"""
NeuPAN Simulator Integration Launch File
Launch complete simulation system: ddr_minimal_sim simulator + NeuPAN planner

Usage:
  ros2 launch neupan_ros2 sim_diff_launch.py sim_env_config:=sim_env_obs.yaml
  ros2 launch neupan_ros2 sim_diff_launch.py sim_env_config:=sim_env_obs_exam.yaml use_rviz:=true
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

logger = get_logger('sim_diff_launch')


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

    # NeuPAN simulation configuration
    neupan_config = os.path.join(
        get_package_share_directory('neupan_ros2'),
        'config',
        'sim_diff.yaml'
    )

    # RViz configuration
    rviz_config = os.path.join(
        get_package_share_directory('neupan_ros2'),
        'rviz',
        'neupan_sim.rviz'
    )

    logger.info(f"Using NeuPAN config: {neupan_config}")
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

    # ========== NeuPAN规划器节点 ==========

    neupan_node = Node(
        package='neupan_ros2',
        executable='neupan_node',
        name='neupan_node',
        output='screen',
        emulate_tty=True,
        parameters=[neupan_config, {'use_sim_time': True}],
        remappings=[
            ('/neupan_cmd_vel', '/cmd_vel')  # neupan输出连接到仿真器输入
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
