import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory('neupan_sim')
    neupan_share = get_package_share_directory('neupan_ros')

    use_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start the prepared NeuPAN RViz view.',
    )
    planning_frame = DeclareLaunchArgument(
        'planning_frame',
        default_value='map',
        description='Reference frame used by local NeuPAN planning.',
    )

    simulator = Node(
        package='neupan_sim',
        executable='neupan_sim_node',
        name='neupan_sim',
        output='screen',
        parameters=[os.path.join(sim_share, 'config', 'quick_start.yaml')],
    )

    planner = Node(
        package='neupan_ros',
        executable='neupan_node',
        name='neupan_node',
        output='screen',
        parameters=[{
            'config_file': os.path.join(
                neupan_share, 'config', 'sentry_diff.yaml'),
            'dune_checkpoint': os.path.join(
                neupan_share, 'models', 'diff_sentry.bin'),
            'control_rate': 50.0,
            'planning_frame': LaunchConfiguration('planning_frame'),
            'base_frame': 'base_link',
            'obstacle_source': 'auto',
            'pointcloud_timeout': 0.5,
            'compensate_obstacle_latency': True,
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(sim_share, 'rviz', 'neupan_sim.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(
        [use_rviz, planning_frame, simulator, planner, rviz])
