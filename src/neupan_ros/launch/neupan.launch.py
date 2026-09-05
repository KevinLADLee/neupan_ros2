import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('neupan_ros')

    default_config = os.path.join(pkg_share, 'config', 'planner.yaml')
    default_model = os.path.join(pkg_share, 'models', 'diff_default.bin')
    planning_frame = LaunchConfiguration('planning_frame')
    base_frame = LaunchConfiguration('base_frame')

    neupan_node = Node(
        package='neupan_ros',
        executable='neupan_node',
        name='neupan_node',
        output='screen',
        parameters=[{
            'config_file': default_config,
            'dune_checkpoint': default_model,
            'planning_frame': planning_frame,
            'base_frame': base_frame,
            'pose_timeout': ParameterValue(
                LaunchConfiguration('pose_timeout'), value_type=float),
        }],
        remappings=[
            ('neupan_cmd_vel', 'cmd_vel'),
        ],
    )

    astar_node = Node(
        package='neupan_ros',
        executable='astar_global_node',
        name='astar_global_node',
        output='screen',
        parameters=[{'base_frame': base_frame}],
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'planning_frame', default_value='odom',
        description='Continuous reference frame used by local NeuPAN planning.'))
    ld.add_action(DeclareLaunchArgument(
        'base_frame', default_value='base_link',
        description='Robot body frame.'))
    ld.add_action(DeclareLaunchArgument(
        'pose_timeout', default_value='0.5',
        description='Maximum robot TF age in seconds before stopping.'))
    ld.add_action(neupan_node)
    ld.add_action(astar_node)

    return ld
