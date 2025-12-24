import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch parameters
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )

    # Configuration paths
    pkg_share = get_package_share_directory('neupan_ros2')
    robot_config_dir = os.path.join(pkg_share, 'config', 'robots', 'limo')
    robot_config = os.path.join(robot_config_dir, 'robot.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'neupan_sim.rviz')

    # NeuPAN core node
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
            ('/neupan_cmd_vel', '/cmd_vel'),
        ]
    )

    # RViz node (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        use_rviz_arg,
        neupan_node,
        rviz_node
    ])
