import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )

    # Configuration paths
    pkg_share = get_package_share_directory('neupan_ros2')
    robot_config_dir = os.path.join(pkg_share, 'config', 'robots', 'scout')
    robot_config = os.path.join(robot_config_dir, 'robot.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'neupan_sim.rviz')

    # Static TF publisher for Livox (Scout-specific)
    base_link_to_livox_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.15', '0', '0.0', '0', '0', '0', 'livox_frame', 'base_link'],
        name='static_tf_livox_to_base_link'
    )

    # Livox pointcloud to laserscan converter (Scout-specific)
    livox_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='livox_custom_msg_to_laserscan_node',
        remappings=[
            ('cloud_in', '/livox/lidar'),
            ('scan', '/scan')
        ],
        parameters=[{
            'target_frame': 'livox_frame',
            'transform_tolerance': 0.01,
            'min_height': -0.2,
            'max_height': 1.0,
            'angle_min': -1.5708,  # -M_PI/2
            'angle_max': 1.5708,   # M_PI/2
            'angle_increment': 0.00435,  # M_PI/360.0
            'scan_time': 0.1,
            'range_min': 0.01,
            'range_max': 10.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        name='pointcloud_to_laserscan'
    )

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
        base_link_to_livox_tf,
        livox_to_scan,
        neupan_node,
        rviz_node
    ])