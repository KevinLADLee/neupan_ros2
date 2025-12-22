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
    robot_config_dir = os.path.join(pkg_share, 'config', 'robots', 'ranger')
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

    # Auxiliary nodes (Ranger-specific)
    # TF publishers for LiDAR frames
    laser_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="hesai_lidar_transform_publisher",
        arguments=['--x', '0.0', '--y', '-0.0', '--z', '0.3',
                   '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'base_laser']
    )

    laser_to_hesai_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="laser_to_hesai_tf",
        arguments=['--x', '0.0', '--y', '-0.0', '--z', '0.0',
                   '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'base_laser', '--child-frame-id', 'hesai_lidar']
    )

    # Pointcloud to LaserScan conversion
    cloud_to_laser_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan_node",
        output="screen",
        parameters=[{
            'target_frame': 'base_link',
            "range_min": 0.1,
            "range_max": 50.0,
            "scan_time": 0.1,
            "min_height": -0.4,
            "max_height": 0.5,
            "angle_min": -3.14159265,
            "angle_max": 3.14159265,
            "angle_increment": 0.00872664625,
            "inf_epsilon": 1.0,
            "tf_tolerance": 0.03,
        }],
        remappings=[
            ("/cloud_in", "/lidar_points"),
            ("/scan", "/scan"),
        ],
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
        laser_transform_publisher,
        laser_to_hesai_tf,
        cloud_to_laser_node,
        neupan_node,
        rviz_node
    ])
