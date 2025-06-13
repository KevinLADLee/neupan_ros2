import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('neupan_ros2'),
        'config',
        'limo_diff.yaml'
    )

    print(f"Using configuration file: {config}")

    return LaunchDescription([
        Node(
            package='neupan_ros2',
            executable='neupan_node',
            name='neupan_node',
            output='screen',
            emulate_tty=True,
            parameters=[config],
        )
    ])

