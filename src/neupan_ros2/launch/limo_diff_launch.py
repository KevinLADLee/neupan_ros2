import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("neupan_ros2"), "rviz", "neupan.rviz"]
    )

    config = os.path.join(
        get_package_share_directory('neupan_ros2'),
        'config',
        'limo_diff.yaml'
    )

    print(f"Using configuration file: {config}")

    # return LaunchDescription([
    #     Node(
    #         package='neupan_ros2',
    #         executable='neupan_node',
    #         name='neupan_node',
    #         output='screen',
    #         emulate_tty=True,
    #         parameters=[config],
    #         remappings=[
    #             ('/neupan_cmd_vel', '/cmd_vel')
    #         ]
    #     )
    # ])

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package='neupan_ros2',
                executable='neupan_node',
                name='neupan_node',
                output="screen",
                emulate_tty=True,
                parameters=[config],
                remappings=[
                    ('/neupan_cmd_vel', '/cmd_vel')
                ]
            ),
            launch_ros.actions.Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_cfg.perform(launch.LaunchContext())],
            ),
        ]
    )


