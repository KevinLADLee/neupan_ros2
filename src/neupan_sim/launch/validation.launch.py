import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def start(context):
    command = [os.path.join(get_package_prefix('neupan_sim'), 'lib', 'neupan_sim', 'neupan_validate'),
               '--suite', LaunchConfiguration('suite').perform(context),
               '--output', LaunchConfiguration('output').perform(context)]
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        command.append('--rviz')
    return [ExecuteProcess(cmd=command, output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('suite', default_value=os.path.join(
            get_package_share_directory('neupan_sim'), 'config', 'validation.yaml')),
        DeclareLaunchArgument('output', description='New directory for resolved configs, logs, JSON and CSV'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=start),
    ])
