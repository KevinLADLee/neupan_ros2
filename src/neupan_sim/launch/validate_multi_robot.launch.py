"""One shared simulation, heterogeneous NeuPAN robots, obstacles and RViz."""
from pathlib import Path
import runpy
import tempfile

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_multi_robot(context):
    # Reuse the same geometry/model resolver as headless validation.
    helper = Path(get_package_prefix('neupan_sim')) / 'lib/neupan_sim/validation_config.py'
    resolve = runpy.run_path(str(helper))['load_shared_cases']
    cases = resolve(LaunchConfiguration('config').perform(context), get_package_share_directory,
                    LaunchConfiguration('scenario').perform(context))
    directory = tempfile.TemporaryDirectory(prefix='neupan-multi-robot-')
    root = Path(directory.name)
    files, planners, displays = [], [], []
    colors = ['255; 100; 80', '70; 170; 255', '255; 210; 60']
    for i, case in enumerate(cases):
        name = case['name']
        planner_file = root / f'{name}.planner.yaml'
        simulator_file = root / f'{name}.sim.yaml'
        planner_file.write_text(yaml.safe_dump(case['planner']))
        simulator_file.write_text(yaml.safe_dump({'/**': {'ros__parameters': case['simulator']}}))
        files.append(str(simulator_file))
        planners.append(Node(
            package='neupan_ros', executable='neupan_node', namespace=name, output='screen',
            parameters=[{'config_file': str(planner_file), 'dune_checkpoint': case['checkpoint'],
                         'planning_frame': case['simulator']['world_frame'],
                         'base_frame': case['simulator']['base_frame'],
                         'control_rate': 20.0, 'obstacle_source': 'pointcloud',
                         'pose_timeout': 1.0, 'pointcloud_timeout': 1.0}]))
        displays.append({'Class': 'rviz_common/Group', 'Name': name, 'Enabled': True, 'Displays': [
            {'Class': 'rviz_default_plugins/MarkerArray', 'Name': 'World and robot', 'Enabled': True,
             'Topic': {'Value': f'/{name}/neupan_sim/markers', 'Depth': 5,
                       'Durability Policy': 'Transient Local', 'Reliability Policy': 'Reliable'}},
            {'Class': 'rviz_default_plugins/Path', 'Name': 'Plan', 'Enabled': True,
             'Color': colors[i % len(colors)], 'Line Style': 'Lines', 'Line Width': 0.04,
             'Topic': {'Value': f'/{name}/neupan_plan', 'Depth': 5}},
            {'Class': 'rviz_default_plugins/LaserScan', 'Name': 'Lidar (includes peers)', 'Enabled': True,
             'Size (m)': 0.025, 'Style': 'Points', 'Color Transformer': 'FlatColor',
             'Color': colors[i % len(colors)],
             'Topic': {'Value': f'/{name}/scan', 'Depth': 5, 'Reliability Policy': 'Reliable'}}]} )
    rviz_file = root / 'multi_robot.rviz'
    rviz_file.write_text(yaml.safe_dump({'Visualization Manager': {
        'Global Options': {'Fixed Frame': cases[0]['simulator']['world_frame'],
                           'Background Color': '35; 35; 40'}, 'Displays': displays,
        'Views': {'Current': {'Class': 'rviz_default_plugins/TopDownOrtho',
                              'Scale': 45, 'X': 0, 'Y': 0}}}}))
    fleet = Node(package='neupan_sim', executable='neupan_fleet_sim', output='screen',
                 parameters=[{'robot_names': [c['name'] for c in cases], 'simulator_files': files}])
    starter = Node(package='neupan_sim', executable='neupan_start_fleet', output='screen',
                   parameters=[{'robot_names': [c['name'] for c in cases], 'startup_timeout': 40.0}])

    def cleanup(_context):
        directory.cleanup()
        return []

    def startup_exit(event, _context):
        if event.returncode:
            return [EmitEvent(event=Shutdown(reason='Multi-robot startup failed; see startup log'))]
        return []

    actions = [RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
               RegisterEventHandler(OnProcessExit(target_action=starter, on_exit=startup_exit)),
               LogInfo(msg=f'Shared world: {len(cases)} robots; configuration: {root}'), fleet, *planners, starter,
               Node(package='rviz2', executable='rviz2', arguments=['-d', str(rviz_file)],
                    condition=IfCondition(LaunchConfiguration('rviz')), output='screen')]
    for process in [fleet, *planners]:
        actions.insert(0, RegisterEventHandler(OnProcessExit(target_action=process, on_exit=[
            EmitEvent(event=Shutdown(reason='A multi-robot simulation/planner process exited'))])))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(Path(get_package_share_directory('neupan_sim')) /
                                                          'config/multi_robot.yaml')),
        DeclareLaunchArgument('scenario', default_value='obstacle_world'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_multi_robot),
    ])
