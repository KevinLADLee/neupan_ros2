# 分布式多机器人部署

本项目的实际机器人部署方式是：每台机器人独立运行自己的定位、传感器、全局规划、
`neupan_node` 和底盘控制器。机器人使用相同的 `ROS_DOMAIN_ID` 加入同一个 DDS 网络，
但不依赖 `neupan_fleet_sim`，也不需要一个中心进程启动所有机器人。

`robot_id` 必须同时用于 ROS namespace 和 TF frame 前缀。例如 `robot_01` 对应：

| 类型 | 名称 |
| --- | --- |
| NeuPAN 节点 | `/robot_01/neupan_node` |
| 激光 | `/robot_01/scan` |
| 完整障碍点云 | `/robot_01/obstacles` |
| 初始路径 | `/robot_01/initial_path` |
| 速度输出 | `/robot_01/cmd_vel` |
| 机器人本体 frame | `robot_01/base_link` |
| 激光 frame | `robot_01/laser_link` |

ROS namespace 会自动隔离相对话题，但不会修改消息中的 `header.frame_id`，也不会修改
TF child frame。因此，多台机器人不能同时使用没有前缀的 `base_link` 或 `laser_link`。

## 通用的单机器人 launch

建议在自己的 bringup 包中创建 `launch/neupan_robot.launch.py`。这个文件只启动一台
机器人的 NeuPAN 节点，机器人 ID、配置、模型和驱动话题都由 launch 参数决定：

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_robot(context):
    robot_id = LaunchConfiguration('robot_id').perform(context).strip('/')
    if not robot_id or '/' in robot_id:
        raise RuntimeError('robot_id must be one namespace component, such as robot_01')

    configured_base = LaunchConfiguration('base_frame').perform(context)
    base_frame = configured_base or f'{robot_id}/base_link'

    return [Node(
        package='neupan_ros',
        executable='neupan_node',
        name='neupan_node',
        namespace=robot_id,
        output='screen',
        parameters=[{
            'config_file': ParameterValue(
                LaunchConfiguration('config_file'), value_type=str),
            'dune_checkpoint': ParameterValue(
                LaunchConfiguration('dune_checkpoint'), value_type=str),
            'planning_frame': ParameterValue(
                LaunchConfiguration('planning_frame'), value_type=str),
            'base_frame': base_frame,
            'obstacle_source': ParameterValue(
                LaunchConfiguration('obstacle_source'), value_type=str),
        }],
        remappings=[
            ('scan', LaunchConfiguration('scan_topic')),
            ('obstacles', LaunchConfiguration('obstacles_topic')),
            ('initial_path', LaunchConfiguration('path_topic')),
            ('neupan_goal', LaunchConfiguration('goal_topic')),
            ('neupan_cmd_vel', LaunchConfiguration('cmd_vel_topic')),
        ],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id'),
        DeclareLaunchArgument('config_file'),
        DeclareLaunchArgument('dune_checkpoint'),
        DeclareLaunchArgument('planning_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value=''),
        DeclareLaunchArgument('obstacle_source', default_value='auto'),
        DeclareLaunchArgument('scan_topic', default_value='scan'),
        DeclareLaunchArgument('obstacles_topic', default_value='obstacles'),
        DeclareLaunchArgument('path_topic', default_value='initial_path'),
        DeclareLaunchArgument('goal_topic', default_value='neupan_goal'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel'),
        OpaqueFunction(function=launch_robot),
    ])
```

默认话题名都是相对名称。例如 `scan_topic=scan` 在 `robot_01` namespace 中解析为
`/robot_01/scan`。如果某个驱动发布绝对话题，可以在该机器人的包装 launch 中将参数
改成绝对名称。

## 为每台机器人创建固定的启动文件

每台机器人再创建一个很小的包装文件，固定它的 ID、几何配置、DUNE 模型和驱动话题。
下面是 `launch/robot_01.launch.py`：

```python
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup = Path(get_package_share_directory('my_fleet_bringup'))
    neupan = Path(get_package_share_directory('neupan_ros'))

    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(bringup / 'launch/neupan_robot.launch.py')),
            launch_arguments={
                'robot_id': 'robot_01',
                'config_file': str(neupan / 'config/scout_mini_diff.yaml'),
                'dune_checkpoint': str(
                    neupan / 'models/diff_scout_mini_612x580.bin'),
                'planning_frame': 'robot_01/odom',
                'base_frame': 'robot_01/base_link',
                'scan_topic': 'scan',
                'obstacles_topic': 'obstacles',
                'path_topic': 'initial_path',
                'cmd_vel_topic': 'cmd_vel',
            }.items(),
        ),
    ])
```

`robot_02.launch.py` 使用同一个通用文件，只修改与机器人身份或硬件有关的值：

```python
launch_arguments={
    'robot_id': 'robot_02',
    'config_file': str(neupan / 'config/compact_polygon_diff.yaml'),
    'dune_checkpoint': str(neupan / 'models/diff_compact_polygon.bin'),
    'planning_frame': 'robot_02/odom',
    'base_frame': 'robot_02/base_link',
    'scan_topic': 'front_lidar/scan',
    'obstacles_topic': 'perception/obstacles',
    'path_topic': 'initial_path',
    'cmd_vel_topic': 'controller/cmd_vel',
}.items()
```

然后分别在对应机器人上启动：

```bash
# robot_01 计算机
ros2 launch my_fleet_bringup robot_01.launch.py

# robot_02 计算机
ros2 launch my_fleet_bringup robot_02.launch.py
```

两个包装文件中的 `ROS_DOMAIN_ID` 必须相同。若系统服务或容器已经设置该环境变量，可以
从 launch 中删除 `SetEnvironmentVariable`，避免存在两处配置来源。多机通信还要求 DDS
发现能够通过网络、防火墙允许所用 DDS 流量，并且 `ROS_LOCALHOST_ONLY` 没有设置为 `1`。

## 每台机器人的 TF 和输入

每台机器人必须独立提供：

1. 连续且时间戳新鲜的 `planning_frame -> <robot_id>/base_link` TF；
2. 传感器 frame 到 `planning_frame` 的 TF；
3. `scan` 或完整的 `obstacles` PointCloud2；
4. `initial_path`、`neupan_waypoints` 或 `neupan_goal`；
5. 接收 `cmd_vel` 的本机底盘控制器。

如果 `robot_state_publisher` 的 URDF 仍使用 `base_link`、`laser_link`，可以为它设置
`frame_prefix: '<robot_id>/'`。如果 URDF 中已经写入机器人前缀，则不要再次设置
`frame_prefix`。

使用每台机器人自己的连续 `robot_N/odom` 作为 `planning_frame`，可以避免 SLAM 回环时
`map` 跳变影响局部规划。全局路径可以发布在 `map` 中，只要 TF 能解析
`robot_N/odom <- map`。若所有定位系统直接提供稳定的公共 `map`，也可以统一使用 `map`。

## 机器人之间如何互相避让

每个 `neupan_node` 都是独立规划器，不会因为处于相同 ROS domain 就主动订阅其他机器人
的 odometry。其他机器人必须出现在本机的障碍物输入中：

- 实际激光能看到其他机器人时，使用 `scan` 即可，但这些点没有速度信息；
- 本机感知或跟踪节点可以把其他机器人加入完整 XYZIV 点云，再发布到本机的
  `obstacles`；位置和 `vx/vy` 必须位于消息 `header.frame_id` 指定的坐标系中；
- 使用 `obstacle_source=pointcloud` 时，点云必须包含当前规划所需的完整环境障碍点，
  不能只包含其他机器人。

这种结构仍是完全分布式的：每台机器人只消费本机传感器或本机融合结果，并独立输出
速度。共享 domain 提供通信可见性，并不引入中心调度器。窄通道优先级、任务分配和路权
预约应由 NeuPAN 之上的任务或交通管理层处理。

## 几何和模型

每台机器人使用自己的 planner YAML 和与自身 footprint 匹配的 DUNE checkpoint。DUNE
模型描述受控机器人的形状，不描述它看到的其他机器人。当前精确支持矩形和单个凸多边形；
复合形状和凹多边形仍不支持。
