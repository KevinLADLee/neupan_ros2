# neupan_ros 节点接口概览

本文概述 `neupan_ros` 功能包提供的节点、话题和 TF 约定。完整节点参数、消息类型、QoS
配置以及服务/动作说明以[功能包 README](../src/neupan_ros/README.md) 为准。所有话题均使用
相对名称，可以通过 ROS 2 命名空间和名称重映射部署到不同机器人。

## `neupan_node`

### 订阅的话题

| 话题 | 消息类型 | QoS | 说明 |
| --- | --- | --- | --- |
| `scan` | `sensor_msgs/msg/LaserScan` | `SensorDataQoS` | LaserScan 障碍物观测 |
| `obstacles` | `sensor_msgs/msg/PointCloud2` | `SensorDataQoS` | XYZ、XYZI 或 XYZIV 障碍物点云 |
| `initial_path` | `nav_msgs/msg/Path` | Reliable / Volatile / Keep Last (10) | 连续参考路径，至少包含两个位姿 |
| `neupan_waypoints` | `nav_msgs/msg/Path` | Reliable / Volatile / Keep Last (10) | 稀疏路径点 |
| `neupan_goal` | `geometry_msgs/msg/PoseStamped` | Reliable / Volatile / Keep Last (10) | 直接目标位姿 |
| `/tf`、`/tf_static` | TF2 | ROS 默认 | 机器人、传感器及参考几何的坐标变换 |

`initial_path`、`neupan_waypoints` 和 `neupan_goal` 是互斥的参考来源；最后收到的有效来源
生效。它们可以位于任意非空固定坐标系。节点保存原始消息，并按最新 TF 持续转换到
`planning_frame`。

PointCloud2 的唯一 XYZIV 定义是
`x/y/z/intensity/vx/vy`。位置与速度都在消息 `header.frame_id` 中表达；位置进行旋转和平移，
速度只进行旋转。详细字段和时间约定见[动态障碍物消息约定](dynamic_obstacles_CN.md)。

### 发布的话题

| 话题 | 消息类型 | 坐标系/语义 |
| --- | --- | --- |
| `neupan_cmd_vel` | `geometry_msgs/msg/Twist` | `linear.x` 沿 `base_frame` +x，`angular.z` 绕 +z |
| `neupan_plan` | `nav_msgs/msg/Path` | NRMP 优化轨迹，位于 `planning_frame` |
| `neupan_ref_state` | `nav_msgs/msg/Path` | 当前参考状态，位于 `planning_frame` |
| `neupan_initial_path` | `nav_msgs/msg/Path` | 传给规划库的参考路径，位于 `planning_frame` |
| `neupan_arrive` | `std_msgs/msg/Bool` | 是否到达当前参考终点 |
| `neupan_diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 求解器、障碍物输入、坐标系和停滞状态 |
| `dune_point_markers` | `visualization_msgs/msg/MarkerArray` | DUNE 点，可视化使用 `planning_frame` |
| `nrmp_point_markers` | `visualization_msgs/msg/MarkerArray` | NRMP 点，可视化使用 `planning_frame` |
| `robot_marker` | `visualization_msgs/msg/Marker` | 机器人轮廓，可视化使用 `planning_frame` |

### 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `config_file` | 空，必须配置 | NeuPAN planner YAML |
| `dune_checkpoint` | 空 | NPTF 模型；纯 PAN/navigation 配置可以为空 |
| `planning_frame` | `odom` | 局部优化使用的连续参考坐标系 |
| `base_frame` | `base_link` | 机器人本体坐标系 |
| `control_rate` | `50.0` | 控制频率，Hz |
| `pose_timeout` | `0.5` | 机器人位姿 TF 的最大年龄，s；必须为有限正数 |
| `obstacle_source` | `auto` | `scan`、`pointcloud` 或 `auto` |
| `scan_timeout` | `0.5` | LaserScan 新鲜度上限，s |
| `pointcloud_timeout` | `0.5` | PointCloud2 新鲜度上限，s |
| `compensate_obstacle_latency` | `true` | 用速度把障碍位置外推到当前时刻 |
| `scan_angle_min/max` | `-3.14 / 3.14` | LaserScan 角度过滤范围，rad |
| `scan_range_min/max` | `0.0 / 5.0` | LaserScan 距离过滤范围，m |
| `scan_downsample` | `1` | LaserScan 下采样步长，必须至少为 1 |
| `flip_angle` | `false` | 反转 LaserScan 波束方向 |
| `include_initial_path_direction` | `false` | 使用输入四元数；否则根据相邻 XY 坐标计算朝向 |
| `solver_fail_grace` | `5` | 连续求解失败多少周期后强制零速度 |
| `stall_speed` | `0.02` | 判定无运动的线/角速度阈值 |
| `stall_timeout` | `3.0` | 无进展诊断超时，s |
| `marker_size` | `0.05` | DUNE/NRMP 点 marker 尺寸，m |
| `marker_z` | `1.0` | 机器人 marker 高度，m |

`planning_frame` 取代了旧 `map_frame` 参数，不提供兼容别名。探索和 SLAM 通常使用
`odom`；固定世界仿真可以显式使用 `map`。完整规则见[坐标系接口契约](coordinate_frames_CN.md)。

机器人变换 `planning_frame <- base_frame` 的时间戳超过 `pose_timeout`，或领先当前时间
超过 50 ms 时，每个控制周期都发布零速度和 WARN 诊断；新鲜 TF 恢复后自动继续规划。
年龄按节点 ROS 时钟计算（启用仿真时间时使用仿真时钟）。参考路径坐标系的静态变换不受
此超时约束。可通过 `ros2 launch neupan_ros neupan.launch.py pose_timeout:=0.3` 调整阈值。

## `astar_global_node`

这个节点仅用于示例和集成验证，不是 Nav2 global planner plugin。

| 方向 | 话题 | 消息类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `map` | `nav_msgs/msg/OccupancyGrid` | Reliable / Transient Local / Keep Last (1)；坐标系取自消息头 |
| 订阅 | `goal_pose` | `geometry_msgs/msg/PoseStamped` | 可来自任意非空固定坐标系 |
| 发布 | `initial_path` | `nav_msgs/msg/Path` | 位于占用栅格地图的坐标系 |

参数包括 `base_frame=base_link`、`robot_radius=0.45`、`allow_unknown=false`、
`simplify_tolerance=0.15` 和 `goal_tolerance=0.3`。节点通过 TF 将机器人状态和目标转换到
当前地图坐标系；地图原点允许包含二维旋转。

## 启动文件

真实机器人默认使用连续 `odom`：

```bash
ros2 launch neupan_ros neupan.launch.py planning_frame:=odom
```

可视化仿真使用固定 `map`，启动文件已显式配置：

```bash
ros2 launch neupan_sim quick_start.launch.py
```

仿真时也可以验证跨坐标系路径：只要 TF 树提供 `odom` 与 `map` 的关系，即可覆盖为：

```bash
ros2 launch neupan_sim quick_start.launch.py planning_frame:=odom
```
