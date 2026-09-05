# ROS 2 接口说明

本文说明活动包 `neupan_ros` 的公开 ROS 接口。所有话题名均为相对名称，可通过 namespace
和 remap 接入实际机器人。

## `neupan_node`

### 输入

| 话题 | 消息类型 | QoS | 说明 |
| --- | --- | --- | --- |
| `scan` | `sensor_msgs/msg/LaserScan` | SensorDataQoS | 静态速度障碍输入 |
| `obstacles` | `sensor_msgs/msg/PointCloud2` | SensorDataQoS | XYZ、XYZI 或 XYZIV 障碍输入 |
| `initial_path` | `nav_msgs/msg/Path` | reliable, depth 10 | 连续参考路径，至少两个 pose |
| `neupan_waypoints` | `nav_msgs/msg/Path` | reliable, depth 10 | 稀疏 waypoint 输入 |
| `neupan_goal` | `geometry_msgs/msg/PoseStamped` | reliable, depth 10 | 直接目标输入 |
| `/tf`、`/tf_static` | TF2 | ROS 默认 | 机器人、传感器及参考几何的坐标变换 |

`initial_path`、`neupan_waypoints` 和 `neupan_goal` 是互斥的参考来源；最后收到的有效来源
生效。它们可以位于任意非空固定 frame。节点保存原始消息，并按最新 TF 持续投影到
`planning_frame`。

PointCloud2 的唯一 XYZIV 定义是
`x/y/z/intensity/vx/vy`。位置与速度都在消息 `header.frame_id` 中表达；位置进行旋转和平移，
速度只进行旋转。详细字段和时间约定见[动态障碍速度接口](dynamic_obstacles_CN.md)。

### 输出

| 话题 | 消息类型 | 坐标系/语义 |
| --- | --- | --- |
| `neupan_cmd_vel` | `geometry_msgs/msg/Twist` | `linear.x` 沿 `base_frame` +x，`angular.z` 绕 +z |
| `neupan_plan` | `nav_msgs/msg/Path` | NRMP 优化轨迹，位于 `planning_frame` |
| `neupan_ref_state` | `nav_msgs/msg/Path` | 当前参考状态，位于 `planning_frame` |
| `neupan_initial_path` | `nav_msgs/msg/Path` | 实际送入 core 的参考路径，位于 `planning_frame` |
| `neupan_arrive` | `std_msgs/msg/Bool` | 是否到达当前参考终点 |
| `neupan_diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 求解、障碍输入、frame 和停滞状态 |
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
| `obstacle_source` | `auto` | `scan`、`pointcloud` 或 `auto` |
| `scan_timeout` | `0.5` | LaserScan 新鲜度上限，s |
| `pointcloud_timeout` | `0.5` | PointCloud2 新鲜度上限，s |
| `compensate_obstacle_latency` | `true` | 用速度把障碍位置外推到当前时刻 |
| `scan_angle_min/max` | `-3.14 / 3.14` | LaserScan 角度过滤范围，rad |
| `scan_range_min/max` | `0.0 / 5.0` | LaserScan 距离过滤范围，m |
| `scan_downsample` | `1` | LaserScan 下采样步长，必须至少为 1 |
| `flip_angle` | `false` | 反转 LaserScan 波束方向 |
| `include_initial_path_direction` | `false` | 使用输入 quaternion；否则从相邻 XY 计算朝向 |
| `solver_fail_grace` | `5` | 连续求解失败多少周期后强制零速度 |
| `stall_speed` | `0.02` | 判定无运动的线/角速度阈值 |
| `stall_timeout` | `3.0` | 无进展诊断超时，s |
| `marker_size` | `0.05` | DUNE/NRMP 点 marker 尺寸，m |
| `marker_z` | `1.0` | 机器人 marker 高度，m |

`planning_frame` 取代了旧 `map_frame` 参数，不提供兼容别名。探索和 SLAM 通常使用
`odom`；固定世界仿真可以显式使用 `map`。完整规则见[坐标系接口契约](coordinate_frames_CN.md)。

## `astar_global_node`

这个节点只用于示例和最简验证，不是生产 costmap/global planner。

| 方向 | 话题 | 消息类型 | 说明 |
| --- | --- | --- | --- |
| 输入 | `map` | `nav_msgs/msg/OccupancyGrid` | transient-local reliable；地图 frame 取自消息 header |
| 输入 | `goal_pose` | `geometry_msgs/msg/PoseStamped` | 可来自任意非空固定 frame |
| 输出 | `initial_path` | `nav_msgs/msg/Path` | 位于 OccupancyGrid 的 frame |

参数包括 `base_frame=base_link`、`robot_radius=0.45`、`allow_unknown=false`、
`simplify_tolerance=0.15` 和 `goal_tolerance=0.3`。节点通过 TF 将机器人状态和目标转换到
当前地图 frame；地图原点允许包含二维旋转。

## 启动

真实机器人默认使用连续 `odom`：

```bash
ros2 launch neupan_ros neupan.launch.py planning_frame:=odom
```

可视化仿真使用固定 `map`，launch 已显式配置：

```bash
ros2 launch neupan_sim quick_start.launch.py
```

仿真时也可以验证跨 frame 路径：只要 TF 树提供 `odom` 与 `map` 的关系，即可覆盖为：

```bash
ros2 launch neupan_sim quick_start.launch.py planning_frame:=odom
```
