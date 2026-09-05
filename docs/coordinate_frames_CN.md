# 坐标系接口契约

本文是原生 NeuPAN runtime 的规范性接口说明。不满足约定的输入会被拒绝，不再通过默认
frame 猜测其含义。

## 基本约定

- 全部采用 SI 单位：位置为米、速度为米每秒、角度为弧度。
- 采用 REP-103 右手坐标系：`base_frame` 的 x 向前、y 向左、z 向上；绕 +z
  逆时针为正 yaw。
- `planning_frame` 是统一的二维局部规划坐标系，默认名称为 `odom`，但可以配置为任意
  连续参考系；它不等同于 ROS 全局 `map` frame。
- 记号 `planning_frame <- sensor_frame` 表示把 sensor frame 中的数据转换到
  `planning_frame`。
- 探索和 SLAM 场景推荐使用连续的 `odom`，避免 `map` 在回环或重定位时跳变并污染
  局部轨迹及动态障碍速度。

## `neupan_core`

core 不识别 frame 名称，也不会查询 TF。一次 `NeuPANPlanner::forward` 调用中的全部空间
数据必须位于同一个世界坐标系：

| 数据 | 维度 | 坐标系和单位 |
| --- | --- | --- |
| `state` | `3 x 1` | 世界系中的 `[x_m, y_m, yaw_rad]` |
| `points` | `2 x N` | 世界系中的障碍点 `[x_m, y_m]` |
| `velocities` | `2 x N` | 沿世界系轴向的 `[vx_mps, vy_mps]` |
| initial path | 逻辑上 `4 x K` | 世界系中的 `[x_m, y_m, yaw_rad, gear]` |

位置和速度按列一一对应。调用 core 的适配层负责维持该约束；例如 map-frame state 与
base-frame obstacle points 混用属于非法输入。

## `neupan_ros` 输入

| 输入 | `header.frame_id` 要求 | 处理方式 |
| --- | --- | --- |
| `scan` | 非空传感器 frame | frame 相同则直接使用，否则按 `header.stamp` 转换到 `planning_frame` |
| `obstacles` | 非空传感器/来源 frame | frame 相同则直接使用，否则按 `header.stamp` 转换位置和速度 |
| `initial_path` | 任意非空固定 frame | 保存原始路径，并按最新 TF 持续转换到 `planning_frame`；至少两个 pose |
| `neupan_waypoints` | 任意非空固定 frame | 保存原始 waypoints，并按最新 TF 持续转换到 `planning_frame` |
| `neupan_goal` | 任意非空固定 frame | 保存原始目标，并按最新 TF 持续转换到 `planning_frame` |
| A* `map` | 任意非空固定 frame | 使用 `OccupancyGrid.header.frame_id`，并支持旋转的 map origin |
| A* `goal_pose` | 任意非空固定 frame | 按最新 TF 转换到当前 OccupancyGrid frame 后规划 |

对于 `nav_msgs/Path`，以顶层 `Path.header.frame_id` 为准，各 PoseStamped 自身的 header
不参与判断。默认从相邻 XY 点计算路径朝向；仅当
`include_initial_path_direction=true` 时才采用输入 quaternion。

传感器消息的零时间戳表示节点当前时间；非零时间戳必须存在对应的历史 TF。查询不到 TF
时丢弃该帧。若传感器 frame 已等于 `planning_frame`，跳过 TF 查询和矩阵变换。

路径、waypoint 和目标不是瞬时测量，而是持续存在的参考几何。节点保留其原始 frame
数据，并在控制周期使用最新 TF 更新到 `planning_frame`。因此 `map -> odom` 在 SLAM
期间发生变化时，不会继续使用旧的一次性转换结果。所有输入的空 `frame_id` 一律拒绝。

PointCloud2 XYZIV 固定为 `x/y/z/intensity/vx/vy`。位置和速度都位于消息
`header.frame_id`：适配层对 XY 位置执行旋转和平移，对速度只执行旋转，随后以
`planning_frame` 数据调用 core。Z 只检查是否为有限值，不参与二维规划。

机器人状态来自最新的 `planning_frame <- base_frame` TF。

## 输出

- `neupan_cmd_vel` 没有 header：`linear.x` 是沿 `base_frame` +x 的 m/s，
  `angular.z` 是绕 +z 逆时针的 rad/s。
- `neupan_plan`、`neupan_ref_state`、`neupan_initial_path` 以及规划可视化均使用
  `planning_frame`。
- 仿真器发布 `map -> base_link -> laser_link`；`scan` 与 `obstacles` 使用
  `laser_link`，`initial_path` 使用 `map`。仿真 launch 显式设置
  `planning_frame=map`，也可以覆盖该 launch 参数。
- 仿真 YAML 中的位姿、路径点、线段和障碍速度全部使用仿真的 `map` 坐标系。

## 必需的 TF 关系

```text
planning_frame
  └── base_frame
        └── scan/obstacles 的 header.frame_id
```

雷达不要求一定是 `base_frame` 的直接子 frame，只要 TF 能在传感器时间戳解析完整变换
即可。全局 `map` 也不要求是 `planning_frame` 的父 frame，只要两者可通过 TF 连通。
