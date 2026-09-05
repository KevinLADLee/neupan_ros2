# 动态障碍速度接口

## `neupan_core` 输入语义

`neupan_core` 接受一一对应的世界坐标点和速度；这里的“世界坐标”是调用方选择的统一
规划坐标系，在 ROS 适配层中就是可配置的 `planning_frame`：

```cpp
planner.forward(state, obstacle_points, obstacle_velocities, info);
```

两者均为 `2 x N` 矩阵，第 `k` 列分别是
`[x_k, y_k]` 和 `[vx_k, vy_k]`。PAN 按原始 NeuPAN 的常速度模型预测：

```text
p_(t,k) = p_(0,k) + t * step_time * v_k,  t = 0..T.
```

如果点数超过 `dune_max_num`，位置和速度使用同一组
`linspace(...).astype(int)` 索引下采样，避免速度错配。空速度矩阵表示所有点静止，因而
旧的静态点云 API 保持兼容。

## PointCloud2 消息约定

`neupan_ros` 同时支持：

- `sensor_msgs/msg/LaserScan`；
- `sensor_msgs/msg/PointCloud2` 的 XYZ、XYZI、XYZIV 格式。

节点通过相对话题 `obstacles` 订阅 PointCloud2，部署时使用 ROS 2 名称重映射连接实际
传感器话题。所有格式都必须
包含标量 FLOAT32 `x/y/z`；规划只使用 `x/y`，但会过滤 `z` 非有限的无效点。`intensity`
字段可为任意类型且不会参与规划。没有速度字段的 XYZ、XYZI 自动填充零速度。

本项目对 XYZIV 采用唯一、明确的定义：

| 速度字段 | 类型 | 解释 |
| --- | --- | --- |
| `vx` + `vy` | 两个 FLOAT32 | 消息坐标系中的二维速度向量 |

即字段集合为 `x/y/z/intensity/vx/vy`，V 表示平面二维速度，不接受字段别名或标量径向
速度。若传感器只输出 Doppler 径向速度，应由上游感知或跟踪节点先估计二维速度。ROS 2
适配层对位置应用旋转和平移、对速度只应用旋转，最后转换到 `planning_frame`。若点云
已经位于 `planning_frame`，则直接使用，不查询 TF 或执行矩阵变换。探索和 SLAM 场景
通常应选择连续的 `odom`，避免全局 `map` 跳变被误认为障碍速度。
消息必须提供非空 `header.frame_id`；完整约定见
[坐标系约定](coordinate_frames_CN.md)。

消息应描述用于本帧规划的**完整障碍点集合**：静态点也要包含，且速度置零。这样
PointCloud2 可以直接替代 LaserScan，不会重复同一障碍的惩罚，也不会漏掉静态环境。
若上游只输出目标中心或只输出运动目标，应先在跟踪或融合节点中生成带速度的完整点云。

## 时间处理

- 使用消息 `header.stamp` 查询对应时刻的 TF；TF 不可用时丢弃该帧。
- 点云接收时间和观测时间都必须在 `pointcloud_timeout` 内。
- `compensate_obstacle_latency=true` 时，进入 PAN 前先做
  `p_now = p_observed + age * v`，之后再从当前时刻预测 `T` 个阶段。
- 负时间差在 50 ms 内视为时钟抖动；更大的未来时间戳不会被采用。

## `obstacle_source` 参数

ROS 参数 `obstacle_source` 支持：

- `scan`：只创建相对话题 `scan` 的订阅，所有速度为零；
- `pointcloud`：只使用 PointCloud2；XYZ/XYZI 也合法，超时立即停止规划输出；
- `auto`：默认值，同时创建两个订阅。PointCloud2 新鲜时整帧使用它，否则回退到新鲜的
  `scan`。

相关参数：

```yaml
obstacle_source: auto
pointcloud_timeout: 0.5
compensate_obstacle_latency: true
```

相对诊断话题 `neupan_diagnostics` 会报告 `obstacle_source`、`obstacle_format`、`obstacle_points` 和
`max_obstacle_speed`，可用于确认部署时是否真正走动态预测链路。

## 最小仿真器

`neupan_sim` 同时发布相对话题 `scan` 和 XYZIV（`x/y/z/intensity/vx/vy`）格式的
`obstacles`。在默认根命名空间中它们分别解析为 `/scan` 与 `/obstacles`。动态圆障碍配置格式为：

```yaml
# [x, y, radius, vx, vy]，均在 map 坐标系
dynamic_circles: [3.2, -1.5, 0.3, 0.0, 0.6]
```

动态障碍在连续几何世界边界精确反射。速度点云由与 LaserScan 相同的解析射线命中结果生成：墙和静态
障碍速度为零，动态障碍命中点携带圆障碍速度，因此它是一份完整、无重复的输入。
