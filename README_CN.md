# NeuPAN ROS 2

这是一个面向 CPU 部署的 NeuPAN 工作空间，在线运行链路全部采用 C++。

原始 NeuPAN 到 C++/OSQP 的逐项推导、已验证范围和已知差异见
[算法等价性说明](docs/algorithm_equivalence_CN.md)。

## 仓库结构

- `src/neupan_core`：不依赖 ROS 的 C++ DUNE、PAN 与 NRMP 实现。
- `src/neupan_ros`：原生 `rclcpp` 节点及 ROS 消息预处理。
- `src/neupan_sim`：单进程、最小闭环验证仿真器。
- `training`：仅用于 DUNE 训练和 NPTF 模型导出的离线 Python 包。

规划器和仿真器可执行文件不依赖 rclpy、NumPy 或 PyTorch；编排仍使用 ROS2
标准 Python launch 文件。旧 Python ROS 节点和旧多节点仿真器不属于正式仓库及
colcon 工作空间。

## 编译

系统依赖为 ROS2、Eigen3 和 yaml-cpp。OSQP v1.0.0 与 osqp-eigen v0.11.2
源码固定在 `thirdparty/` 中；构建脚本会先将它们编译为 CPU 静态库，因此正常
构建不需要联网，也不会使用系统中其他版本的求解器。

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
./setup.sh
./build.sh
source install/setup.bash
```

运行可视化 Quick Start。该命令会同时启动更完整的共享仓储场景、原生 NeuPAN 节点和
预配置的 RViz：

```bash
ros2 launch neupan_sim quick_start.launch.py
```

场景包含四组交错货架、曲折参考路线、静态设施，以及三个位于开放通道内的动态横穿
障碍。无界面环境可以追加 `rviz:=false`。

简单、确定性的回归验证场景使用：

```bash
ros2 launch neupan_sim verify.launch.py
```

仿真器发布 `/scan`、`/initial_path`、`/odom` 和 TF；原生规划器输出
`/neupan_cmd_vel`，直接驱动仿真器。ROS 层同时接受 LaserScan，以及 XYZ、XYZI、XYZIV
格式的 PointCloud2。仿真器发布带 `x/y/z/intensity/vx/vy` 字段的
`/obstacles`，用于验证常速度动态障碍预测。消息契约见
[动态障碍速度接口](docs/dynamic_obstacles_CN.md)。

所有空间输入遵循严格的[坐标系接口契约](docs/coordinate_frames_CN.md)：传感器数据按消息
时间戳转换；路径和目标保留原始 frame，并持续投影到可配置的局部
`planning_frame`（默认 `odom`）。

完整的话题、参数、QoS 与坐标系行为见
[ROS 接口说明](docs/ros_interfaces_CN.md)。

## 离线训练

训练环境独立于 ROS2/colcon：

```bash
python -m venv .venv-training
. .venv-training/bin/activate
pip install -e ./training

neupan-train --output runs/diff --length 0.5 --width 0.5
neupan-export runs/diff/model_5000.pth src/neupan_core/models/diff.bin \
  --length 0.5 --width 0.5
```

当前 C++ 主线覆盖 differential-drive、line initial path 和逐点常速度动态障碍；
Ackermann、omni、Dubins/loop 尚待迁移。
