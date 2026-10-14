# 安装说明

NeuPAN ROS 2 测试以下组合：

| ROS 2 | Ubuntu |
| --- | --- |
| Humble | 22.04（Jammy） |
| Jazzy | 24.04（Noble） |

请先安装对应的 ROS 2 发行版。本工作空间不使用 `rosdep`。

## 克隆与安装依赖

```bash
git clone https://github.com/KevinLADLee/neupan_ros2.git
cd neupan_ros2

ROS_DISTRO=humble
source /opt/ros/$ROS_DISTRO/setup.bash
./install_deps.sh
```

将 `ROS_DISTRO` 设置为 `humble` 或 `jazzy`。`install_deps.sh` 根据该值使用 apt 安装
Ubuntu 和 ROS 2 依赖。

apt 软件包的权威清单统一维护在
[`install_deps.sh`](../install_deps.sh) 中，避免脚本、文档和 CI 说明发生漂移。

OSQP v1.0.0、osqp-eigen v0.11.2 和 QDLDL v0.1.8 已包含在 `thirdparty/`
目录的 `neupan_solver_vendor` 功能包中；构建过程不会联网下载求解器，也不会选择
系统中的其他求解器版本。

## 构建与运行

```bash
./build.sh
source install/setup.bash
ros2 launch neupan_sim quick_start.launch.py
```

`build.sh` 会在构建前检查 ROS 发行版、原生依赖头文件和随仓库提供的求解器源码。
构建并运行完整测试：

```bash
./build.sh --test
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

如需单独构建某个功能包，使用 `--package`，例如
`./build.sh --package neupan_ros` 或 `./build.sh --test --package neupan_core`。
原有的位置参数写法仍然兼容。

### 集成到已有工作空间

也可以将本仓库克隆到已有工作空间的源码目录：

```bash
cd ~/ros2_ws/src
git clone https://github.com/KevinLADLee/neupan_ros2.git
cd ..
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Colcon 会同时发现 `neupan_solver_vendor` 和三个第一方功能包，并按依赖顺序构建；
无需运行仓库内的 setup 或第三方构建命令。

## 常见问题

- 提示 `ROS 2 is not sourced`：source `/opt/ros` 下对应的 setup 文件后重试。
- 缺少 Eigen 或 yaml-cpp 头文件：重新执行 `./install_deps.sh`。
- 无法使用 RViz：启动 demo 时添加 `rviz:=false`。
- 切换 ROS 发行版后构建异常：使用新的工作空间，或删除生成的 `build`、`install` 和
  `log` 目录后重新构建。
