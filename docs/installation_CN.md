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

<details>
<summary>手动 apt 安装命令</summary>

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git libeigen3-dev libgtest-dev libyaml-cpp-dev \
  python3-colcon-common-extensions \
  ros-$ROS_DISTRO-ament-cmake ros-$ROS_DISTRO-ament-cmake-gtest \
  ros-$ROS_DISTRO-ament-index-python \
  ros-$ROS_DISTRO-diagnostic-msgs ros-$ROS_DISTRO-geometry-msgs \
  ros-$ROS_DISTRO-launch ros-$ROS_DISTRO-launch-ros \
  ros-$ROS_DISTRO-nav-msgs ros-$ROS_DISTRO-rclcpp \
  ros-$ROS_DISTRO-rclcpp-components ros-$ROS_DISTRO-ros2launch \
  ros-$ROS_DISTRO-rviz2 ros-$ROS_DISTRO-sensor-msgs \
  ros-$ROS_DISTRO-std-msgs ros-$ROS_DISTRO-tf2 ros-$ROS_DISTRO-tf2-ros \
  ros-$ROS_DISTRO-visualization-msgs
```

</details>

OSQP v1.0.0、osqp-eigen v0.11.2 和 QDLDL v0.1.8 已包含在 `thirdparty/`
目录中；构建过程不会联网下载求解器，也不会选择系统中的其他求解器版本。

## 构建与运行

```bash
./setup.sh
./build.sh
source install/setup.bash
ros2 launch neupan_sim quick_start.launch.py
```

`setup.sh` 会检查 ROS 环境和原生依赖头文件。构建并运行完整测试：

```bash
./build.sh test
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

如需单独构建某个功能包，将名称传给 `build.sh`，例如 `./build.sh neupan_ros` 或
`./build.sh test neupan_core`。

## 常见问题

- 提示 `ROS 2 is not sourced`：source `/opt/ros` 下对应的 setup 文件后重试。
- 缺少 Eigen 或 yaml-cpp 头文件：重新执行 `./install_deps.sh`。
- 无法使用 RViz：启动 demo 时添加 `rviz:=false`。
- 切换 ROS 发行版后构建异常：使用新的工作空间，或删除生成的 `build`、`install` 和
  `log` 目录后重新构建。
