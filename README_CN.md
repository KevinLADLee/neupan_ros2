<div align="center">

# NeuPAN ROS 2

**基于 NeuPAN 的原生 C++ ROS 2 局部规划器**

<a href="https://github.com/hanruihua/NeuPAN"><img src="https://img.shields.io/github/stars/hanruihua/NeuPAN?style=flat" alt="NeuPAN stars"></a>
<a href="https://ieeexplore.ieee.org/document/10938329"><img src="https://img.shields.io/badge/Paper-IEEE-brightgreen" alt="IEEE 论文"></a>
<a href="https://arxiv.org/pdf/2403.06828.pdf"><img src="https://img.shields.io/badge/Paper-arXiv-brightgreen" alt="arXiv 论文"></a>
<a href="https://youtu.be/SdSLWUmZZgQ"><img src="https://img.shields.io/badge/Video-YouTube-red" alt="YouTube 视频"></a>
<a href="https://www.bilibili.com/video/BV1Zx421y778/"><img src="https://img.shields.io/badge/Video-Bilibili-blue" alt="Bilibili 视频"></a>
<a href="https://hanruihua.github.io/neupan_project/"><img src="https://img.shields.io/badge/Website-NeuPAN-orange" alt="NeuPAN 项目主页"></a>

<a href="https://docs.ros.org/en/humble/"><img src="https://img.shields.io/badge/ROS%202-Humble-blue" alt="ROS 2 Humble"></a>
<a href="https://docs.ros.org/en/jazzy/"><img src="https://img.shields.io/badge/ROS%202-Jazzy-blue" alt="ROS 2 Jazzy"></a>
<a href="https://github.com/KevinLADLee/neupan_ros2/actions/workflows/ros2-ci.yml"><img src="https://github.com/KevinLADLee/neupan_ros2/actions/workflows/ros2-ci.yml/badge.svg" alt="ROS 2 CI"></a>
<a href="LICENSE"><img src="https://img.shields.io/badge/License-GPL%20v3-blue" alt="GPL v3 协议"></a>

[English](README.md) | [中文](README_CN.md)

</div>

NeuPAN ROS 2 提供将 [NeuPAN](https://github.com/hanruihua/NeuPAN) 部署为独立局部规划
节点的 ROS 2 功能包。节点订阅参考路径和二维障碍物观测，通过 TF 获取机器人位姿，并为
差速机器人发布 `geometry_msgs/msg/Twist` 速度指令和 `nav_msgs/msg/Path` 局部轨迹。
该节点不是 Nav2 controller plugin。

- ROS 2 节点和规划库使用 C++17 实现；在线规划不依赖 PyTorch、NumPy 或 `rclpy`。
- 支持 ROS 2 Humble/Ubuntu 22.04 与 ROS 2 Jazzy/Ubuntu 24.04。
- 面向 CPU 可复现部署：仓库内置 OSQP、osqp-eigen 和 QDLDL。

本项目目前由 [Hive Matrix Limited](mailto:sales@hive-matrix.com) 持续开发和维护。
Hive Matrix Limited 是 KevinLADLee 创立的初创公司；KevinLADLee 同时也是 NeuPAN 作者之一。

## Quick Start

安装 ROS 2 Humble 或 Jazzy，克隆仓库后执行：

```bash
source /opt/ros/humble/setup.bash  # 或 /opt/ros/jazzy/setup.bash
./install_deps.sh                  # 显式安装 apt 依赖，不使用 rosdep
./setup.sh
./build.sh
source install/setup.bash
ros2 launch neupan_sim quick_start.launch.py
```

该命令会启动仿真节点、局部规划节点和 RViz；无界面运行时添加
`rviz:=false`。克隆方式、系统要求、手动安装命令、构建选项及排障见
[安装说明](docs/installation_CN.md)。

## ROS 2 功能包

| 功能包 | 内容 | 功能包文档 |
| --- | --- | --- |
| `neupan_core` | 与 ROS 通信无关的 C++ 规划库 | [C++ API 与规划器配置](src/neupan_core/README.md) |
| `neupan_ros` | `neupan_node`、`astar_global_node` 及启动/配置文件 | [节点、参数、话题与 TF](src/neupan_ros/README.md) |
| `neupan_sim` | `neupan_sim_node` 及集成测试场景 | [节点、参数、话题与启动文件](src/neupan_sim/README.md) |

`neupan_ros` 依赖 `neupan_core`；`neupan_sim` 提供闭环演示和集成测试环境。各功能包的
构建类型与安装目标见 [src/README.md](src/README.md)。

离线模型训练位于 [`training/`](training/README.md)，它是独立 Python 项目，不是 ROS 2
功能包，也不由 colcon 构建。

## 文档

- [坐标系约定](docs/coordinate_frames_CN.md)
- [算法等价性说明](docs/algorithm_equivalence_CN.md)
- [ROS 2 话题概览](docs/ros_interfaces_CN.md)
- [动态障碍物消息约定](docs/dynamic_obstacles_CN.md)

当前 C++ 运行时支持差速机器人、线式初始路径和逐点常速度障碍预测；暂不支持
Ackermann、omni 和 Dubins/loop 路径。

## 引用

如果 NeuPAN 对您的工作有帮助，请引用原始论文：

```bibtex
@ARTICLE{10938329,
  author={Han, Ruihua and Wang, Shuai and Wang, Shuaijun and Zhang, Zeqing and Chen, Jianjun and Lin, Shijie and Li, Chengyang and Xu, Chengzhong and Eldar, Yonina C. and Hao, Qi and Pan, Jia},
  journal={IEEE Transactions on Robotics},
  title={NeuPAN: Direct Point Robot Navigation With End-to-End Model-Based Learning},
  year={2025},
  volume={41},
  pages={2804-2824},
  doi={10.1109/TRO.2025.3554252}
}
```

## 致谢

当前开发、ROS 2 集成和机器人测试由
[Hive Matrix Limited](mailto:sales@hive-matrix.com) 主导。本工作空间基于
[NeuPAN](https://github.com/hanruihua/NeuPAN)、
[NeuPAN-ROS](https://github.com/hanruihua/neupan_ros) 以及 Kaiyuan Zhang 的
[neupan_cpp](https://github.com/zhangkaiyuan007/neupan_cpp)，仿真工作还参考了
[DDR-opt](https://github.com/ZJU-FAST-Lab/DDR-opt)。NeuPAN 算法成果归原论文全体作者所有。

## 开源协议

[GNU General Public License v3.0](LICENSE)。
