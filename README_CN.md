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
<a href="LICENSE"><img src="https://img.shields.io/badge/License-GPL--3.0--or--later-blue" alt="GPL-3.0-or-later 协议"></a>

[English](README.md) | [中文](README_CN.md)

</div>

NeuPAN ROS 2 通过原生 C++ 实现，将
[NeuPAN](https://github.com/hanruihua/NeuPAN) 导航框架带到 ROS 2，面向机器人部署与仿真。
项目支持 ROS 2 Humble 和 Jazzy。

由 [Hive Matrix Limited](mailto:sales@hive-matrix.com) 维护。

## 致谢

本项目基于 [NeuPAN](https://github.com/hanruihua/NeuPAN)、
[NeuPAN-ROS](https://github.com/hanruihua/neupan_ros) 以及 Kaiyuan Zhang 的
[neupan_cpp](https://github.com/zhangkaiyuan007/neupan_cpp)，仿真工作还参考了
[DDR-opt](https://github.com/ZJU-FAST-Lab/DDR-opt)。NeuPAN 算法成果归原论文全体作者所有。

## Quick Start

安装 ROS 2 Humble 或 Jazzy 并克隆仓库。将 `ROS_DISTRO` 设置为 `humble` 或 `jazzy`，
然后执行：

```bash
ROS_DISTRO=humble
source /opt/ros/$ROS_DISTRO/setup.bash
./install_deps.sh
./build.sh
source install/setup.bash
ros2 launch neupan_sim quick_start.launch.py
```

该命令会启动演示和 RViz；无界面运行时添加 `rviz:=false`。系统要求和构建选项见
[安装说明](docs/installation_CN.md)。

## 工作空间功能包

| 功能包 | 用途 | 文档 |
| --- | --- | --- |
| `neupan_solver_vendor` | 离线求解器依赖 | [README](thirdparty/README.md) |
| `neupan_core` | NeuPAN 规划库 | [README](src/neupan_core/README.md) |
| `neupan_ros` | ROS 2 集成 | [README](src/neupan_ros/README.md) |
| `neupan_sim` | 仿真与验证 | [README](src/neupan_sim/README.md) |

工作空间结构见 [src/README.md](src/README.md)，离线模型训练见
[`training/README.md`](training/README.md)。
[Scout Mini Diff 示例](examples/scout_mini_diff/README.md) 提供 612 mm × 580 mm
车体的模型训练、配套配置和运行方法。

## 文档

- [坐标系约定](docs/coordinate_frames_CN.md)
- [ROS 2 话题概览](docs/ros_interfaces_CN.md)
- [动态障碍物消息约定](docs/dynamic_obstacles_CN.md)

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

## 开源协议

NeuPAN ROS 2 的第一方代码采用
[GNU General Public License v3.0 或更高版本](LICENSE)。本项目移植或参考的
NeuPAN、NeuPAN-ROS、neupan_cpp 和 DDR-opt 均为 GPL-3.0 项目。

随仓库提供的求解器组件保留各自的 Apache-2.0 或 BSD-3-Clause 协议。版本、来源、
许可证文件及再分发声明见[第三方依赖清单](thirdparty/README.md)。
