<div align="center">

# NeuPAN ROS 2

**Native C++ ROS 2 local planner based on NeuPAN**

<a href="https://github.com/hanruihua/NeuPAN"><img src="https://img.shields.io/github/stars/hanruihua/NeuPAN?style=flat" alt="NeuPAN stars"></a>
<a href="https://ieeexplore.ieee.org/document/10938329"><img src="https://img.shields.io/badge/Paper-IEEE-brightgreen" alt="IEEE paper"></a>
<a href="https://arxiv.org/pdf/2403.06828.pdf"><img src="https://img.shields.io/badge/Paper-arXiv-brightgreen" alt="arXiv paper"></a>
<a href="https://youtu.be/SdSLWUmZZgQ"><img src="https://img.shields.io/badge/Video-YouTube-red" alt="YouTube video"></a>
<a href="https://www.bilibili.com/video/BV1Zx421y778/"><img src="https://img.shields.io/badge/Video-Bilibili-blue" alt="Bilibili video"></a>
<a href="https://hanruihua.github.io/neupan_project/"><img src="https://img.shields.io/badge/Website-NeuPAN-orange" alt="NeuPAN website"></a>

<a href="https://docs.ros.org/en/humble/"><img src="https://img.shields.io/badge/ROS%202-Humble-blue" alt="ROS 2 Humble"></a>
<a href="https://docs.ros.org/en/jazzy/"><img src="https://img.shields.io/badge/ROS%202-Jazzy-blue" alt="ROS 2 Jazzy"></a>
<a href="https://github.com/KevinLADLee/neupan_ros2/actions/workflows/ros2-ci.yml"><img src="https://github.com/KevinLADLee/neupan_ros2/actions/workflows/ros2-ci.yml/badge.svg" alt="ROS 2 CI"></a>
<a href="LICENSE"><img src="https://img.shields.io/badge/License-GPL%20v3-blue" alt="GPL v3 license"></a>

[English](README.md) | [中文](README_CN.md)

</div>

NeuPAN ROS 2 provides ROS 2 packages for deploying
[NeuPAN](https://github.com/hanruihua/NeuPAN) as a standalone local-planning
node. The node subscribes to a reference path and 2D obstacle observations,
obtains the robot pose from TF, and publishes a `geometry_msgs/msg/Twist`
velocity command and `nav_msgs/msg/Path` local trajectory for a
differential-drive robot. It is not a Nav2 controller plugin.

- The ROS 2 nodes and planning library are implemented in C++17; online
  planning does not require PyTorch, NumPy, or `rclpy`.
- Tested on ROS 2 Humble/Ubuntu 22.04 and ROS 2 Jazzy/Ubuntu 24.04.
- CPU-oriented and reproducible: OSQP, osqp-eigen, and QDLDL are bundled.

This project is actively developed and maintained by
[Hive Matrix Limited](mailto:sales@hive-matrix.com), a startup founded by
KevinLADLee, who is also one of the NeuPAN authors.

## Quick Start

Install ROS 2 Humble or Jazzy, clone the repository, and run:

```bash
source /opt/ros/humble/setup.bash  # or /opt/ros/jazzy/setup.bash
./install_deps.sh                  # explicit apt packages; no rosdep
./setup.sh
./build.sh
source install/setup.bash
ros2 launch neupan_sim quick_start.launch.py
```

This starts the simulator node, local-planner node, and RViz. Use
`rviz:=false` for a headless run. See the
[installation guide](docs/installation.md) for cloning, prerequisites, manual
dependency installation, build options, and troubleshooting.

## ROS 2 packages

| Package | Contents | Package documentation |
| --- | --- | --- |
| `neupan_core` | ROS-independent C++ planning library | [C++ API and planner configuration](src/neupan_core/README.md) |
| `neupan_ros` | `neupan_node`, `astar_global_node`, and launch/config files | [Nodes, parameters, topics, and TF](src/neupan_ros/README.md) |
| `neupan_sim` | `neupan_sim_node` and integration-test scenarios | [Node, parameters, topics, and launch files](src/neupan_sim/README.md) |

`neupan_ros` depends on `neupan_core`; `neupan_sim` provides a closed-loop demo
and integration-test environment. See [src/README.md](src/README.md) for build
types and installed targets.

Offline model training is a separate Python project under [`training/`](training/README.md).
It is not a ROS 2 package and is not built by colcon.

## Documentation

- [Coordinate-frame contract](docs/coordinate_frames.md)
- [Algorithm equivalence](docs/algorithm_equivalence_CN.md)
- [ROS 2 topic overview (Chinese)](docs/ros_interfaces_CN.md)
- [Dynamic-obstacle message contract (Chinese)](docs/dynamic_obstacles_CN.md)

Current scope: differential-drive robots, line-style initial paths, and
per-point constant-velocity obstacle prediction. Ackermann, omni, and
Dubins/loop paths are not yet supported by this C++ runtime.

## Citation

If NeuPAN is useful in your work, please cite the original paper:

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

## Acknowledgments

Current development, ROS 2 integration, and robot testing are led by
[Hive Matrix Limited](mailto:sales@hive-matrix.com). This workspace builds on
[NeuPAN](https://github.com/hanruihua/NeuPAN),
[NeuPAN-ROS](https://github.com/hanruihua/neupan_ros), and Kaiyuan Zhang's
[neupan_cpp](https://github.com/zhangkaiyuan007/neupan_cpp). The simulation work
also references [DDR-opt](https://github.com/ZJU-FAST-Lab/DDR-opt). NeuPAN
algorithm credit remains with all authors of the original paper.

## License

[GNU General Public License v3.0](LICENSE).
