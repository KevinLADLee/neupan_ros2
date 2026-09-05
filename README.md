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

NeuPAN ROS 2 brings the [NeuPAN](https://github.com/hanruihua/NeuPAN) navigation
framework to ROS 2 with a native C++ implementation for robot deployment and
simulation. It supports ROS 2 Humble and Jazzy.

Maintained by [Hive Matrix Limited](mailto:sales@hive-matrix.com).

## Acknowledgments

This project builds on [NeuPAN](https://github.com/hanruihua/NeuPAN),
[NeuPAN-ROS](https://github.com/hanruihua/neupan_ros), and Kaiyuan Zhang's
[neupan_cpp](https://github.com/zhangkaiyuan007/neupan_cpp). The simulation work
also references [DDR-opt](https://github.com/ZJU-FAST-Lab/DDR-opt). NeuPAN
algorithm credit remains with all authors of the original paper.

## Quick Start

Install ROS 2 Humble or Jazzy and clone the repository. Set `ROS_DISTRO` to
`humble` or `jazzy`, then run:

```bash
ROS_DISTRO=humble
source /opt/ros/$ROS_DISTRO/setup.bash
./install_deps.sh
./setup.sh
./build.sh
source install/setup.bash
ros2 launch neupan_sim quick_start.launch.py
```

This starts the demonstration and RViz. Use `rviz:=false` for a headless run.
See the [installation guide](docs/installation.md) for prerequisites and build
options.

## ROS 2 packages

| Package | Purpose | Documentation |
| --- | --- | --- |
| `neupan_core` | NeuPAN planning library | [README](src/neupan_core/README.md) |
| `neupan_ros` | ROS 2 integration | [README](src/neupan_ros/README.md) |
| `neupan_sim` | Simulation and verification | [README](src/neupan_sim/README.md) |

See [src/README.md](src/README.md) for the workspace structure. Offline model
training is documented in [`training/README.md`](training/README.md).

## Documentation

- [Coordinate-frame contract](docs/coordinate_frames.md)
- [Algorithm equivalence](docs/algorithm_equivalence_CN.md)
- [ROS 2 topic overview (Chinese)](docs/ros_interfaces_CN.md)
- [Dynamic-obstacle message contract (Chinese)](docs/dynamic_obstacles_CN.md)

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

## License

[GNU General Public License v3.0](LICENSE).
