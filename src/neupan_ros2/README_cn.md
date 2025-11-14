# NeuPAN ROS2

<a href="https://ieeexplore.ieee.org/abstract/document/10938329"><img src='https://img.shields.io/badge/PDF-IEEE-brightgreen' alt='PDF'></a>
<a href="https://arxiv.org/pdf/2403.06828.pdf"><img src='https://img.shields.io/badge/PDF-Arxiv-brightgreen' alt='PDF'></a>
<a href="https://youtu.be/SdSLWUmZZgQ"><img src='https://img.shields.io/badge/Video-Youtube-blue' alt='youtube'></a>
<a href="https://www.bilibili.com/video/BV1Zx421y778/?vd_source=cf6ba629063343717a192a5be9fe8985"><img src='https://img.shields.io/badge/Video-Bilibili-blue' alt='youtube'></a>
<a href="https://hanruihua.github.io/neupan_project/"><img src='https://img.shields.io/badge/Website-NeuPAN-orange' alt='website'></a>
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-GPL%20v3-blue.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.10+-green.svg)](https://www.python.org/)

**中文版** | [**English**](README.md)

---

## 🌟 项目概述

**NeuPAN ROS2** 是 [NeuPAN](https://github.com/hanruihua/neupan) 的 ROS2 封装包，NeuPAN 是一个先进的端到端模型学习框架，用于基于点云的机器人直接导航。本软件包实现了 NeuPAN 强大的导航能力与 ROS2 机器人系统的无缝集成。

### 主要特性

- ✨ **端到端学习**：基于点云的直接导航，无需显式建图
- 🚀 **实时性能**：高效的神经网络推理，实现自主导航
- 🤖 **多平台支持**：同时支持仿真环境和实体机器人（Limo、自定义平台）
- 🔧 **灵活配置**：基于 YAML 的易于定制的参数系统
- 📡 **ROS2 原生**：与 ROS2 Humble 生态系统完全集成

---

## 📦 前置条件与依赖

### 系统要求

- **操作系统**：Ubuntu 22.04 LTS
- **ROS2 发行版**：Humble Hawksbill
- **Python**：3.10 或更高版本

### 核心依赖

#### ROS2 软件包
```bash
# ROS2 Humble（推荐完整桌面版安装）
sudo apt install ros-humble-desktop-full

# 额外的 ROS2 软件包
sudo apt install ros-humble-rviz2 \
                 ros-humble-tf2-ros \
                 ros-humble-sensor-msgs \
                 ros-humble-nav-msgs \
                 ros-humble-geometry-msgs \
                 ros-humble-visualization-msgs
```

#### Python 依赖

⚠️ **重要**：NeuPAN 需要 numpy < 2.0

```bash
# PyTorch（根据您的配置选择 CPU 或 GPU 版本）
# 访问 https://pytorch.org 了解安装选项
pip3 install torch torchvision

# NeuPAN 核心库
pip3 install neupan

# 其他 Python 包（注意 numpy 版本要求）
pip3 install "numpy<2.0" scipy matplotlib pyyaml
```

详细的 Python 环境设置请参考官方 NeuPAN 仓库：
**https://github.com/hanruihua/NeuPAN**

### 可选依赖

- **仿真环境**：[ddr_minimal_sim](../ddr_minimal_sim)（包含在本工作空间中）
- **Limo 机器人**：AgileX Limo ROS2 驱动软件包

---

## 🚀 安装步骤

> **注意**：此软件包现已成为 NeuPAN ROS2 工作空间的一部分。完整安装说明请参见[工作空间 README](../../README.md)。

### 快速安装（作为工作空间的一部分）

此软件包已与 ddr_minimal_sim 一起包含在 NeuPAN ROS2 工作空间中。安装步骤：

```bash
# 克隆工作空间
git clone https://github.com/KevinLADLee/neupan_ros2.git
cd neupan_ros2

# 安装系统依赖
chmod +x setup.sh
./setup.sh

# 安装 Python 依赖（参见上述要求）
pip3 install neupan
pip3 install torch torchvision
pip3 install "numpy<2.0" scipy matplotlib pyyaml

# 构建工作空间
chmod +x build.sh
./build.sh

# Source 工作空间
source install/setup.bash
```

详细的安装、故障排除和使用说明，请参阅[工作空间 README](../../README.md)。

---

---

## 📖 快速开始

### 🎮 1. 仿真模式（使用 ddr_minimal_sim）

启动包含 NeuPAN 规划器的完整仿真环境：

```bash
# 激活工作空间
source ~/neupan_ws/install/setup.bash

# 使用默认环境启动
ros2 launch neupan_ros2 sim_diff_launch.py

# 或指定自定义环境配置
ros2 launch neupan_ros2 sim_diff_launch.py sim_env_config:=sim_env_obs.yaml use_rviz:=true
```

**可用环境配置**：
- `sim_env_obs.yaml`：基础障碍物环境
- `sim_env_obs_exam.yaml`：复杂障碍物场景（默认）

### 🤖 2. 实体机器人模式（Limo 平台）

用于 AgileX Limo 差速驱动机器人：

```bash
# 在 Limo 机器人上启动 NeuPAN
ros2 launch neupan_ros2 limo_diff_launch.py

# 使用自定义配置
ros2 launch neupan_ros2 limo_diff_launch.py config:=limo_diff.yaml
```

> **注意**：本软件包已针对 [AgileX Limo ROS2](https://www.agilex.ai/education/18) 机器人进行优化。如需了解该平台信息，请联系我们的合作伙伴：sales@hive-matrix.com。

### ⚙️ 3. 自定义配置

```bash
# 使用自定义参数文件启动
ros2 launch neupan_ros2 neupan_launch.py config:=neupan_params.yaml
```

---

## 🎯 配置说明

### 配置文件

配置文件位于：
```
config/
├── limo_diff.yaml              # Limo 机器人配置
├── sim_diff.yaml                # 仿真配置
└── neupan_config/
    ├── neupan_sim_diff.yaml     # NeuPAN 规划器参数
    └── dune_checkpoint/
        └── model_5000.pth       # 预训练神经网络模型
```

### 关键参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `use_sim_time` | 使用仿真时间 | `true`/`false` |
| `neupan_config_file` | 规划器配置文件 | `neupan_sim_diff.yaml` |
| `dune_checkpoint_file` | 神经网络模型文件 | `model_5000.pth` |
| `map_frame` | 全局坐标系 | `map` |
| `base_frame` | 机器人基座坐标系 | `base_link` |
| `scan_range_max` | 激光扫描最大距离（米） | `5.0` |
| `scan_range_min` | 激光扫描最小距离（米） | `0.01` |
| `ref_speed` | 参考导航速度（米/秒） | `0.5` |
| `collision_threshold` | 碰撞避障阈值（米） | `0.01` |

完整参数文档请参见 [config/sim_diff.yaml](config/sim_diff.yaml)。

---

## 📚 文档说明

### ROS2 话题

#### 订阅话题
| 话题 | 类型 | 说明 |
|------|------|------|
| `/scan` | `sensor_msgs/LaserScan` | 用于障碍物检测的激光扫描数据 |
| `/plan` | `nav_msgs/Path` | 全局路径航点 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | 导航目标位姿 |

#### 发布话题
| 话题 | 类型 | 说明 |
|------|------|------|
| `/neupan_cmd_vel` | `geometry_msgs/Twist` | 速度命令（默认重映射到 `/cmd_vel`） |
| `/neupan_plan` | `nav_msgs/Path` | 优化后的轨迹 |
| `/neupan_ref_state` | `nav_msgs/Path` | 参考状态轨迹 |
| `/neupan_initial_path` | `nav_msgs/Path` | 初始路径可视化 |
| `/dune_point_markers` | `visualization_msgs/MarkerArray` | DUNE 网络可视化 |
| `/nrmp_point_markers` | `visualization_msgs/MarkerArray` | NRMP 网络可视化 |
| `/robot_marker` | `visualization_msgs/Marker` | 机器人轮廓可视化 |

### TF 坐标树

```
map
 └── odom（可选）
      └── base_link
           └── laser_link（如果独立）
```

### Launch 文件

| Launch 文件 | 用途 | 使用场景 |
|-------------|------|----------|
| `sim_diff_launch.py` | 完整仿真系统 | 仿真测试 |
| `limo_diff_launch.py` | Limo 机器人部署 | 实体机器人导航 |
| `neupan_launch.py` | 独立规划器节点 | 自定义集成 |

---

## 🔗 相关链接

- **原始 ROS1 封装**：[NeuPAN-ROS](https://github.com/hanruihua/neupan_ros)
- **核心算法库**：[NeuPAN](https://github.com/hanruihua/neupan)
- **研究论文**：[IEEE Transactions on Robotics (2025)](https://ieeexplore.ieee.org/document/10938329)
- **项目主页**：[NeuPAN Project Page](https://hanruihua.github.io/neupan_project/)
- **ROS2 Humble 文档**：[docs.ros.org/en/humble](https://docs.ros.org/en/humble/)

---

## 📄 开源协议

本项目采用 [GNU General Public License v3.0](LICENSE) 协议开源。

---

## 📖 引用

如果您觉得本代码或论文对您有帮助，感谢您为本仓库点个星标 ⭐ 并引用我们的论文：

```bibtex
@article{han2025neupan,
  title={Neupan: Direct point robot navigation with end-to-end model-based learning},
  author={Han, Ruihua and Wang, Shuai and Wang, Shuaijun and Zhang, Zeqing and Chen, Jianjun and Lin, Shijie and Li, Chengyang and Xu, Chengzhong and Eldar, Yonina C and Hao, Qi and others},
  journal={IEEE Transactions on Robotics},
  year={2025},
  publisher={IEEE}
}
```

---

## 🤝 致谢

- **NeuPAN 原始算法**：由香港大学 [Ruihua HAN](https://github.com/hanruihua) 及`SIAT-INVS`团队开发
- **ROS2 集成**：针对 AgileX Limo 平台优化和测试
- **硬件合作伙伴**：AgileX x Hive Matrix（[sales@hive-matrix.com](mailto:sales@hive-matrix.com)）

---

## 📮 联系与支持

如有问题、反馈或合作机会，请联系：

- **Issues**：[GitHub Issues](https://github.com/KevinLADLee/neupan_ros2/issues)
- **此项目联系邮箱**：chengyangli@connect.hku.hk
- **原项目维护者**：hanrh@connect.hku.hk

---

**🎉 祝您使用 NeuPAN 导航愉快！🤖**
