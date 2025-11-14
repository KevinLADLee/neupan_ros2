# NeuPAN ROS2 Workspace

<div align="center">

<a href="https://ieeexplore.ieee.org/abstract/document/10938329"><img src='https://img.shields.io/badge/PDF-IEEE-brightgreen' alt='PDF'></a>
<a href="https://arxiv.org/pdf/2403.06828.pdf"><img src='https://img.shields.io/badge/PDF-Arxiv-brightgreen' alt='PDF'></a>
<a href="https://youtu.be/SdSLWUmZZgQ"><img src='https://img.shields.io/badge/Video-Youtube-blue' alt='youtube'></a>
<a href="https://www.bilibili.com/video/BV1Zx421y778/?vd_source=cf6ba629063343717a192a5be9fe8985"><img src='https://img.shields.io/badge/Video-Bilibili-blue' alt='youtube'></a>
<a href="https://hanruihua.github.io/neupan_project/"><img src='https://img.shields.io/badge/Website-NeuPAN-orange' alt='website'></a>
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-GPL%20v3-blue.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.10+-green.svg)](https://www.python.org/)

[English](README.md) | [中文](#概述)

</div>

---

### 概述

**NeuPAN ROS2 工作空间** 是一个基于 ROS2 的完整导航系统，包含：

- **NeuPAN 规划器** ([`src/neupan_ros2`](src/neupan_ros2)): 基于神经网络的端到端导航规划
- **DDR Minimal Sim** ([`src/ddr_minimal_sim`](src/ddr_minimal_sim)): 轻量级差分驱动机器人仿真器

### 主要特性

- 🤖 **端到端学习**: 使用神经网络直接从激光扫描生成速度指令
- 🎯 **实时规划**: 快速神经网络推理，响应迅速
- 🔄 **仿真到真机**: 在仿真和物理机器人（如 Limo）之间无缝切换
- 🛠️ **易于测试**: 预配置场景用于算法验证
- 📦 **模块化设计**: 独立包设计，使用灵活

### 演示

<div align="center">
  <img src="imgs/sim-maze.gif" alt="NeuPAN 在迷宫场景中导航" width="600"/>
  <p><em>NeuPAN 在复杂迷宫场景中自主导航</em></p>
</div>

### 快速开始

#### 1. 前置要求

- **ROS2**: Humble 或更新版本
- **系统**: Ubuntu 22.04 (推荐)
- **硬件**: CPU (可选 GPU 用于加速训练)

#### 2. 安装依赖

```bash
# 克隆工作空间
git clone https://github.com/KevinLADLee/neupan_ros2.git
cd neupan_ros2

# 运行安装脚本（仅安装 ROS2 和 C++ 依赖）
chmod +x setup.sh
./setup.sh
```

<details>
<summary>或手动安装</summary>

**ROS2 依赖:**
```bash
sudo apt update
sudo apt install -y \
    ros-humble-tf2-tools \
    ros-humble-tf2-ros \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-visualization-msgs \
    libeigen3-dev \
    libyaml-cpp-dev
```

**Python 依赖:**

⚠️ **重要:** NeuPAN 需要 numpy < 2.0

请参考官方 NeuPAN 仓库了解详细安装说明：
**https://github.com/hanruihua/NeuPAN**

典型安装：
```bash
# 安装 PyTorch (从 https://pytorch.org 选择 CPU 或 GPU 版本)
pip3 install torch torchvision

# 安装 NeuPAN 和依赖
pip3 install neupan
pip3 install "numpy<2.0" scipy matplotlib pyyaml
```
</details>

#### 3. 编译工作空间

```bash
# 使用编译脚本
chmod +x build.sh
./build.sh

# 或手动编译
colcon build --symlink-install
source install/setup.bash
```

#### 4. 运行演示

**仿真 + NeuPAN:**
```bash
source install/setup.bash
ros2 launch neupan_ros2 sim_diff_launch.py sim_env_config:=scenario_corridor.yaml
```

**可用场景:** 查看 [ddr_minimal_sim 场景列表](src/ddr_minimal_sim/README.md#pre-configured-scenarios) 获取完整列表（走廊、迷宫、窄通道、U型陷阱、随机障碍、空旷空间）

### 使用场景

#### 场景 1: 真实机器人部署 (Limo)

在物理 Limo 机器人上部署 NeuPAN:

```bash
# 确保 Limo 驱动正在运行
ros2 launch neupan_ros2 limo_diff_launch.py
```

#### 场景 2: 完整仿真

完整系统（仿真器 + NeuPAN 规划器）:

```bash
ros2 launch neupan_ros2 sim_diff_launch.py

ros2 launch neupan_ros2 sim_diff_launch.py sim_env_config:=scenario_maze.yaml
```

### 包详情

#### 📦 src/neupan_ros2

[NeuPAN-ROS](https://github.com/hanruihua/neupan_ros) 神经导航规划器的 ROS2 实现。
**[→ 文档](src/neupan_ros2/README.md)**

#### 📦 src/ddr_minimal_sim

轻量级差分驱动机器人仿真器，支持激光扫描和预配置测试场景。
**[→ 文档](src/ddr_minimal_sim/README.md)**


### 开发

#### 编译

```bash
# 编译所有包
colcon build --symlink-install

# 编译特定包
colcon build --packages-select neupan_ros2
colcon build --packages-select ddr_minimal_sim
```

#### 自定义

- **仿真器场景**: 参见 [ddr_minimal_sim 开发指南](src/ddr_minimal_sim/README.md#development) 了解自定义场景
- **NeuPAN 参数**: 修改 `src/neupan_ros2/config/neupan_config/neupan_sim_diff.yaml`

### 引用

如果这段代码或论文对你有帮助，请为本仓库点个星 ⭐ 并引用我们的论文：

```bibtex
@ARTICLE{10938329,
  author={Han, Ruihua and Wang, Shuai and Wang, Shuaijun and Zhang, Zeqing and Chen, Jianjun and Lin, Shijie and Li, Chengyang and Xu, Chengzhong and Eldar, Yonina C. and Hao, Qi and Pan, Jia},
  journal={IEEE Transactions on Robotics},
  title={NeuPAN: Direct Point Robot Navigation With End-to-End Model-Based Learning},
  year={2025},
  volume={41},
  number={},
  pages={2804-2824},
  doi={10.1109/TRO.2025.3554252}}
```

### 许可证

本项目采用 **GNU 通用公共许可证 v3.0** 授权 - 详见 [LICENSE](LICENSE) 文件。

### 故障排除

<details>
<summary>编译错误</summary>

- 确保所有依赖已安装: `./setup.sh`
- 检查 ROS2 已source: `source /opt/ros/humble/setup.bash`
- 清理编译: `rm -rf build install log && colcon build`
</details>

<details>
<summary>运行时错误</summary>

- 验证包发现: `ros2 pkg list | grep -E "neupan|ddr"`
- 检查话题: `ros2 topic list`
- 查看日志: `ros2 run rqt_console rqt_console`
</details>

<details>
<summary>NeuPAN 模型未找到</summary>

- 检查模型文件存在: `src/neupan_ros2/config/dune_checkpoint/model_5000.pth`
- 如果缺失，从仓库下载
</details>

### 贡献

欢迎贡献！请在 [GitHub](https://github.com/KevinLADLee/neupan_ros2) 提交 issue 或 pull request。

### 致谢

- [NeuPAN](https://github.com/hanruihua/NeuPAN) & [NeuPAN-ROS](https://github.com/hanruihua/neupan_ros) - 原始算法
- [DDR-opt](https://github.com/ZJU-FAST-Lab/DDR-opt) - 仿真器参考
- ROS2 集成由 [HiveMatrix Limited](mailto:sales@hive-matrix.com) 在 [松灵机器人 Limo](https://global.agilex.ai/products/limo-pro) 平台优化与测试
  - *对 Limo 机器人或技术支持感兴趣？联系 HiveMatrix - 您的商业支持为持续开源开发提供动力*

---

**作者**: KevinLADLee (kevinladlee@gmail.com)
