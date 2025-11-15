# NeuPAN ROS2

<a href="https://ieeexplore.ieee.org/abstract/document/10938329"><img src='https://img.shields.io/badge/PDF-IEEE-brightgreen' alt='PDF'></a>
<a href="https://arxiv.org/pdf/2403.06828.pdf"><img src='https://img.shields.io/badge/PDF-Arxiv-brightgreen' alt='PDF'></a>
<a href="https://youtu.be/SdSLWUmZZgQ"><img src='https://img.shields.io/badge/Video-Youtube-blue' alt='youtube'></a>
<a href="https://www.bilibili.com/video/BV1Zx421y778/?vd_source=cf6ba629063343717a192a5be9fe8985"><img src='https://img.shields.io/badge/Video-Bilibili-blue' alt='youtube'></a>
<a href="https://hanruihua.github.io/neupan_project/"><img src='https://img.shields.io/badge/Website-NeuPAN-orange' alt='website'></a>
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-GPL%20v3-blue.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.10+-green.svg)](https://www.python.org/)

[**中文版**](README_cn.md) | **English**

---

## 🌟 Overview

**NeuPAN ROS2** is a ROS2 wrapper for [NeuPAN](https://github.com/hanruihua/neupan), an advanced end-to-end model-based learning framework for direct point robot navigation. This package enables seamless integration of NeuPAN's powerful navigation capabilities into ROS2 robotics systems.

### Key Features

- ✨ **End-to-End Learning**: Direct point cloud-based navigation without explicit mapping
- 🚀 **Real-Time Performance**: Efficient neural network inference for autonomous navigation
- 🤖 **Multiple Platform Support**: Works with both simulation and real robots (Limo, custom platforms)
- 🔧 **Flexible Configuration**: Easy-to-customize YAML-based parameter system
- 📡 **ROS2 Native**: Full integration with ROS2 Humble ecosystem

---

## 📦 Prerequisites & Dependencies

### System Requirements

- **Operating System**: Ubuntu 22.04 LTS
- **ROS2 Distribution**: Humble Hawksbill
- **Python**: 3.10 or higher

### Core Dependencies

#### ROS2 Packages
```bash
# ROS2 Humble (full desktop installation recommended)
sudo apt install ros-humble-desktop-full

# Additional ROS2 packages
sudo apt install ros-humble-rviz2 \
                 ros-humble-tf2-ros \
                 ros-humble-sensor-msgs \
                 ros-humble-nav-msgs \
                 ros-humble-geometry-msgs \
                 ros-humble-visualization-msgs
```

#### Python Dependencies

⚠️ **Important:** NeuPAN requires numpy < 2.0

```bash
# PyTorch (CPU or GPU version depending on your setup)
# See https://pytorch.org for installation options
pip3 install torch torchvision

# NeuPAN core library
pip3 install neupan

# Other Python packages (note numpy version requirement)
pip3 install "numpy<2.0" scipy matplotlib pyyaml
```

For detailed Python environment setup, please refer to the official NeuPAN repository:
**https://github.com/hanruihua/NeuPAN**

### Optional Dependencies

- **For Simulation**: [ddr_minimal_sim](../ddr_minimal_sim) (included in this workspace)
- **For Limo Robot**: AgileX Limo ROS2 driver packages

---

## 🚀 Installation

> **Note**: This package is now part of the NeuPAN ROS2 Workspace. For complete installation instructions, please see the [workspace README](../../README.md).

### Quick Installation (Part of Workspace)

This package is included in the NeuPAN ROS2 Workspace along with ddr_minimal_sim. To install:

```bash
# Clone the workspace
git clone https://github.com/KevinLADLee/neupan_ros2.git
cd neupan_ros2

# Install system dependencies
chmod +x setup.sh
./setup.sh

# Install Python dependencies (see requirements above)
pip3 install neupan
pip3 install torch torchvision
pip3 install "numpy<2.0" scipy matplotlib pyyaml

# Build workspace
chmod +x build.sh
./build.sh

# Source the workspace
source install/setup.bash
```

For detailed installation, troubleshooting, and usage instructions, refer to the [workspace README](../../README.md).

### Step 5: Verify Installation

```bash
ros2 pkg list | grep neupan
# Should output: neupan_ros2
```

---

## 📖 Quick Start

### 🎮 1. Simulation Mode (with ddr_minimal_sim)

Launch the complete simulation environment with NeuPAN planner:

```bash
# Source your workspace
source ~/neupan_ws/install/setup.bash

# Launch with default environment
ros2 launch neupan_ros2 sim_diff_launch.py

# Or specify custom environment configuration
ros2 launch neupan_ros2 sim_diff_launch.py sim_env_config:=sim_env_obs.yaml use_rviz:=true
```

**Available Environment Configs**:
- `sim_env_obs.yaml`: Basic obstacle environment
- `sim_env_obs_exam.yaml`: Complex obstacle course (default)

### 🤖 2. Real Robot Mode (Limo Platform)

For AgileX Limo differential drive robot:

```bash
# Launch NeuPAN on Limo robot
ros2 launch neupan_ros2 limo_diff_launch.py

# With custom configuration
ros2 launch neupan_ros2 limo_diff_launch.py config:=limo_diff.yaml
```

> **Note**: This package is optimized for [AgileX Limo ROS2](https://www.agilex.ai/education/18) robot. For inquiries about this platform, contact our partner at sales@hive-matrix.com.

### ⚙️ 3. Custom Configuration

```bash
# Launch with custom parameter file
ros2 launch neupan_ros2 neupan_launch.py config:=neupan_params.yaml
```

---

## 🎯 Configuration

### Configuration Files

Configuration files are located in:
```
config/
├── limo_diff.yaml              # Limo robot configuration
├── sim_diff.yaml                # Simulation configuration
└── neupan_config/
    ├── neupan_sim_diff.yaml     # NeuPAN planner parameters
    └── dune_checkpoint/
        └── model_5000.pth       # Pre-trained neural network model
```

### Key Parameters

#### Core Parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| `use_sim_time` | Use simulation time | `true`/`false` |
| `neupan_config_file` | Planner configuration file | `neupan_sim_diff.yaml` |
| `dune_checkpoint_file` | Neural network model file | `model_5000.pth` |
| `map_frame` | Global coordinate frame | `map` |
| `base_frame` | Robot base coordinate frame | `base_link` |
| `scan_range_max` | Maximum laser scan distance (m) | `5.0` |
| `scan_range_min` | Minimum laser scan distance (m) | `0.01` |
| `ref_speed` | Reference navigation speed (m/s) | `0.5` |
| `collision_threshold` | Collision avoidance threshold (m) | `0.01` |

#### Topic Configuration Parameters (NEW)

All topic names are now configurable via ROS parameters for flexible integration:

| Parameter | Description | Default |
|-----------|-------------|---------|
| `cmd_vel_topic` | Velocity command output topic | `/neupan_cmd_vel` |
| `scan_topic` | Laser scan input topic | `/scan` |
| `plan_input_topic` | Global path input topic | `/plan` |
| `goal_topic` | Goal pose input topic | `/goal_pose` |
| `plan_output_topic` | Optimized trajectory output topic | `/neupan_plan` |
| `ref_state_topic` | Reference state output topic | `/neupan_ref_state` |
| `initial_path_topic` | Initial path visualization topic | `/neupan_initial_path` |
| `dune_markers_topic` | DUNE visualization markers topic | `/dune_point_markers` |
| `robot_marker_topic` | Robot footprint marker topic | `/robot_marker` |
| `nrmp_markers_topic` | NRMP visualization markers topic | `/nrmp_point_markers` |

For complete parameter documentation, see [config/sim_diff.yaml](config/sim_diff.yaml).

---

## 📚 Documentation

### ROS2 Topics

> **Note**: All topic names are configurable via ROS parameters. The topics listed below show default values. To customize topic names, see the [Topic Configuration Parameters](#topic-configuration-parameters-new) section.

#### Subscribed Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | Laser scan data for obstacle detection |
| `/plan` | `nav_msgs/Path` | Global path waypoints |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Navigation goal pose |

#### Published Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/neupan_cmd_vel` | `geometry_msgs/Twist` | Velocity commands (remapped to `/cmd_vel` by default) |
| `/neupan_plan` | `nav_msgs/Path` | Optimized trajectory |
| `/neupan_ref_state` | `nav_msgs/Path` | Reference state trajectory |
| `/neupan_initial_path` | `nav_msgs/Path` | Initial path visualization |
| `/dune_point_markers` | `visualization_msgs/MarkerArray` | DUNE network visualization |
| `/nrmp_point_markers` | `visualization_msgs/MarkerArray` | NRMP network visualization |
| `/robot_marker` | `visualization_msgs/Marker` | Robot footprint visualization |

### TF Frames

```
map
 └── odom (optional)
      └── base_link
           └── laser_link (if separate)
```

### Launch Files

| Launch File | Purpose | Usage |
|-------------|---------|-------|
| `sim_diff_launch.py` | Full simulation system | Simulation testing |
| `limo_diff_launch.py` | Limo robot deployment | Real robot navigation |
| `neupan_launch.py` | Standalone planner node | Custom integration |

---

## 🔗 Related Links

- **Original ROS1 Wrapper**: [NeuPAN-ROS](https://github.com/hanruihua/neupan_ros)
- **Core Algorithm Library**: [NeuPAN](https://github.com/hanruihua/neupan)
- **Research Paper**: [IEEE Transactions on Robotics (2025)](https://ieeexplore.ieee.org/document/10938329)
- **Project Website**: [NeuPAN Project Page](https://hanruihua.github.io/neupan_project/)
- **ROS2 Humble Documentation**: [docs.ros.org/en/humble](https://docs.ros.org/en/humble/)

---

## 📄 License

This project is licensed under the [GNU General Public License v3.0](LICENSE).

---

## 📖 Citation

If you find this code or paper helpful, please kindly star ⭐ this repository and cite our paper:

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

## 🤝 Acknowledgments

- **Original NeuPAN Algorithm**: Developed by [Ruihua HAN](https://github.com/hanruihua) and [SIAT-INVS](https://siat-invs.com/) Team.
- **ROS2 Integration**: Optimized and tested for AgileX Limo platform
- **Hardware Partner**: AgileX x Hive Matrix ([sales@hive-matrix.com](mailto:sales@hive-matrix.com))

---

## 📮 Contact & Support

For questions, issues, or collaboration opportunities:

- **Issues**: [GitHub Issues](https://github.com/KevinLADLee/neupan_ros2/issues)
- **Email**: chengyangli@connect.hku.hk
- **Original Project Maintainer**: hanrh@connect.hku.hk

---

**🎉 Happy Navigating with NeuPAN! 🤖**
