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

# NEW: Standalone NeuPAN simulation (no simulator)
ros2 launch neupan_ros2 simulation.launch.py

# Full simulation with ddr_minimal_sim environment
ros2 launch neupan_ros2 sim_complete.launch.py

# Or specify custom environment configuration
ros2 launch neupan_ros2 sim_complete.launch.py sim_env_config:=scenario_maze.yaml use_rviz:=true
```

**Available Environment Configs**:
- `sim_env_obs.yaml`: Basic obstacle environment
- `sim_env_obs_exam.yaml`: Complex obstacle course (default)

### 🤖 2. Real Robot Mode

**LIMO Robot** (differential drive):

```bash
# NEW: Simple command for LIMO
ros2 launch neupan_ros2 limo.launch.py

# With RViz disabled
ros2 launch neupan_ros2 limo.launch.py use_rviz:=false
```

**Ranger Robot** (ackermann drive):

```bash
# NEW: Includes pointcloud conversion and TF publishers
ros2 launch neupan_ros2 ranger.launch.py

# With RViz disabled
ros2 launch neupan_ros2 ranger.launch.py use_rviz:=false
```

> **Note**: This package is optimized for [AgileX Limo ROS2](https://www.agilex.ai/education/18) robot and Ranger Mini. For inquiries, contact our partner at sales@hive-matrix.com.

---

## 🎯 Configuration

### **NEW** Robot-Based Configuration Structure

**Version 0.4.0** introduces a cleaner, robot-centric configuration system. Each robot has its own dedicated directory containing all necessary configuration files and models.

#### Directory Structure

```
config/robots/
├── limo/                          # LIMO robot configuration
│   ├── robot.yaml                 # ROS node parameters
│   ├── planner.yaml               # NeuPAN planner parameters
│   └── models/
│       └── dune_model_5000.pth    # DUNE neural network model
│
├── ranger/                        # Ranger Mini robot configuration
│   ├── robot.yaml
│   ├── planner.yaml
│   └── models/
│       └── dune_model_5000.pth
│
├── simulation/                    # Simulation configuration
│   ├── robot.yaml
│   ├── planner.yaml
│   └── models/
│       └── dune_model_5000.pth
│
├── _template/                     # Template for new robots
│   ├── README.md                  # How to add new robots
│   ├── robot.yaml.template
│   ├── planner.yaml.template
│   └── models/README.md
│
└── README.md                      # Comprehensive documentation
```

#### Supported Robots

| Robot | Kinematics | Dimensions (L×W) | Wheelbase | Launch Command |
|-------|-----------|------------------|-----------|----------------|
| **LIMO** | Differential | 0.322m × 0.22m | N/A | `ros2 launch neupan_ros2 limo.launch.py` |
| **Ranger** | Ackermann | 0.720m × 0.500m | 0.500m | `ros2 launch neupan_ros2 ranger.launch.py` |
| **Simulation** | Differential | 0.322m × 0.22m | N/A | `ros2 launch neupan_ros2 simulation.launch.py` |

### Configuration Files

### Key Parameters

Each robot configuration consists of two YAML files:

#### 1. `robot.yaml` - ROS Integration Parameters

| Parameter | Description | Example |
|-----------|-------------|---------|
| `robot_type` | Robot identifier | `'limo'`, `'ranger'`, `'simulation'` |
| `robot_description` | Human-readable description | `'LIMO differential drive robot'` |
| `planner_config_file` | Planner config filename (relative) | `'planner.yaml'` |
| `dune_checkpoint_file` | DUNE model path (relative) | `'models/dune_model_5000.pth'` |
| `map_frame` | Global coordinate frame | `'map'` |
| `base_frame` | Robot base coordinate frame | `'base_link'` |
| `lidar_frame` | LiDAR coordinate frame | `'base_link'` |
| `scan_range_max` | Maximum laser scan distance (m) | `5.0` |
| `scan_range_min` | Minimum laser scan distance (m) | `0.01` |
| `control_frequency` | Planning loop frequency (Hz) | `50.0` |

#### 2. `planner.yaml` - NeuPAN Planner Parameters

| Parameter | Description | Example |
|-----------|-------------|---------|
| `receding` | MPC receding horizon | `8` - `10` |
| `step_time` | MPC time step (s) | `0.2` - `0.25` |
| `ref_speed` | Reference navigation speed (m/s) | `0.5` |
| `robot.kinematics` | Robot kinematics type | `'diff'` or `'acker'` |
| `robot.length` | Robot length (m) | `0.322` (LIMO), `0.720` (Ranger) |
| `robot.width` | Robot width (m) | `0.22` (LIMO), `0.500` (Ranger) |
| `robot.wheelbase` | Wheelbase for ackermann (m) | `0.500` (Ranger only) |
| `robot.max_speed` | Max linear/angular speed | `[0.5, 2.0]` |
| `ipath.curve_style` | Path generation style | `'line'` (diff), `'dubins'` (acker) |
| `collision_threshold` | Collision avoidance threshold (m) | `0.01` |

### Adding a New Robot

See [config/robots/README.md](config/robots/README.md) for detailed instructions.

**Quick Steps**:

1. **Copy the template**:
   ```bash
   cd config/robots
   cp -r _template my_robot
   mv my_robot/robot.yaml.template my_robot/robot.yaml
   mv my_robot/planner.yaml.template my_robot/planner.yaml
   ```

2. **Edit configuration files**:
   - Set `robot_type` and physical dimensions
   - Choose kinematics type and path style
   - Add or symlink DUNE model

3. **Create launch file**:
   ```bash
   cp launch/limo.launch.py launch/my_robot.launch.py
   # Edit robot_config_dir path in the file
   ```

4. **Build and test**:
   ```bash
   colcon build --packages-select neupan_ros2
   ros2 launch neupan_ros2 my_robot.launch.py
   ```

#### Topic Configuration Parameters

All topic names are configurable via ROS parameters for flexible integration:

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

#### Visualization Control Parameters (NEW in v0.3.0)

Control visualization markers to optimize performance on low-power platforms:

| Parameter | Description | Default (Sim) | Default (Real) |
|-----------|-------------|---------------|----------------|
| `enable_visualization` | Master switch for all visualization markers | `false` | `true` |
| `enable_dune_markers` | Enable DUNE point cloud visualization | `true` | `true` |
| `enable_nrmp_markers` | Enable NRMP point cloud visualization | `true` | `true` |
| `enable_robot_marker` | Enable robot footprint visualization | `true` | `true` |

**Performance Impact:**
- Visualization disabled (`enable_visualization: false`): ~5-10% CPU reduction on embedded platforms
- Selective markers: Disable DUNE/NRMP to keep only robot footprint for minimal overhead

**Example Configuration:**
```yaml
# Minimal visualization for embedded platforms
enable_visualization: true
enable_dune_markers: false    # Disable CPU-intensive point clouds
enable_nrmp_markers: false
enable_robot_marker: true     # Keep robot visualization only
```

#### Control Loop Parameters (NEW in v0.3.0)

| Parameter | Description | Default | Recommended Range |
|-----------|-------------|---------|-------------------|
| `control_frequency` | Planning and control loop frequency (Hz) | `50.0` | `10.0 - 100.0` |

**Tuning Guide:**
- **High-speed robots** (>1 m/s): 50-100 Hz for responsive control
- **Slow robots** (<0.5 m/s): 20-30 Hz sufficient, saves CPU
- **Embedded platforms**: Start at 30 Hz, increase if needed

For complete parameter documentation, see the robot configuration files in [config/robots/](config/robots/).

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
| `simulation.launch.py` | Standalone NeuPAN simulation | Testing without external simulator |
| `sim_complete.launch.py` | Full simulation system with ddr_minimal_sim | Complete simulation testing |
| `limo.launch.py` | LIMO robot deployment | LIMO differential drive robot |
| `ranger.launch.py` | Ranger robot deployment | Ranger ackermann robot |

---

## 🏗️ Architecture

### Thread Safety

NeuPAN ROS2 uses a thread-safe multi-threaded architecture for optimal performance on multi-core systems:

**Executor:**
- **MultiThreadedExecutor**: Enables concurrent callback processing for better CPU utilization

**Callback Groups:**
- **Control Timer** (`MutuallyExclusiveCallbackGroup`):
  - Runs control loop (`run()`) in isolation
  - Prevents concurrent planner execution
  - Ensures deterministic planning behavior

- **Sensor Subscriptions** (`ReentrantCallbackGroup`):
  - Scan, path, and goal callbacks can run concurrently
  - Optimizes sensor data processing on multi-core systems
  - Reduces callback latency

**State Protection:**
- All shared state (`robot_state`, `obstacle_points`, planner state) protected by `threading.Lock`
- Fine-grained locking minimizes lock contention (75-95% reduction vs coarse locking)
- Safe for concurrent sensor updates during planning

**Benefits:**
- ✅ Thread-safe on multi-core systems
- ✅ No race conditions in sensor processing
- ✅ Optimal CPU utilization
- ✅ Improved real-time responsiveness

### Modular Design

The package follows a modular architecture for better maintainability:

**neupan_node.py** (main node ~800 lines):
- ROS2 integration layer
- Subscription/publisher management
- Control loop coordination
- Integrates planner with ROS2 ecosystem

**visualization_manager.py** (visualization module ~322 lines):
- Optional RViz marker generation
- Independent from planning logic
- Thread-safe visualization publishing
- Handles DUNE, NRMP, and robot footprint markers
- Can be disabled for zero overhead on embedded platforms

**utils.py** (utilities ~51 lines):
- Coordinate transformation helpers
- `yaw_to_quat()`: Convert yaw angle to quaternion
- `quat_to_yaw()`: Extract yaw from quaternion
- Shared utility functions

**Benefits:**
- ✅ Clean separation of concerns
- ✅ Easier to maintain and extend
- ✅ Optional visualization reduces CPU load
- ✅ Reusable utility functions

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
