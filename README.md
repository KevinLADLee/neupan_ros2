# NeuPAN ROS 2

CPU-oriented NeuPAN workspace with a native C++ deployment path.

See the [algorithm equivalence note](docs/algorithm_equivalence_CN.md) for the
equation-by-equation derivation from the original NeuPAN implementation, the
verified scope, and remaining differences.

## Repository layout

- `src/neupan_core`: ROS-independent C++ implementation of DUNE, PAN and NRMP.
- `src/neupan_ros`: native `rclcpp` node and ROS message preprocessing.
- `src/neupan_sim`: minimal single-process closed-loop verification simulator.
- `training`: offline Python package for DUNE training and NPTF export.

The planner and simulator executables have no rclpy, NumPy or PyTorch runtime
dependency. Standard ROS 2 Python launch files are still used for orchestration.
Superseded Python ROS runtime and multi-node simulator packages are not part of
the repository or colcon workspace.

## Runtime dependencies

- ROS 2 Humble or newer
- Eigen3
- yaml-cpp

OSQP v1.0.0 and osqp-eigen v0.11.2 are vendored in `thirdparty/`. The build
script compiles them as static CPU libraries before invoking colcon, so a
normal build does not download or discover a system solver installation.

## Build

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
./setup.sh
./build.sh
source install/setup.bash
```

Run the visual quick start. It launches the richer shared-warehouse scenario,
native NeuPAN node and the prepared RViz view:

```bash
ros2 launch neupan_sim quick_start.launch.py
```

It includes four shelf islands, a winding reference route, static facilities
and three moving obstacles crossing the open aisles. For a headless run, append
`rviz:=false`.

Run the small deterministic regression scenario separately with:

```bash
ros2 launch neupan_sim verify.launch.py
```

The ROS layer accepts LaserScan and XYZ, XYZI or XYZIV PointCloud2 inputs. The
simulator publishes `/scan`, `/obstacles`, `/initial_path`, `/odom`
and TF. `/obstacles` uses `x/y/z/intensity/vx/vy`. See the
[dynamic-obstacle interface](docs/dynamic_obstacles_CN.md) for its coordinate,
timestamp, and completeness contract.

All spatial inputs follow the strict
[coordinate-frame contract](docs/coordinate_frames.md); sensor frames are
transformed at their message timestamps, while persistent paths and goals are
continuously projected into the configurable local `planning_frame`.
See the [ROS interface reference](src/neupan_ros/README.md) for all topics,
parameters, QoS choices and frame behavior.

## Offline training

Training has a separate environment and does not enter the colcon build:

```bash
python -m venv .venv-training
. .venv-training/bin/activate
pip install -e ./training

neupan-train --output runs/diff --length 0.5 --width 0.5
neupan-export runs/diff/model_5000.pth src/neupan_core/models/diff.bin \
  --length 0.5 --width 0.5
```

See [training/README.md](training/README.md) for the model export contract.

## Current scope

The new C++ path currently targets differential-drive robots with line-style
initial paths and per-point constant-velocity obstacle prediction. Ackermann,
omni and Dubins/loop paths remain migration work.

[中文说明](README_CN.md)
