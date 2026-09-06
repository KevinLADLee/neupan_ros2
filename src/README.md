# ROS 2 packages

The `src/` space contains three first-party C++ ROS 2 packages. Colcon also
discovers the repository's `neupan_solver_vendor` support package under
`../thirdparty`, for a total of four workspace packages.

The same source tree is CI-tested with ROS 2 Humble/Ubuntu 22.04 and ROS 2
Jazzy/Ubuntu 24.04. Dependency installation is explicit and does not require
rosdep; see the repository-level [Quick Start](../README.md#quick-start).

| Package | Build type | Installed targets | Documentation |
| --- | --- | --- | --- |
| `neupan_core` | `cmake` | CMake target `neupan::neupan` | [C++ API and planner configuration](neupan_core/README.md) |
| `neupan_ros` | `ament_cmake` | Executables `neupan_node`, `astar_global_node`; component `neupan_ros::NeuPANNode` | [Nodes, parameters, topics, and TF](neupan_ros/README.md) |
| `neupan_sim` | `ament_cmake` | Executable `neupan_sim_node`; library `neupan_simulation` | [Node, parameters, topics, and launch files](neupan_sim/README.md) |

## Quick Start

Run from the repository root:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
./build.sh
source install/setup.bash
ros2 launch neupan_sim quick_start.launch.py
```

Build or test a selected package with:

```bash
./build.sh --package neupan_ros
./build.sh --test --package neupan_core
colcon test --packages-select neupan_core
colcon test-result --verbose
```

`neupan_core` uses the plain CMake build type and has no ROS graph entities.
It declares `neupan_solver_vendor` as a workspace dependency, so a normal
`colcon build` builds the bundled solver first even when this repository is
cloned under another workspace's `src/` directory.
`neupan_ros` links against `neupan_core` and provides the ROS 2 nodes.
`neupan_sim` has an execution dependency on `neupan_ros` because its launch
files start the local-planner node for closed-loop tests.

Offline Python training lives in [`../training`](../training/README.md). It is
not a ROS 2 package and is not part of the colcon source space.
