# Installation

NeuPAN ROS 2 supports these tested combinations:

| ROS 2 | Ubuntu |
| --- | --- |
| Humble | 22.04 (Jammy) |
| Jazzy | 24.04 (Noble) |

Install the matching ROS 2 distribution before continuing. The workspace does
not use `rosdep`.

## Clone and install dependencies

```bash
git clone https://github.com/KevinLADLee/neupan_ros2.git
cd neupan_ros2

ROS_DISTRO=humble
source /opt/ros/$ROS_DISTRO/setup.bash
./install_deps.sh
```

Set `ROS_DISTRO` to `humble` or `jazzy`. `install_deps.sh` uses that value to
install the required Ubuntu and ROS 2 packages with apt.

The authoritative apt package list is maintained in
[`install_deps.sh`](../install_deps.sh) to prevent the script, documentation,
and CI instructions from drifting apart.

OSQP v1.0.0, osqp-eigen v0.11.2, and QDLDL v0.1.8 are already included under
`thirdparty/` as the `neupan_solver_vendor` package; the build does not download
or select a system solver.

## Build and run

```bash
./build.sh
source install/setup.bash
ros2 launch neupan_sim quick_start.launch.py
```

`build.sh` validates the ROS distribution, native headers, and bundled solver
sources before building. Build and run the test suite with:

```bash
./build.sh --test
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

For a package-specific build, use `--package`, for example
`./build.sh --package neupan_ros` or
`./build.sh --test --package neupan_core`. The former positional syntax remains
supported for compatibility.

### Use inside an existing workspace

The repository can also be cloned below an existing source space:

```bash
cd ~/ros2_ws/src
git clone https://github.com/KevinLADLee/neupan_ros2.git
cd ..
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Colcon discovers `neupan_solver_vendor` together with the three first-party
packages and builds them in dependency order. No repository-local setup or
third-party build command is required.

## Troubleshooting

- `ROS 2 is not sourced`: source the matching file under `/opt/ros` and retry.
- Missing Eigen or yaml-cpp headers: rerun `./install_deps.sh`.
- RViz is unavailable: run the demo with `rviz:=false`.
- Stale build after switching ROS distributions: use a fresh workspace checkout
  or remove the generated `build`, `install`, and `log` directories before
  rebuilding.
