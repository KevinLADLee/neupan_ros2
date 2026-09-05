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

source /opt/ros/humble/setup.bash  # or /opt/ros/jazzy/setup.bash
./install_deps.sh
```

`install_deps.sh` uses `$ROS_DISTRO` to install the required Ubuntu and ROS 2
packages with apt. It accepts only `humble` and `jazzy`.

<details>
<summary>Manual apt command</summary>

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git libeigen3-dev libgtest-dev libyaml-cpp-dev \
  python3-colcon-common-extensions \
  ros-$ROS_DISTRO-ament-cmake ros-$ROS_DISTRO-ament-cmake-gtest \
  ros-$ROS_DISTRO-ament-index-python \
  ros-$ROS_DISTRO-diagnostic-msgs ros-$ROS_DISTRO-geometry-msgs \
  ros-$ROS_DISTRO-launch ros-$ROS_DISTRO-launch-ros \
  ros-$ROS_DISTRO-nav-msgs ros-$ROS_DISTRO-rclcpp \
  ros-$ROS_DISTRO-rclcpp-components ros-$ROS_DISTRO-ros2launch \
  ros-$ROS_DISTRO-rviz2 ros-$ROS_DISTRO-sensor-msgs \
  ros-$ROS_DISTRO-std-msgs ros-$ROS_DISTRO-tf2 ros-$ROS_DISTRO-tf2-ros \
  ros-$ROS_DISTRO-visualization-msgs
```

</details>

OSQP v1.0.0, osqp-eigen v0.11.2, and QDLDL v0.1.8 are already included under
`thirdparty/`; the build does not download or select a system solver.

## Build and run

```bash
./setup.sh
./build.sh
source install/setup.bash
ros2 launch neupan_sim quick_start.launch.py
```

`setup.sh` validates the sourced ROS environment and native headers. Build and
run the test suite with:

```bash
./build.sh test
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

For a package-specific build, pass its name to `build.sh`, for example
`./build.sh neupan_ros` or `./build.sh test neupan_core`.

## Troubleshooting

- `ROS 2 is not sourced`: source the matching file under `/opt/ros` and retry.
- Missing Eigen or yaml-cpp headers: rerun `./install_deps.sh`.
- RViz is unavailable: run the demo with `rviz:=false`.
- Stale build after switching ROS distributions: use a fresh workspace checkout
  or remove the generated `build`, `install`, and `log` directories before
  rebuilding.
