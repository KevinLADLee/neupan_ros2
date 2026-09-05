# Continuous integration

[`ros2-ci.yml`](workflows/ros2-ci.yml) builds and tests the native workspace on
every push and pull request to `main` or `master`, and can also be run manually.

## Compatibility matrix

| ROS 2 | Ubuntu | Container |
| --- | --- | --- |
| Humble | 22.04 Jammy | `ros:humble-ros-base-jammy` |
| Jazzy | 24.04 Noble | `ros:jazzy-ros-base-noble` |

Both matrix jobs install the same explicit apt dependency list. CI does not run
rosdep, so a missing system dependency cannot be hidden by automatic package
resolution.

## Checks

Each matrix job:

1. Installs the compiler, CMake, Eigen3, yaml-cpp, GTest, and colcon.
2. Confirms the colcon source space contains exactly `neupan_core`, `neupan_ros`,
   and `neupan_sim`.
3. Builds the vendored solver and all three packages with tests enabled.
4. Runs every package test and prints verbose results.
5. Byte-compiles the offline training project and parses its `pyproject.toml`.

The solver sources are vendored under `thirdparty/`, so CI does not download
OSQP, osqp-eigen, or QDLDL during the build.

## Local equivalent

Install dependencies using the repository [Quick Start](../README.md#quick-start),
then run:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
./build.sh test
colcon test --event-handlers console_direct+
colcon test-result --verbose
python3 -m compileall -q training/src
python3 -c "import pathlib, tomllib; tomllib.loads(pathlib.Path('training/pyproject.toml').read_text())"
```
