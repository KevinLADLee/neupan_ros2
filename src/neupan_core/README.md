# neupan_core

`neupan_core` is a plain-CMake ROS 2 package that exports the ROS-independent
C++17 NeuPAN planning library. DUNE inference uses Eigen, PAN couples
perception and planning, and NRMP is assembled as an OSQP quadratic program.
It is built in both the ROS 2 Humble and Jazzy CI jobs.

## Quick Start

From the workspace root, build and run the core tests:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
./build.sh --test --package neupan_core
colcon test --packages-select neupan_core --event-handlers console_direct+
colcon test-result --verbose
```

Downstream CMake code uses the exported target:

```cmake
find_package(neupan_core REQUIRED)
target_link_libraries(your_target PRIVATE neupan::neupan)
```

## C++ API

The main entry point is `neupan::NeuPANPlanner` from
`<neupan/neupan_planner.hpp>`:

```cpp
auto planner = neupan::NeuPANPlanner::fromYaml("planner.yaml", "model.bin");
neupan::NeuPANPlanner::Info info;
neupan::Vec3 state(x, y, yaw);
neupan::Mat2X points(2, obstacle_count);
neupan::Mat2X velocities(2, obstacle_count);
const neupan::Vec2 command = planner.forward(state, points, velocities, info);
```

| Input / output | Shape or type | Contract |
| --- | --- | --- |
| `state` | `Vec3` | `[x, y, yaw]` in metres/radians |
| `points` | `2 x N` | Obstacle XY columns in the common world frame; `N` may be zero |
| `velocities` | `2 x N` | Velocity paired column-wise with `points`, in m/s; an empty `2 x 0` matrix means static points |
| return value | `Vec2` | `[linear velocity, angular velocity]` in m/s and rad/s |
| `Info::arrive` | `bool` | Initial-path goal has been reached |
| `Info::stop` | `bool` | Minimum DUNE distance is below `collision_threshold` |
| `Info::solved` | `bool` | At least one PAN/NRMP solve converged this cycle |
| `Info::solver_status` | `int` | Raw OsqpEigen solver status |
| `Info::min_distance` | `double` | Minimum stage-0 DUNE distance; infinity when no point was evaluated |
| `Info::opt_s`, `Info::opt_u` | `3 x (T+1)`, `2 x T` | Optimized states and controls |
| `Info::ref_s` | `3 x (T+1)` | Reference states |
| `Info::dune_points`, `Info::nrmp_points` | `2 x N` | Points retained by the DUNE/NRMP stages for diagnostics or visualization |

Reference paths can be supplied with `setInitialPath`, `setWaypoints`, or
`updateInitialPathFromGoal`. `replaceInitialPath` preserves progress during a
global-path update, while `reset` clears planner progress and warm-start state.
`setInitialPath` preserves the supplied samples and expects a dense path. Use
`setWaypoints` to generate a path from sparse waypoints.

The core performs no coordinate transforms. State, path, obstacle positions,
and velocities must use one right-handed planar frame. See the
[coordinate-frame contract](../../docs/coordinate_frames.md).

## Planner configuration

These are `neupan_core` configuration keys, not ROS 2 node parameters. Defaults
below are used when a YAML key is omitted. The shipped examples are
[`planner.yaml`](../neupan_ros/config/planner.yaml) and
[`sentry_diff.yaml`](../neupan_ros/config/sentry_diff.yaml).

### MPC and robot

| Key | Default | Meaning |
| --- | --- | --- |
| `receding` | `10` | MPC horizon length `T`; must be at least 1 |
| `step_time` | `0.1` | Horizon step in seconds; must be positive |
| `ref_speed` | `4.0` | Reference linear speed in m/s |
| `collision_threshold` | `0.1` | Emergency-stop DUNE distance in metres; must be positive |
| `robot.kinematics` | `diff` | Robot model; only `diff` is currently supported |
| `robot.max_speed` | `[8.0, 1.0]` | Maximum `[linear m/s, angular rad/s]` |
| `robot.max_acce` | `[8.0, 3.0]` | Maximum `[linear m/s², angular rad/s²]` |
| `robot.length` | `1.6` | Rectangular footprint length in metres |
| `robot.width` | `2.0` | Rectangular footprint width in metres |
| `robot.wheelbase` | `0.0` | Base-origin longitudinal offset used to construct the footprint |

### Initial path

| Key | Default | Meaning |
| --- | --- | --- |
| `ipath.curve_style` | `line` | Curve type; only `line` is currently supported |
| `ipath.waypoints` | empty | Optional list of `[x, y, yaw]` waypoints |
| `ipath.loop` | `false` | Treat the configured path as a loop |
| `ipath.interval` | `-1.0` | Path sample spacing; a negative value selects `step_time * ref_speed` |
| `ipath.arrive_threshold` | `0.1` | Position tolerance at the path end in metres |
| `ipath.close_threshold` | `0.1` | Distance used while locating progress on the path |
| `ipath.ind_range` | `10` | Forward search range in path indices |
| `ipath.arrive_index_threshold` | `1` | Remaining-index threshold used by arrival detection |

### PAN, DUNE, and NRMP

| Key | Default | Meaning |
| --- | --- | --- |
| `pan.iter_num` | `2` | Maximum alternating PAN iterations; must be at least 1 |
| `pan.dune_max_num` | `100` | Maximum obstacle points passed to DUNE |
| `pan.nrmp_max_num` | `10` | Maximum obstacle constraints per horizon stage |
| `pan.iter_threshold` | `0.1` | PAN convergence threshold |
| `pan.dune_checkpoint` | empty | NPTF `.bin` model path; can be overridden by `fromYaml`'s second argument |
| `adjust.q_s` | `[1, 1, 1]` | State tracking weight; accepts one scalar or three values |
| `adjust.p_u` | `1.0` | Control tracking weight |
| `adjust.eta` | `10.0` | Clearance reward weight |
| `adjust.d_max` | `1.0` | Maximum optimized clearance in metres |
| `adjust.d_min` | `0.1` | Minimum optimized clearance in metres; cannot be negative |
| `adjust.ro_obs` | `400.0` | Obstacle-contact penalty weight |
| `adjust.bk` | `0.1` | Obstacle constraint offset |

`pan.nrmp_max_num` and `pan.dune_max_num` must be non-negative. A model is
required when both values are positive. Setting either limit to zero enables
navigation-only behavior and removes the DUNE model-file dependency.

The NPTF model embeds the trained footprint matrices. Loading fails when those
matrices do not match the configured robot footprint; retrain and export a model
for the new geometry instead of bypassing that check. See
[training/README.md](../../training/README.md).

## Build options

| CMake option | Default | Effect |
| --- | --- | --- |
| `BUILD_TESTING` | `OFF` | Build equivalence, DUNE, NRMP, and replay tests |
| `NEUPAN_BUILD_TOOLS` | `OFF` | Build `neupan_bench` and `neupan_ab_safety_margin` |
| `NEUPAN_NATIVE` | `OFF` | Enable `-march=native -mtune=native`; bundled osqp-eigen must use matching flags |

## ROS graph entities

This package installs a C++ library and model resources. It does not install a
ROS 2 node executable and therefore has no node parameters, topics, services,
actions, or TF frames. ROS graph integration is provided by `neupan_ros`.
