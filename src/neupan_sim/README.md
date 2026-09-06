# neupan_sim

For a convex polygon robot, set the ROS parameter `robot_vertices` to the flat
list `[x1, y1, x2, y2, ...]` in metres in the robot state frame. For example, the
planner's trapezoid `robot.vertices` corresponds to
`[-0.8, -1.0, -1.8, 1.0, 1.8, 1.0, 0.8, -1.0]`. Use the same ordered vertices
in the planner and simulator. Nonempty vertices override `robot_size`; the
default empty list keeps the existing rectangular robot. Collision checks
against circles, segments and world bounds, and the robot marker, use this
polygon. This changes geometry only; the simulated motion remains differential drive.

`neupan_sim` is an `ament_cmake` package for closed-loop NeuPAN demonstrations
and integration tests. It provides a deterministic differential-drive simulator
with continuous collision geometry; it is not a general-purpose robotics
simulator. The package is tested with ROS 2 Humble and Jazzy.

## Quick start

From the repository root:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
./build.sh
source install/setup.bash
ros2 launch neupan_sim quick_start.launch.py
```

The launch file starts the `neupan_sim` and `neupan_node` nodes and RViz. For a
headless run:

```bash
ros2 launch neupan_sim quick_start.launch.py rviz:=false
```

Run the deterministic pass/fail scenario with:

```bash
ros2 launch neupan_sim verify.launch.py
```

## Node

### `neupan_sim`

- Executable: `neupan_sim_node`
- Default node name: `neupan_sim`

#### Parameters

The installed parameter files
[`quick_start.yaml`](config/quick_start.yaml) and
[`verify.yaml`](config/verify.yaml) override some defaults listed below.

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `scenario_name` | `string` | `simulation` | Scenario identifier used in log and diagnostic messages |
| `physics_rate` | `double` | `100.0` | Simulation, TF, and odometry update frequency in Hz |
| `sensor_rate` | `double` | `20.0` | LaserScan and PointCloud2 publication frequency in Hz |
| `diagnostics_rate` | `double` | `10.0` | Diagnostic and marker publication frequency in Hz |
| `integration_substeps` | `integer` | `2` | Integration steps performed during each simulation update |
| `initial_pose` | `double array` | `[0.0, 0.0, 0.0]` | Initial `[x, y, yaw]` in `map`, in metres and radians |
| `robot_size` | `double array` | `[0.5, 0.5]` | Rectangular `[length, width]` in metres |
| `speed_limits` | `double array` | `[2.0, 1.5]` | Maximum `[linear velocity, angular velocity]` in m/s and rad/s |
| `acceleration_limits` | `double array` | `[2.0, 2.0]` | Maximum `[linear acceleration, angular acceleration]` in m/s² and rad/s² |
| `command_timeout` | `double` | `0.25` | Command age after which the target velocity is set to zero, in seconds |
| `goal_tolerance` | `double` | `0.12` | Goal-distance threshold for `goal_reached`, in metres |
| `simulation_timeout` | `double` | `30.0` | Scenario time limit in seconds |
| `path_waypoints` | `double array` | `[0, 0, 6, 0]` | Flattened `[x, y]` pairs; at least two pairs are required |
| `path_spacing` | `double` | `0.1` | Spacing of poses in the published path, in metres |
| `world_bounds` | `double array` | `[-2, 8, -4, 4]` | `[min_x, max_x, min_y, max_y]` in metres |
| `static_circles` | `double array` | `[]` | Repeated `[x, y, radius]` values |
| `dynamic_circles` | `double array` | `[]` | Repeated `[x, y, radius, vx, vy]` values in the `map` frame |
| `segments` | `double array` | `[]` | Repeated `[x1, y1, x2, y2]` line segments |
| `laser_pose` | `double array` | `[0.15, 0.0, 0.0]` | `[x, y, yaw]` transform from `base_link` to `laser_link` |
| `scan_angle_min` | `double` | `-pi` | First LaserScan beam angle in radians |
| `scan_angle_max` | `double` | `pi` | Last LaserScan beam angle in radians |
| `scan_angle_increment` | `double` | `pi / 180` | LaserScan angular resolution in radians |
| `scan_range_min` | `double` | `0.05` | Minimum ray-cast range in metres |
| `scan_range_max` | `double` | `8.0` | Maximum ray-cast range in metres |

`sensor_rate` and `diagnostics_rate` must not exceed `physics_rate`. Robot
dimensions and actuator limits should match the `neupan_core` planner
configuration used by `neupan_node`.

#### Subscribed topics

| Name | Message type | QoS | Description |
| --- | --- | --- | --- |
| `neupan_cmd_vel` | `geometry_msgs/msg/Twist` | Reliable / Volatile / Keep Last (1) | Target `linear.x` and `angular.z` for the differential-drive model |

#### Published topics

| Name | Message type | QoS | Description |
| --- | --- | --- | --- |
| `odom` | `nav_msgs/msg/Odometry` | Reliable / Volatile / Keep Last (10) | Robot odometry; frame `map`, child frame `base_link` |
| `scan` | `sensor_msgs/msg/LaserScan` | Reliable / Volatile / Keep Last (5) | Laser scan in `laser_link` |
| `obstacles` | `sensor_msgs/msg/PointCloud2` | Reliable / Volatile / Keep Last (5) | XYZIV obstacle cloud in `laser_link` |
| `initial_path` | `nav_msgs/msg/Path` | Reliable / Transient Local / Keep Last (1) | Dense reference path in `map` |
| `neupan_sim/markers` | `visualization_msgs/msg/MarkerArray` | Reliable / Transient Local / Keep Last (1) | Scenario and robot visualization markers |
| `neupan_sim/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Reliable / Transient Local / Keep Last (1) | Scenario result and metrics |

The XYZIV fields are `x`, `y`, `z`, `intensity`, `vx`, and `vy`. Position and
velocity are expressed in `laser_link`; `intensity=1` denotes a dynamic circle
and `intensity=0` denotes static geometry.

#### Services and actions

The node defines no application-specific services or actions. The standard ROS
2 parameter services created by `rclcpp::Node` remain available.

#### Published transforms

- `map -> base_link` at `physics_rate`
- `base_link -> laser_link` at `physics_rate`

## Launch files

| Launch file | Purpose | `rviz` default | `planning_frame` default |
| --- | --- | --- | --- |
| `quick_start.launch.py` | Warehouse demonstration | `true` | `map` |
| `verify.launch.py` | Deterministic integration test | `false` | `map` |

Both launch arguments are configurable. `planning_frame` is passed to
`neupan_node`; changing it requires a corresponding TF connection to `map`.

## Integration-test result

The `neupan_sim/diagnostics` topic reports `running`, `goal_reached`,
`collision`, or `timed_out`. The diagnostic key/value pairs include elapsed
time, goal distance, path length, clearance, actual velocity, command age, and
LaserScan hit count.

All scenario geometry uses `map`, while sensor messages use `laser_link`. See
the [coordinate-frame contract](../../docs/coordinate_frames.md) and
[dynamic-obstacle message contract](../../docs/dynamic_obstacles_CN.md).

## Parallel shape validation

Run the shape × scenario matrix with a single command (a nonzero exit means
at least one unexpected result or an infrastructure failure):

```bash
ros2 run neupan_sim neupan_validate --output /tmp/neupan-validation-1
# Optional single RViz, with scene copies laid out in a grid:
ros2 run neupan_sim neupan_validate --output /tmp/neupan-validation-2 --rviz
# Launch equivalent:
ros2 launch neupan_sim validation.launch.py output:=/tmp/neupan-validation-3 rviz:=true
```

The default [suite](config/validation.yaml) covers square, Scout Mini rectangle,
and the original NeuPAN trapezoid in open, static-obstacle, corridor, and moving
obstacle scenes (12 cases). Each case owns its planner/model, simulated robot,
topics, and TF frames. Geometry and actuator limits are derived from the planner
configuration, including rectangle axle offsets. Copies do not sense or collide
with each other. Display-only static transforms place their maps under
`validation_world` without changing their local physics.

Simulators start paused and continue publishing stationary TF, sensors and paths.
The supervisor waits for every planner to report a successful solve, then calls
each simulator's `start` Trigger service. The default standalone simulator still
starts immediately. `world_frame`, `base_frame`, `laser_frame`, and `start_paused`
are optional simulator parameters; defaults preserve the original single-robot
launches.

Use `--suite FILE` for another matrix or repeat `--case square_open` to select
specific cases. Profiles reference planner/model files relative to the suite,
or via `package://PACKAGE/path`. The trapezoid checkpoint is exported from
upstream `example/model/polygon_robot/model_5000.pth`; its provenance and numerical
verification are recorded in `neupan_core/models/diff_polygon.bin.json`.

The output directory contains resolved per-case YAMLs, child process logs,
`validation.rviz`, `summary.json`, and `summary.csv`. Existing results are never
overwritten. Summary rows contain the terminal result, elapsed simulated time,
path length, goal distance, minimum exact geometric clearance and the number of
unsolved diagnostic samples. These samples are not unique solver-failure events.
The supervisor stops all children on completion, startup failure, interruption,
or wall-clock timeout. Concurrent CPU load can affect ROS scheduling; simulated
time and trajectory metrics should be compared separately from wall time.
