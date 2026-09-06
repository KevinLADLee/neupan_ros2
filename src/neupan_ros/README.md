# neupan_ros

Differential-drive robots support rectangles and single convex polygons via
`robot.vertices` in the planner YAML; see [polygon_diff.yaml](config/polygon_diff.yaml).
Use the same geometry for [DUNE training](../../training/README.md), and supply
the resulting model via `dune_checkpoint`. RViz `robot_marker` displays the
actual polygon in the robot state frame, including asymmetric footprints.
Omni/Ackermann kinematics and compound or concave robot shapes are not supported.

After training, launch with explicit geometry and matching weights:

```bash
ros2 launch neupan_ros neupan.launch.py \
  config_file:=$PWD/src/neupan_ros/config/polygon_diff.yaml \
  dune_checkpoint:=$PWD/training/runs/polygon/model.bin
```

`neupan_ros` is an `ament_cmake` package that exposes `neupan_core` through ROS
2. It installs the `neupan_node` local planner, a small `astar_global_node` used
by the example launch file, configuration files, and a composable node.
It is tested with ROS 2 Humble and Jazzy.

All topic names documented below are relative names and support ROS 2 namespace
and remapping rules.

## Quick start

Run the closed-loop example from the repository root:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
./build.sh
source install/setup.bash
ros2 launch neupan_sim quick_start.launch.py
```

Start the package launch file for robot integration:

```bash
ros2 launch neupan_ros neupan.launch.py \
  planning_frame:=odom base_frame:=base_link
```

`neupan.launch.py` starts both package executables and remaps
`neupan_cmd_vel` to `cmd_vel`. A deployment must provide the required TF tree,
obstacle observations, occupancy grid, and goal. If another global planner
publishes `initial_path`, run `neupan_node` without `astar_global_node`.

## Nodes

### `neupan_node`

- Executable: `neupan_node`
- Default node name: `neupan_node`
- Composable node: `neupan_ros::NeuPANNode`

The node obtains the robot pose from TF, converts obstacle and reference-path
messages into `planning_frame`, calls `neupan_core`, and publishes the local
velocity command and planner state.

#### Parameters

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `config_file` | `string` | `""` (required) | Path to the `neupan_core` planner configuration file |
| `dune_checkpoint` | `string` | `""` | Overrides `pan.dune_checkpoint` from `config_file` |
| `planning_frame` | `string` | `odom` | Common coordinate frame for planning, paths, and markers |
| `base_frame` | `string` | `base_link` | Robot body frame used for the TF pose lookup |
| `control_rate` | `double` | `50.0` | Planning timer frequency in Hz |
| `pose_timeout` | `double` | `0.5` | Maximum robot TF age in seconds; must be finite and positive |
| `include_initial_path_direction` | `bool` | `false` | Use the orientation of each input path pose instead of the XY tangent |
| `obstacle_source` | `string` | `auto` | `scan`, `pointcloud`, or `auto`; `auto` prefers a fresh point cloud |
| `scan_timeout` | `double` | `0.5` | Maximum LaserScan age in seconds |
| `pointcloud_timeout` | `double` | `0.5` | Maximum PointCloud2 age in seconds |
| `compensate_obstacle_latency` | `bool` | `true` | Propagate dynamic points by velocity times observation age |
| `scan_angle_min` | `double` | `-3.14` | Minimum accepted LaserScan angle in radians |
| `scan_angle_max` | `double` | `3.14` | Maximum accepted LaserScan angle in radians |
| `scan_range_min` | `double` | `0.0` | Minimum accepted LaserScan range in metres |
| `scan_range_max` | `double` | `5.0` | Maximum accepted LaserScan range in metres |
| `scan_downsample` | `integer` | `1` | Keep every Nth valid LaserScan beam; must be at least 1 |
| `flip_angle` | `bool` | `false` | Negate LaserScan beam angles before XY conversion |
| `solver_fail_grace` | `integer` | `5` | Consecutive unsolved cycles allowed before publishing zero velocity |
| `stall_speed` | `double` | `0.02` | Command-component threshold used for stall detection |
| `stall_timeout` | `double` | `3.0` | Zero-command duration before reporting a stall, in seconds |
| `marker_size` | `double` | `0.05` | DUNE and NRMP marker diameter in metres |
| `marker_z` | `double` | `1.0` | Marker height above the planning plane in metres |

The keys inside `config_file` configure the C++ planner; they are not ROS 2 node
parameters. See the [`neupan_core` planner configuration](../neupan_core/README.md#planner-configuration).

The robot transform `planning_frame <- base_frame` must stay fresh. If its
timestamp is older than `pose_timeout` (or more than 50 ms in the future), the
node publishes zero velocity and a warning diagnostic on every control cycle.
Tracking resumes automatically once a fresh robot transform is available.
Age uses the node's ROS clock, including simulated time when enabled. This
check does not expire static reference-frame transforms. For example:

```bash
ros2 launch neupan_ros neupan.launch.py pose_timeout:=0.3
```

#### Subscribed topics

| Name | Message type | QoS | Description |
| --- | --- | --- | --- |
| `scan` | `sensor_msgs/msg/LaserScan` | `SensorDataQoS` | Created when `obstacle_source` is `scan` or `auto` |
| `obstacles` | `sensor_msgs/msg/PointCloud2` | `SensorDataQoS` | Created when `obstacle_source` is `pointcloud` or `auto`; accepts XYZ, XYZI, or XYZIV |
| `initial_path` | `nav_msgs/msg/Path` | Reliable / Volatile / Keep Last (10) | Dense path containing at least two poses |
| `neupan_waypoints` | `nav_msgs/msg/Path` | Reliable / Volatile / Keep Last (10) | Sparse waypoints; the current robot pose is prepended |
| `neupan_goal` | `geometry_msgs/msg/PoseStamped` | Reliable / Volatile / Keep Last (10) | Goal used to generate a straight reference path |

The most recently received reference-path input is active. Its original frame is
retained and the path is transformed into `planning_frame` at each control
cycle.

Supported PointCloud2 field layouts are:

| Layout | Required fields | Interpretation |
| --- | --- | --- |
| XYZ | `x`, `y`, `z` | Static obstacle points |
| XYZI | `x`, `y`, `z`, `intensity` | Static points; `intensity` is ignored by the planner |
| XYZIV | `x`, `y`, `z`, `intensity`, `vx`, `vy` | Dynamic points with planar velocity in the message frame |

Non-finite points are discarded, and `vx` and `vy` must be present together.
Organized clouds may contain row padding; both byte orders and unaligned
FLOAT32 fields are supported. Inconsistent strides, data lengths, or field
offsets are rejected before point data is read.
See the [dynamic-obstacle message contract](../../docs/dynamic_obstacles_CN.md).

#### Published topics

All publishers use Reliable / Volatile / Keep Last (10) QoS.

| Name | Message type | Description |
| --- | --- | --- |
| `neupan_cmd_vel` | `geometry_msgs/msg/Twist` | Commanded `linear.x` and `angular.z`, published every control cycle |
| `neupan_plan` | `nav_msgs/msg/Path` | Optimized local trajectory in `planning_frame` |
| `neupan_ref_state` | `nav_msgs/msg/Path` | Current reference horizon in `planning_frame` |
| `neupan_initial_path` | `nav_msgs/msg/Path` | Active reference path after transformation and resampling |
| `neupan_arrive` | `std_msgs/msg/Bool` | Goal-reached state, published every control cycle |
| `neupan_diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Solver, input, safety, and command status |
| `dune_point_markers` | `visualization_msgs/msg/MarkerArray` | Obstacle points retained by DUNE |
| `nrmp_point_markers` | `visualization_msgs/msg/MarkerArray` | Points used by the NRMP constraints |
| `robot_marker` | `visualization_msgs/msg/Marker` | Configured robot footprint |

#### Services and actions

The node defines no application-specific services or actions. The standard ROS
2 parameter services created by `rclcpp::Node` remain available.

#### TF

The node looks up these transforms:

- latest `planning_frame <- base_frame` for the robot pose
- timestamped `planning_frame <- sensor_frame` for LaserScan and PointCloud2
- latest `planning_frame <- reference_frame` for path and goal messages

If a safety-critical input cannot be transformed, the node publishes a zero
velocity command. The former `map_frame` parameter is not supported; use
`planning_frame`. See the [coordinate-frame contract](../../docs/coordinate_frames.md).

### `astar_global_node`

- Executable: `astar_global_node`
- Default node name: `astar_global_node`

This example node creates an initial path from an occupancy grid. It is not a
Nav2 planner plugin and is not intended to replace a production global planner.

#### Parameters

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `base_frame` | `string` | `base_link` | Frame used to obtain the A* start pose from TF |
| `robot_radius` | `double` | `0.45` | Occupancy-grid inflation radius in metres |
| `allow_unknown` | `bool` | `false` | Allow traversal through unknown cells |
| `simplify_tolerance` | `double` | `0.15` | Douglas-Peucker path simplification tolerance in metres |
| `goal_tolerance` | `double` | `0.3` | Ignore repeated goals closer than this distance in metres |

#### Subscribed topics

| Name | Message type | QoS | Description |
| --- | --- | --- | --- |
| `map` | `nav_msgs/msg/OccupancyGrid` | Reliable / Transient Local / Keep Last (1) | Map and output-path coordinate frame |
| `goal_pose` | `geometry_msgs/msg/PoseStamped` | Reliable / Volatile / Keep Last (10) | Planning goal; transformed into the map frame |

#### Published topics

| Name | Message type | QoS | Description |
| --- | --- | --- | --- |
| `initial_path` | `nav_msgs/msg/Path` | Reliable / Volatile / Keep Last (10) | Simplified A* path in the occupancy-grid frame |

The node defines no application-specific services or actions.

## Launch files

### `neupan.launch.py`

Starts `neupan_node` and `astar_global_node`, loads the packaged planner
configuration and DUNE model, and remaps `neupan_cmd_vel` to `cmd_vel`.

| Launch argument | Default | Description |
| --- | --- | --- |
| `planning_frame` | `odom` | Value passed to `neupan_node.planning_frame` |
| `base_frame` | `base_link` | Value passed to both nodes' `base_frame` parameter |
