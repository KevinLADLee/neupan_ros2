# neupan_ros

Native rclcpp deployment package for `neupan_core`.

All ROS topic names below are relative. With the default root namespace they
resolve to the shown absolute names, while namespaces and remappings continue
to work normally.

Inputs:

- `/scan` (`sensor_msgs/LaserScan`)
- `/obstacles` (`sensor_msgs/PointCloud2`): XYZ, XYZI, or XYZIV
- `/initial_path` (`nav_msgs/Path`)
- `/neupan_waypoints` (`nav_msgs/Path`)
- `/neupan_goal` (`geometry_msgs/PoseStamped`)
- TF `planning_frame <- base_frame` and, when frames differ,
  `planning_frame <- sensor/reference frame`

Outputs:

- `/neupan_cmd_vel` (`geometry_msgs/Twist`)
- `/neupan_plan` (`nav_msgs/Path`)
- `/neupan_ref_state` (`nav_msgs/Path`)
- `/neupan_initial_path` (`nav_msgs/Path`)
- `/neupan_arrive` (`std_msgs/Bool`)
- `/neupan_diagnostics` (`diagnostic_msgs/DiagnosticArray`)
- `/dune_point_markers`, `/nrmp_point_markers`
  (`visualization_msgs/MarkerArray`)
- `/robot_marker` (`visualization_msgs/Marker`)

`obstacle_source=scan` creates only the LaserScan subscription,
`obstacle_source=pointcloud` creates only the PointCloud2 subscription, and
`obstacle_source=auto` creates both for freshness-based selection. The point
cloud input uses the relative name `obstacles`; connect a different sensor topic
with a ROS 2 remap instead of a package-specific topic parameter.

`planning_frame` defaults to continuous `odom`. Sensor messages in that frame
use a zero-transform fast path; other sensor frames are transformed at
`header.stamp`. Paths, waypoints and goals may use any non-empty fixed frame;
their original geometry is retained and reprojected with the latest TF. See the
[coordinate-frame contract](../../docs/coordinate_frames.md) for the complete
units, timestamp and per-topic rules.

The old `map_frame` parameter is intentionally unsupported. Set
`planning_frame=odom` for a continuous local planning frame, or explicitly use
`map` in a fixed-world simulation. `config_file` is required; other important
parameters include `base_frame`, `control_rate`, `obstacle_source`, sensor
timeouts, scan filters and `compensate_obstacle_latency`.

Laser filtering, adaptive PointCloud2 decoding, TF transformation, DUNE
inference and QP assembly all run inside the C++ process. It does not embed
Python or rclpy. XYZIV is strictly `x/y/z/intensity/vx/vy`; `vx/vy` is the
planar obstacle velocity in the message frame.

`laser_scan_preprocessor` and `pointcloud_preprocessor` both produce an
`ObstacleObservation`. The ROS node then applies one shared SE(2) transform and
caches the result; the control loop reads the selected cache without copying the
whole obstacle matrix unless dynamic-obstacle latency compensation is enabled.

The planner is registered as the component `neupan_ros::NeuPANNode`. The
package also installs `neupan_node`, a standalone single-threaded executable
generated from the same component, so the two deployment modes cannot drift.

`astar_global_node` is a demo-only A* planner. It adopts the frame from each
`OccupancyGrid`, transforms `goal_pose` into that frame and publishes
`initial_path` in the same frame; it has no configured map-frame parameter.

See the [Chinese ROS interface reference](../../docs/ros_interfaces_CN.md) for
the complete topic, QoS and parameter tables.
