# Coordinate-frame contract

This document is normative for the native NeuPAN runtime. Inputs that do not
satisfy it are rejected instead of being interpreted in an implicit frame.

## Conventions

- SI units: position in metres, velocity in metres per second, angle in radians.
- Right-handed REP-103 axes: `base_frame` x forward, y left, z up; positive yaw
  is counter-clockwise about +z.
- `planning_frame` is the common local planning frame. It defaults to `odom`,
  but may be any continuous reference frame; it is not synonymous with the ROS
  global `map` frame.
- In the notation `planning_frame <- sensor_frame`, coordinates expressed in
  the sensor frame are transformed into `planning_frame`.
- Exploration and SLAM deployments should normally use continuous `odom` so a
  loop-closure or relocalization jump in `map` cannot contaminate local paths or
  dynamic-obstacle velocities.

## `neupan_core`

The core does not know frame names and performs no TF lookup. One call to
`NeuPANPlanner::forward` must use a single common world frame:

| Value | Shape | Frame and unit |
| --- | --- | --- |
| `state` | `3 x 1` | `[x_m, y_m, yaw_rad]` in the common world frame |
| `points` | `2 x N` | obstacle `[x_m, y_m]` in that frame |
| `velocities` | `2 x N` | obstacle `[vx_mps, vy_mps]` along that frame's axes |
| initial path | `4 x K` logically | `[x_m, y_m, yaw_rad, gear]` in that frame |

Position and velocity columns are paired. The adapter calling the core owns
the invariant; mixing robot-frame points with a map-frame state is invalid.

## `neupan_ros` inputs

| Input | Required `header.frame_id` | Handling |
| --- | --- | --- |
| `scan` | non-empty sensor frame | used directly when frames match; otherwise transformed at `header.stamp` |
| `obstacles` | non-empty sensor/source frame | used directly when frames match; otherwise positions and velocities are transformed at `header.stamp` |
| `initial_path` | any non-empty fixed frame | retained and continuously transformed with the latest TF; at least two poses |
| `neupan_waypoints` | any non-empty fixed frame | retained and continuously transformed into `planning_frame` |
| `neupan_goal` | any non-empty fixed frame | retained and continuously transformed into `planning_frame` |
| A* `map` | any non-empty fixed frame | uses `OccupancyGrid.header.frame_id` and supports a rotated map origin |
| A* `goal_pose` | any non-empty fixed frame | transformed into the current OccupancyGrid frame with the latest TF |

For `nav_msgs/Path`, the top-level `Path.header.frame_id` is authoritative;
individual `PoseStamped.header` values are not used. Path headings are derived
from adjacent XY points unless `include_initial_path_direction=true`.

For sensor messages, a zero timestamp means the node's current time. A
non-zero timestamp requires a matching historical TF. Missing TF causes that
sensor frame to be discarded. A sensor already in `planning_frame` skips the
TF lookup and matrix transform.

Paths, waypoints and goals are persistent reference geometry rather than sensor
samples. The node retains their original-frame data and updates their projection
into `planning_frame` using the latest TF during control. A changing SLAM
`map -> odom` transform therefore cannot leave a stale one-shot conversion.
Empty frame IDs are always rejected.

PointCloud2 XYZIV means `x/y/z/intensity/vx/vy`. Position and velocity are both
expressed in the message frame. The ROS adapter applies rotation and translation
to XY positions, only rotation to velocity vectors, and then calls the core in
`planning_frame`. Z is checked for finiteness but is not used by the planar planner.

The robot state comes from the latest `planning_frame <- base_frame` TF.

## Outputs

- `neupan_cmd_vel` has no header. `linear.x` is along `base_frame` +x in m/s;
  `angular.z` is positive counter-clockwise about +z in rad/s.
- `neupan_plan`, `neupan_ref_state`, `neupan_initial_path`, planner markers and
  diagnostics are produced in or describe `planning_frame`.
- The simulator publishes `map -> base_link -> laser_link`; `/scan` and
  `/obstacles` use `laser_link`, while `/initial_path` uses `map`.
- Simulator YAML positions, path points, segments and obstacle velocities are
  all expressed in its `map` frame.

## Required TF example

```text
planning_frame
  └── base_frame
        └── lidar frame from scan/obstacles header.frame_id
```

The lidar need not be a direct child of the base as long as TF can resolve the
complete transform at the sensor timestamp. A global `map` need not be the
parent of `planning_frame`; the frames only need to be connected when global
references are supplied.
