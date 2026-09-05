# neupan_sim

`neupan_sim` is a continuous-geometry mini simulator for the native NeuPAN
deployment loop. It provides a user-facing demonstration and a separate,
deterministic verification scenario; it is not a general-purpose simulator.

## Visual quick start

```bash
ros2 launch neupan_sim quick_start.launch.py
```

The shared-warehouse demo contains four closed shelf islands, a winding route,
static facilities and three moving obstacles crossing the open aisles. RViz
starts by default with the continuous world, robot, travelled trajectory,
dynamic-obstacle velocity arrows, XYZIV hits, initial/reference/planned paths
and an in-scene result label. Use `rviz:=false` for headless runs.

## Deterministic verification

```bash
ros2 launch neupan_sim verify.launch.py
```

This smaller scenario is intended for repeatable pass/fail checks and therefore
runs headless by default. Append `rviz:=true` when debugging it visually.

## Verification model

- ROS-independent C++ simulation core with exact circle and line-segment ray
  intersections; no occupancy-grid discretization.
- Exact differential-drive arc integration with configurable velocity,
  acceleration, command-timeout and integration-substep limits.
- Oriented rectangular robot collision checks against circles, segments and
  world boundaries.
- Moving circles with map-frame velocity and exact reflection at the world
  boundary.
- A single physics timer drives state, TF, odometry and sensor scheduling, so
  every published sensor frame describes one coherent state snapshot.
- LaserScan misses are `+inf`; misses are omitted from PointCloud2 instead of
  being converted into false obstacles at `range_max`.
- PointCloud2 uses strict XYZIV fields (`x/y/z/intensity/vx/vy`) and reports
  velocity in `laser_link` for exercising the deployment transform path.
- A piecewise-linear reference path can be configured with arbitrary
  `path_waypoints`.

## Pass/fail output

`neupan_sim/diagnostics` publishes a latched verification status:

- `running`
- `goal_reached`
- `collision`
- `timed_out`

It also reports elapsed time, final goal distance, travelled path length,
current and minimum geometric clearance, actual velocity, command age and the
number of real scan hits. Collision and timeout are failure states.

## Continuous scenario format

The YAML file uses flattened ROS 2 parameter arrays:

```yaml
world_bounds: [-2.0, 8.0, -4.0, 4.0]  # min_x, max_x, min_y, max_y
path_waypoints: [0.0, 0.0, 3.0, 1.0, 6.0, 0.0]
static_circles: [3.0, 0.65, 0.3]       # x, y, radius
dynamic_circles: [3.2, -1.5, 0.3, 0.0, 0.6]  # x, y, r, vx, vy
segments: [2.0, -2.0, 2.0, -0.7]      # x1, y1, x2, y2
```

The robot size and actuator limits should match the NeuPAN planner YAML. This
keeps failures attributable to the algorithm or interface rather than to a
hidden simulator/planner model mismatch.

All scenario geometry is expressed in the simulator's `map` frame. Scan and
XYZIV outputs are expressed in `laser_link`; velocity components rotate with
that frame. The demo explicitly chooses `planning_frame=map`; real exploration
deployments normally use `odom`. See the
[coordinate-frame contract](../../docs/coordinate_frames.md).
