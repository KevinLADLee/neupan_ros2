# DDR Minimal Sim

A lightweight differential-drive robot (DDR) simulator for ROS2, designed for rapid prototyping and testing of navigation algorithms.

## Overview

DDR Minimal Sim provides a minimal, efficient simulation environment for differential-drive robots with:

- **Differential Drive Kinematics**: Accurate 2-wheel drive simulation with configurable dynamics
- **2D Laser Scanner**: Ray-casting based laser simulation with noise modeling
- **Flexible Environments**: YAML-based scenario configuration system
- **Visualization**: Real-time RViz markers and occupancy grid publishing

This simulator is part of the [NeuPAN ROS2 Workspace](../../README.md) and serves as the recommended testing platform for the NeuPAN navigation planner.

## Features

### Core Components

1. **Simulator Node** (`simulator_node`)
   - Implements differential drive kinematics
   - Publishes odometry, TF transforms, and simulation clock
   - Subscribes to velocity commands (`/cmd_vel`)
   - Supports noise modeling and dynamic constraints

2. **Environment Node** (`environment_node`)
   - Manages static obstacles and boundaries
   - Publishes visualization markers and occupancy grid
   - Loads scenarios from YAML configuration files
   - Includes scenario library with pre-configured environments

3. **Laser Simulator Node** (`laser_simulator_node`)
   - Simulates 2D laser range finder using ray-casting
   - Publishes LaserScan messages on `/scan` topic
   - Configurable range, resolution, and noise parameters

### Pre-configured Scenarios

Six ready-to-use testing scenarios are included:

| Scenario | File | Description |
|----------|------|-------------|
| **Corridor** | `scenario_corridor.yaml` | Long corridor navigation (default for NeuPAN) |
| **Maze** | `scenario_maze.yaml` | Complex maze with multiple turns |
| **Narrow Passage** | `scenario_narrow_passage.yaml` | Challenging narrow passages |
| **U-Trap** | `scenario_u_trap.yaml` | U-shaped dead-end trap |
| **Random Polygons** | `scenario_polygon_random.yaml` | Random polygonal obstacles |
| **Empty** | `scenario_empty.yaml` | Open space for basic testing |

## Quick Start

### As Part of NeuPAN Workspace

**Recommended:** Use the complete NeuPAN system:

```bash
# From workspace root
source install/setup.bash
ros2 launch neupan_ros2 sim_diff_launch.py sim_env_config:=scenario_corridor.yaml
```

### Standalone Usage

Run the simulator independently for custom navigation algorithm testing:

```bash
# Launch complete simulator (all nodes + static TF)
ros2 launch ddr_minimal_sim complete_sim.launch.py sim_env_config:=scenario_maze.yaml

# In another terminal, publish velocity commands
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"
```

### Launch Arguments

```bash
ros2 launch ddr_minimal_sim complete_sim.launch.py \
    sim_env_config:=scenario_maze.yaml \  # Environment configuration
    rviz:=true                             # Launch RViz (default: true)
```

## Configuration

### Robot Parameters

Edit scenario YAML files to configure robot properties:

```yaml
robot:
  init_pose: [x, y, theta]    # Initial position and orientation
  max_velocity: [v, w]        # Maximum linear and angular velocity
  footprint:
    type: "rectangle"
    length: 0.322              # Robot length (m)
    width: 0.22                # Robot width (m)
```

### Laser Scanner

```yaml
laser:
  range_max: 10.0              # Maximum range (m)
  num_rays: 360                # Number of laser rays
  noise_stddev: 0.01           # Measurement noise (m)
```

### Environment

```yaml
environment:
  map: "config/maps/corridor_neupan.yaml"  # Map file path
  obstacles:
    - type: "circle"
      position: [x, y]
      radius: r
    - type: "polygon"
      vertices: [[x1,y1], [x2,y2], ...]
```

## Topics

### Subscribed

- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands for the robot

### Published

- `/odom` (nav_msgs/Odometry) - Robot odometry
- `/scan` (sensor_msgs/LaserScan) - Simulated laser scan
- `/clock` (rosgraph_msgs/Clock) - Simulation clock for `use_sim_time`
- `/vehicle_markers` (visualization_msgs/MarkerArray) - Robot visualization
- `/environment_markers` (visualization_msgs/MarkerArray) - Obstacle visualization
- `/environment_grid` (nav_msgs/OccupancyGrid) - Occupancy grid map

### TF Frames

- `map` -> `odom` (static transform, published by launch file)
- `odom` -> `base_link` (dynamic, published by simulator_node)
- `base_link` -> `laser_link` (static identity transform)

## Dependencies

- **ROS2**: Humble or later
- **C++ Libraries**:
  - Eigen3 (linear algebra)
  - yaml-cpp (YAML parsing)
- **ROS2 Packages**:
  - rclcpp, std_msgs, geometry_msgs, nav_msgs, sensor_msgs, visualization_msgs
  - tf2, tf2_ros, tf2_geometry_msgs

## Development

### Adding Custom Scenarios

1. Create a new YAML file in `config/`:

```yaml
scenario_name: "My Custom Scenario"
robot:
  init_pose: [0.0, 0.0, 0.0]
  max_velocity: [0.5, 2.0]
  footprint:
    type: "rectangle"
    length: 0.322
    width: 0.22

environment:
  obstacles:
    - type: "circle"
      position: [5.0, 0.0]
      radius: 0.5
```

2. Launch with your scenario:

```bash
ros2 launch ddr_minimal_sim complete_sim.launch.py sim_env_config:=my_custom.yaml
```

### Extending the Simulator

Key classes to modify:

- **`MinimalSimulator`** (`include/ddr_minimal_sim/simulator.hpp`): Robot dynamics and kinematics
- **`LaserSimulator`** (`include/ddr_minimal_sim/laser_simulator.hpp`): Sensor modeling
- **`EnvironmentNode`** (`src/environment_node.cpp`): Obstacle management
- **`ScenarioLibrary`** (`src/scenario_library.cpp`): Pre-configured scenarios

## Performance

- **Computational Load**: Minimal (designed for real-time on modern CPUs)
- **Simulation Frequency**: Configurable (default: 100 Hz for dynamics, 20 Hz for laser)
- **Laser Ray-casting**: Efficient 2D algorithm (sub-millisecond per scan)

## Limitations

- 2D only (no 3D simulation)
- Static obstacles only (no dynamic objects)
- Simplified collision detection (uses robot footprint)
- No sensor noise models beyond Gaussian

## Troubleshooting

**Problem:** Robot doesn't move when publishing to `/cmd_vel`

- Check topic connection: `ros2 topic info /cmd_vel`
- Verify velocity limits in scenario YAML
- Ensure `use_sim_time` parameter is consistent across nodes

**Problem:** Laser scan shows no data

- Check if laser_simulator_node is running: `ros2 node list`
- Verify laser range settings in scenario file
- Check TF tree: `ros2 run tf2_tools view_frames`

**Problem:** RViz shows nothing

- Ensure RViz fixed frame is set to `map` or `odom`
- Check that visualization topics are being published: `ros2 topic echo /vehicle_markers --once`

## License

See the main workspace [LICENSE](../../LICENSE) file for license information.

## Author

**KevinLADLee**
GitHub: https://github.com/KevinLADLee

## Contributing

This package is part of the NeuPAN ROS2 Workspace. For contributions, please refer to the main [repository](https://github.com/KevinLADLee/neupan_ros2).

## Acknowledgments

- Designed specifically for testing the [NeuPAN](https://github.com/hanruihua/NeuPAN) neural navigation planner
- Inspired by minimalist simulator design principles
- Referred to [DDR-opt](https://github.com/ZJU-FAST-Lab/DDR-opt) codebase for building the minimal simulator
- Thanks to the ROS2 community

---

**For complete system documentation, see the [workspace README](../../README.md).**
