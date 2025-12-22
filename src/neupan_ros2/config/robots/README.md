# NeuPAN Multi-Robot Configuration

This directory contains robot-specific configurations for NeuPAN. Each robot has its own subdirectory with all necessary configuration files and models.

## Directory Structure

```
robots/
├── limo/                   # LIMO differential drive robot
│   ├── robot.yaml          # ROS node parameters
│   ├── planner.yaml        # NeuPAN planner configuration
│   └── models/
│       └── dune_model_5000.pth
│
├── ranger/                 # Ranger Mini ackermann drive robot
│   ├── robot.yaml
│   ├── planner.yaml
│   └── models/
│       └── dune_model_5000.pth
│
├── simulation/             # Simulated robot
│   ├── robot.yaml
│   ├── planner.yaml
│   └── models/
│       └── dune_model_5000.pth (symlink)
│
└── _template/              # Template for new robots
    ├── README.md
    ├── robot.yaml.template
    ├── planner.yaml.template
    └── models/
        └── README.md
```

## Supported Robots

| Robot | Kinematics | Dimensions (L×W) | Wheelbase | Launch Command |
|-------|-----------|------------------|-----------|----------------|
| **LIMO** | Differential | 0.322m × 0.22m | N/A | `ros2 launch neupan_ros2 limo.launch.py` |
| **Ranger** | Ackermann | 0.720m × 0.500m | 0.500m | `ros2 launch neupan_ros2 ranger.launch.py` |
| **Simulation** | Differential | 0.322m × 0.22m | N/A | `ros2 launch neupan_ros2 simulation.launch.py` |

## Adding a New Robot

See `_template/README.md` for detailed instructions.

**Quick steps**:

1. Copy the template:
   ```bash
   cd config/robots
   cp -r _template my_robot
   ```

2. Rename and configure files:
   ```bash
   cd my_robot
   mv robot.yaml.template robot.yaml
   mv planner.yaml.template planner.yaml
   # Edit both files with your robot's specifications
   ```

3. Add DUNE model (symlink or train new)

4. Create launch file (copy from `limo.launch.py`)

5. Test:
   ```bash
   colcon build --packages-select neupan_ros2
   ros2 launch neupan_ros2 my_robot.launch.py
   ```

## Configuration Files

### robot.yaml

Contains ROS integration parameters:
- **Robot identification**: `robot_type`, `robot_description`
- **File paths**: Relative paths to planner config and DUNE model
- **TF frames**: `map_frame`, `base_frame`, `lidar_frame`
- **Visualization**: Marker settings, enable/disable flags
- **Scan processing**: Range, angle, downsampling parameters
- **Behavior**: Control frequency, path refresh settings
- **Topics**: Optional topic name remapping

### planner.yaml

Contains NeuPAN planner parameters:
- **MPC settings**: Receding horizon, step time, reference speed
- **Robot properties**: Kinematics type, physical dimensions
- **Speed limits**: Maximum speed and acceleration
- **Path generation**: Curve style, minimum radius, thresholds
- **PAN algorithm**: Iteration settings, thresholds
- **Tuning parameters**: Control weights, collision distances

## Design Principles

1. **Self-contained**: Each robot folder contains everything needed for that robot
2. **Flat naming**: Simple filenames (`robot.yaml`, `planner.yaml`) - context from folder
3. **Relative paths**: All internal paths relative to robot directory
4. **Clear separation**: ROS parameters vs. planner parameters
5. **Model colocated**: DUNE checkpoint lives with robot config

## Troubleshooting

### Model Loading Errors

Check:
- Model file exists at the path specified in `robot.yaml`
- Model file is a valid PyTorch checkpoint
- File permissions are correct

### Wrong Dimensions

If robot behaves unexpectedly:
- Verify `length` and `width` in `planner.yaml` match physical robot
- Check DUNE model was trained for similar dimensions
- Look at console logs for dimension mismatch warnings

### TF Frame Errors

Ensure:
- Frame names in `robot.yaml` match your robot's TF tree
- All frames exist and are being published
- Use `ros2 run tf2_tools view_frames` to visualize TF tree

### Launch Failures

- Check `robot_config_dir` path in launch file is correct
- Verify all required files exist (robot.yaml, planner.yaml, model file)
- Look for detailed error messages in console output

## Migration from Old Structure

If you have existing configurations in the old format:

**Old location** → **New location**
- `config/<robot>.yaml` → `config/robots/<robot>/robot.yaml`
- `config/neupan_config/neupan_<robot>.yaml` → `config/robots/<robot>/planner.yaml`
- `config/dune_checkpoint/<robot_type>/` → `config/robots/<robot>/models/`

Update the new `robot.yaml` to include:
- `robot_type` and `robot_description`
- Change `neupan_config_file` → `planner_config_file: 'planner.yaml'`
- Change `dune_checkpoint_file` → `dune_checkpoint_file: 'models/dune_model_5000.pth'`

## Best Practices

1. **Test incrementally**: Verify each robot configuration individually
2. **Document changes**: Add comments explaining non-obvious parameter choices
3. **Version control**: Commit working configurations before making changes
4. **Safe testing**: Always test in simulation or safe environment first
5. **Model management**: Keep track of which DUNE model works with which robot dimensions
6. **Naming consistency**: Use lowercase, descriptive names for robot types

## Support

For issues or questions:
- Check console logs for detailed error messages
- Verify all configuration parameters are set correctly
- Test with known-working robots (limo, ranger) to isolate issues
- Review the template README for configuration guidance
