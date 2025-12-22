# Robot Configuration Template

This directory contains template files for adding a new robot to NeuPAN.

## Quick Start

1. **Copy this template directory**:
   ```bash
   cd config/robots
   cp -r _template my_robot_name
   ```

2. **Rename template files**:
   ```bash
   cd my_robot_name
   mv robot.yaml.template robot.yaml
   mv planner.yaml.template planner.yaml
   ```

3. **Edit robot.yaml**:
   - Change `robot_type` to your robot name (e.g., 'my_robot_name')
   - Update `robot_description` with a clear description
   - Set `map_frame`, `base_frame`, `lidar_frame` to match your robot's TF tree
   - Adjust visualization and scan processing parameters as needed

4. **Edit planner.yaml**:
   - Set `kinematics`: 'diff' for differential drive, 'acker' for ackermann drive
   - **CRITICAL**: Set `length` and `width` to match your robot's physical dimensions (in meters)
   - Set `max_speed` and `max_acce` based on your robot's capabilities
   - Choose `curve_style`: 'line' for differential, 'dubins' or 'reeds' for ackermann
   - For ackermann robots, set `wheelbase` correctly

5. **Add DUNE model**:
   - If your robot has similar dimensions to an existing robot, create a symlink:
     ```bash
     ln -s ../limo/models/dune_model_5000.pth models/dune_model_5000.pth
     ```
   - Otherwise, train a new DUNE model and place it in `models/`

6. **Create a launch file**:
   - Copy an existing launch file (e.g., `launch/limo.launch.py`)
   - Rename it to `my_robot_name.launch.py`
   - Update the `robot_config_dir` path to point to your robot directory

7. **Build and test**:
   ```bash
   colcon build --packages-select neupan_ros2
   source install/setup.bash
   ros2 launch neupan_ros2 my_robot_name.launch.py
   ```

## Configuration Checklist

### robot.yaml
- [ ] `robot_type` matches your robot folder name
- [ ] `robot_description` is descriptive
- [ ] TF frames match your robot's URDF/setup
- [ ] Scan processing parameters appropriate for your LiDAR
- [ ] Topic names match your robot's interface (or use defaults)

### planner.yaml
- [ ] `kinematics` type is correct ('diff' or 'acker')
- [ ] `length` and `width` match physical robot (measured!)
- [ ] `max_speed` and `max_acce` are safe for your robot
- [ ] `curve_style` appropriate for kinematics type
- [ ] For ackermann: `min_radius` and `wheelbase` are correct

### DUNE Model
- [ ] Model file exists in `models/` directory
- [ ] Model was trained on similar robot dimensions (±20%)
- [ ] Model file is valid PyTorch checkpoint (.pth)

## Notes

- The DUNE neural network model is trained for specific robot dimensions. Using a model trained on significantly different dimensions may result in poor performance.
- Test thoroughly in a safe environment before deployment.
- See `config/robots/README.md` for more detailed documentation.
