# DUNE Model Directory

Place your DUNE neural network model file here.

## File Requirements

- **Filename**: `dune_model_5000.pth` (or update `dune_checkpoint_file` in `robot.yaml`)
- **Format**: PyTorch checkpoint file (.pth)
- **Training**: Model should be trained on robot dimensions matching those in `planner.yaml`

## Options

### 1. Use an Existing Model (Symlink)

If your robot has similar dimensions to an existing robot, you can create a symbolic link:

```bash
# Link to LIMO model (0.322m x 0.22m)
ln -s ../../limo/models/dune_model_5000.pth dune_model_5000.pth

# Link to Ranger model (0.720m x 0.500m)
ln -s ../../ranger/models/dune_model_5000.pth dune_model_5000.pth
```

### 2. Train a New Model

If your robot has significantly different dimensions, you should train a new DUNE model:

1. Use the NeuPAN training scripts with your robot's dimensions
2. Train until convergence (typically 5000 iterations)
3. Copy the trained model file here
4. Verify the dimensions match those in `../planner.yaml`

## Dimension Tolerance

- Using a model trained on dimensions within ±20% of your robot may work adequately
- For best performance, train a dedicated model for your exact robot dimensions
- Mismatched dimensions can lead to:
  - Poor obstacle avoidance
  - Incorrect collision predictions
  - Suboptimal path planning

## Verification

After placing your model, verify it loads correctly:

```bash
ros2 launch neupan_ros2 <your_robot>.launch.py
```

Check the console output for:
- Model loading success message
- Robot dimensions logged match your expectations
