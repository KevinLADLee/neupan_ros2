# Scout Mini Diff

Run NeuPAN with a **612 mm long × 580 mm wide** differential-drive robot,
or train a model for the same footprint.

The footprint is centred on `base_link`: `length: 0.612`, `width: 0.580`,
`wheelbase: 0.0`. Here, `wheelbase` controls the footprint's longitudinal
offset; it is not the physical wheel spacing.

## Run the supplied model

Follow the repository [build instructions](../../README.md), then run from
the repository root:

```bash
colcon build --packages-up-to neupan_ros
source install/setup.bash
bash examples/scout_mini_diff/run.sh
```

The example uses the installed
[planner configuration](../../src/neupan_ros/config/scout_mini_diff.yaml) and
[DUNE model](../../src/neupan_core/models/diff_scout_mini_612x580.bin).
Connect the robot's TF, obstacle sensor and path inputs as described in the
[ROS interface documentation](../../src/neupan_ros/README.md).
The node publishes velocity commands on `cmd_vel`.

Additional ROS arguments are forwarded to the node, for example:

```bash
bash examples/scout_mini_diff/run.sh -p base_frame:=base_footprint -r scan:=front/scan
```

Keep the configured body frame at the footprint centre. Adjust the example's
speed and acceleration limits for your robot. C++ inference uses the supplied
`.bin` file and does not require Python, PyTorch or CUDA.

## Train a model

Install the [training environment](../../training/README.md), then run:

```bash
uv sync --project training --locked
uv run --project training --locked neupan-train \
  --config training/configs/scout_mini_diff.yaml \
  --output training/runs/scout_mini
```

The [training configuration](../../training/configs/scout_mini_diff.yaml)
uses CUDA, 100,000 samples, batch size 1024 and up to 5,000 epochs.
Use a new output directory for each run. Training produces `best.pt`,
`last.pt` for resuming, and `model.bin` for deployment.

To run the newly trained model:

```bash
bash examples/scout_mini_diff/run.sh \
  -p dune_checkpoint:="$PWD/training/runs/scout_mini/model.bin"
```

If you change the footprint, update both the training and planner
configurations and retrain the model.
