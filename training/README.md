# NeuPAN training

This directory is an offline Python project. It is intentionally outside the
ROS 2 workspace and is not a runtime dependency of `neupan_core` or
`neupan_ros`.

## Quick Start

Install [uv](https://docs.astral.sh/uv/getting-started/installation/), then run
this once from the repository root:

```bash
uv sync --project training --locked
```

uv uses the Python 3.10 pin in `training/.python-version`, creates the isolated
environment at `training/.venv`, installs this Python project, and restores the
exact dependency versions recorded in `training/uv.lock`. No manual activation is
required. Because the trainer currently runs on CPU, uv selects PyTorch's CPU
wheels on Linux and Windows instead of downloading unused CUDA components.

## Train and export

Run the commands from the repository root:

```bash
uv run --project training --locked neupan-train \
  --output training/runs/diff --length 0.5 --width 0.5
uv run --project training --locked neupan-export \
  training/runs/diff/model_5000.pth src/neupan_core/models/diff.bin \
  --length 0.5 --width 0.5
```

The exported NPTF binary stores the MLP tensors and the footprint matrices
needed by the native C++ runtime.

## `neupan-train` command-line options

```text
neupan-train --output DIR --length METRES --width METRES [options]
```

| Argument | Default | Meaning |
| --- | --- | --- |
| `--output` | required | Directory for checkpoints, metadata, and loss logs |
| `--length` | required | Rectangular robot length in metres |
| `--width` | required | Rectangular robot width in metres |
| `--wheelbase` | `0.0` | Base-origin longitudinal offset used to construct the footprint |
| `--data-size` | `100000` | Number of generated obstacle-point samples |
| `--batch-size` | `256` | Training and validation batch size |
| `--epochs` | `5000` | Final training epoch |
| `--seed` | `0` | PyTorch random seed |

The current trainer uses CPU tensors. It writes `train_dict.pkl`, appends metrics
to `results.txt`, and saves `model_<epoch>.pth` every 500 epochs, including epoch
0 and the default final epoch 5000.

## `neupan-export` command-line options

```text
neupan-export CHECKPOINT OUTPUT --length METRES --width METRES [options]
```

| Argument | Default | Meaning |
| --- | --- | --- |
| `checkpoint` | required | PyTorch `model_<epoch>.pth` state dictionary |
| `output` | required | Destination NPTF `.bin` file |
| `--length` | required | Trained robot length in metres |
| `--width` | required | Trained robot width in metres |
| `--wheelbase` | `0.0` | Must match the value used for training and deployment |
| `--edge-dim` | `4` | Number of convex-footprint edges; the rectangular CLI requires 4 |

The NPTF output contains ordered MLP weights, LayerNorm parameters, activation
records, and `meta.G`/`meta.h` footprint matrices. `neupan_core` rejects a model
whose embedded footprint differs from the configured robot dimensions. Keep
`length`, `width`, and `wheelbase` identical in training, export, and the planner
YAML.

This directory is not a ROS 2 package. It does not install ROS 2 nodes or
declare topics, services, actions, or node parameters.
