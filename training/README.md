# NeuPAN training

Offline DUNE training and model export for the native C++ planner. Training is
not a ROS 2 package or a runtime dependency. CUDA is the default training device;
CPU is an explicit alternative. The network architecture and NPTF runtime format
remain compatible with `neupan_core`.

See the [Scout Mini Diff example](../examples/scout_mini_diff/README.md) for
training and running a **612 mm × 580 mm** robot with the supplied model.

## Environment

Install [uv](https://docs.astral.sh/uv/getting-started/installation/) >= 0.8.16,
then run from the repository root:

```bash
uv sync --project training --locked
```

This installs the default `gpu` dependency group: CUDA 12.8 PyTorch on Linux and
Windows. A compatible NVIDIA driver is required; a separate CUDA toolkit install
is not required for the prebuilt wheels. macOS users must select CPU explicitly.
The Python environment and all dependency versions are recorded in `uv.lock`.
See [uv's PyTorch integration](https://docs.astral.sh/uv/guides/integration/pytorch/).

For a CPU-only installation (also used by hosted CI):

```bash
uv sync --project training --locked --no-default-groups --group cpu
uv run --project training --locked --no-default-groups --group cpu \
  neupan-train --device cpu --config training/configs/diff.yaml --output training/runs/cpu
```

Keep the group flags on subsequent `uv run` calls, otherwise uv restores the
GPU default environment. A CUDA-enabled installation can also run `--device cpu`.
Training fails with an actionable error when CUDA is requested but unavailable;
it never silently changes to CPU.

## Train and export

```bash
uv run --project training --locked neupan-train \
  --config training/configs/diff.yaml --output training/runs/diff
```

Use one robot YAML for training and deployment. The `robot.length`, `robot.width`
and optional `robot.wheelbase` fields are consumed from either a planner YAML or
the example above; an optional `train` section sets training parameters. CLI
flags override YAML values. The CLI currently supports rectangular footprints.

Geometry flags remain available:

```bash
uv run --project training --locked neupan-train \
  --output training/runs/custom --length 0.8 --width 0.5 --wheelbase 0.2 \
  --epochs 1000 --batch-size 1024 --device cuda:0
```

`--epochs N` means exactly N total epochs (numbered 1 through N), unless early
stopping terminates sooner. The final epoch is always saved and validated.
Training automatically exports the best model to `model.bin`; use `--no-export`
when only training artifacts are wanted.

| Artifact | Purpose |
| --- | --- |
| `best.pt` | Best validation model, bound geometry and training configuration |
| `last.pt` | Resume checkpoint: current weights, Adam state, next learning rate, RNG states, epoch, best weights and early-stop state |
| `epoch_N.pt` | Periodic full recovery checkpoint |
| `config.json` | Resolved configuration, geometry, dataset identity and device information |
| `report.json` | Training history, timings and best-model validation metrics |
| `model.bin` | NPTF weights and geometry loaded by the native planner |
| `model.bin.json` | Export metadata, model checksum and C++ verification status |
| `model.bin.validation.nptf` | Fixed points and Python outputs for C++ verification |

A new run refuses to overwrite an existing training run. Choose a new output
folder or resume it. Treat `last.pt` and the periodic checkpoints as recovery
artifacts; `best.pt` is intended for inference/export.

## Resume

```bash
uv run --project training --locked neupan-train \
  --resume training/runs/diff/last.pt --epochs 2000
```

Resume reads robot geometry and training settings from the checkpoint. The epoch
argument is the new total, not the number of additional epochs. A different
`--output` can continue into a new folder. Changing the dataset, batch size,
learning-rate schedule or early-stop settings is rejected to avoid silently
changing the experiment. Exact replay requires the same device type, visible
CUDA devices and software environment. CUDA is still the default; add
`--device cpu` when resuming a CPU run.

## Data and validation

Labels are generated on CPU and cached under `training/.cache/datasets` using a
hash of geometry, sample count/range, seed, label method and generator/solver
versions. Use `--cache-dir DIR` to share a cache or `--no-cache` to disable it.
The train/validation split is seeded. Contiguous tensors are moved to the selected
device once; training shuffles indices on that device each epoch. The default
100,000-point rectangle dataset is small enough to reside on a typical GPU.

ECOS remains the default label generator and supports the trainer's general
convex half-space formulation. `--label-method rectangle` enables a vectorized
analytic backend for axis-aligned rectangles, preserving G row ordering and
normal scaling. It returns both distance and dual mu. On the footprint boundary
mu is not unique; this backend selects zero, so it is an explicit option rather
than an unnoticed change to ECOS supervision.

Training samples one random rotation per batch directly on its device.
Validation uses the exact average of the four fixed rotations 0, pi/2, pi and
3pi/2, which also equals the uniform-angle expected auxiliary loss. Validation
consumes no training RNG, records no gradients and weights metrics by sample
count, including the tail batch. The report includes distance MAE, maximum error,
maximum distance overestimate, dual norm violation and MAE for samples within
0.5 m of or inside the footprint. A missing near-boundary sample is reported as
null, not zero. These metrics describe the held-out validation set; they do not
constitute a navigation safety or convergence guarantee.

Early stopping monitors total deterministic validation loss. `--patience` counts
validation checks without an improvement greater than `--min-delta`;
`--patience 0` disables it. Best-model selection always tracks the lowest observed
validation loss. The default checks every 20 epochs, plus the first and final
epoch. Configuration and CLI options include learning rate, decay schedule,
sampling range, validation/save frequency, seed and CPU thread count.

## Export and C++ verification

Geometry comes directly from the checkpoint. No dimensions need to be re-entered:

```bash
uv run --project training --locked neupan-export \
  training/runs/diff/best.pt training/runs/diff/model.bin
```

Build `neupan_verify_model` with `NEUPAN_BUILD_TOOLS=ON` in the native core, or
build the standalone verifier using only Eigen and a C++ compiler:

```bash
g++ -std=c++17 -O2 -I/usr/include/eigen3 -Isrc/neupan_core/include \
  src/neupan_core/tools/verify_model.cpp src/neupan_core/src/mlp.cpp \
  src/neupan_core/src/tensor_io.cpp -o /tmp/neupan_verify_model

uv run --project training --locked neupan-export \
  training/runs/diff/best.pt training/runs/diff/model.bin \
  --verify-with /tmp/neupan_verify_model
```

`--verify-with` is also accepted by `neupan-train` for automatic verification
following training. An installed `neupan_verify_model` on PATH is used automatically.
The verifier checks geometry and Python/C++ output agreement (mu tolerance 1e-4,
distance tolerance 1e-3) over 1,024 fixed points. Without a verifier, export status
is explicitly `not_run`. Failed verification makes the command fail and records
`failed` in the manifest; do not deploy that artifact.

Legacy plain `.pth` state dictionaries still require explicit `--length`,
`--width` and optionally `--wheelbase`. Their geometry cannot be authenticated
against training and is marked `legacy_geometry_unverified`. For new checkpoints,
a conflicting geometry override is rejected. Keep the planner robot geometry
consistent with the model and point its `pan.dune_checkpoint` to the exported
file (using the path format accepted by your launch configuration).

## Tests and performance baseline

```bash
uv run --project training --locked python -m unittest discover -s training/tests -v
NEUPAN_REQUIRE_CUDA=1 NEUPAN_VERIFY_MODEL=/tmp/neupan_verify_model \
  uv run --project training --locked python -m unittest discover -s training/tests -v

uv run --project training --locked neupan-benchmark \
  --device cuda --batch-sizes 256 1024 4096 --output training/runs/benchmark.json
```

The regression suite covers losses/gradients, deterministic validation, tail
batches, cache reuse, label feasibility, checkpoint resume, geometry mismatch,
CLI train/export and GPU execution. CUDA tests skip on CPU-only hosts unless
`NEUPAN_REQUIRE_CUDA=1` is set. Hosted CI installs the CPU group and runs the
training and C++ export tests; a GPU baseline can be required on a CUDA machine
with the command above.

The benchmark reports warmed-up epoch time and throughput, including batch
indexing and optimization. It synchronizes CUDA around measurements and records
the device, PyTorch version and settings. It is not a convergence comparison:
larger batches perform fewer optimizer updates per epoch. Select production
settings by time to the required validation quality as well as throughput.

The recorded [RTX 4090 baseline](benchmarks/rtx4090.json) uses 100,000 points,
PyTorch 2.11.0+cu128, one CPU thread, one warmup epoch and five timed epochs:

| Batch size | Median epoch time | Samples/s |
| --- | --- | --- |
| 256 | 432 ms | 231,388 |
| 1024 (training default) | 110 ms | 912,510 |
| 4096 | 28 ms | 3,549,530 |

These measurements exclude label generation, validation and artifact writing.
