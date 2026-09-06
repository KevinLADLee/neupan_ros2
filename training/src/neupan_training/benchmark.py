"""Reproducible CUDA-first epoch benchmark; not a convergence benchmark."""

import argparse
from pathlib import Path
import statistics
import tempfile
import time

import torch

from .artifacts import select_device, write_json
from .data import TensorBatches
from .export import rectangle_gh
from .model import ObsPointNet
from .run import synchronize
from .trainer import DUNETrain


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--batch-sizes", nargs="+", type=int, default=[256, 1024, 4096])
    parser.add_argument("--threads", type=int, default=1)
    parser.add_argument("--data-size", type=int, default=100000)
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--output", type=Path, default=Path("training/runs/benchmark.json"))
    args = parser.parse_args()
    if min(args.batch_sizes + [args.data_size, args.repeats, args.threads]) < 1:
        parser.error("sizes, repeats and threads must be positive")
    device = select_device(args.device)
    torch.set_num_threads(args.threads)
    g, h = map(torch.from_numpy, rectangle_gh(0.5, 0.5))
    rows = []
    with tempfile.TemporaryDirectory() as root:
        trainer = DUNETrain(ObsPointNet(), g, h, root)
        started = time.perf_counter()
        dataset = trainer.generate_data_set(args.data_size, [-25, -25, 25, 25], seed=0, method="rectangle")
        generation = time.perf_counter() - started
        tensors = tuple(t.to(device) for t in (dataset.input_data, dataset.label_data, dataset.distance_data))
        for batch_size in args.batch_sizes:
            torch.manual_seed(0)
            trainer = DUNETrain(ObsPointNet().to(device), g, h, root)
            trainer.optimizer.param_groups[0]["lr"] = 5e-5
            loader = TensorBatches(tensors, batch_size, shuffle=True)
            trainer.train_one_epoch(loader)  # warm up kernels/optimizer
            timings = []
            for _ in range(args.repeats):
                synchronize(device)
                started = time.perf_counter()
                losses = trainer.train_one_epoch(loader)
                synchronize(device)
                timings.append(time.perf_counter() - started)
            median = statistics.median(timings)
            rows.append({"batch_size": batch_size, "epoch_seconds": median,
                         "samples_per_second": args.data_size / median,
                         "last_train_losses": list(losses)})
            print(rows[-1])
    write_json({"device": str(device), "threads": args.threads, "torch": str(torch.__version__),
                "gpu": torch.cuda.get_device_name(device) if device.type == "cuda" else None,
                "data_size": args.data_size, "repeats": args.repeats,
                "data_range": [-25, -25, 25, 25], "seed": 0, "warmup_epochs": 1,
                "learning_rate": 5e-5, "label_method": "rectangle",
                "robot": {"length": 0.5, "width": 0.5, "wheelbase": 0.0},
                "label_generation_seconds": generation, "results": rows,
                "note": "Warm epoch throughput only; batch sizes have different optimizer update counts."},
               args.output)


if __name__ == "__main__":
    main()
