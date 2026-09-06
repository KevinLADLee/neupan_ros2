"""GPU-first training from one robot configuration, with automatic best-model export."""

import argparse
from pathlib import Path

import numpy as np
import torch
import yaml

from .artifacts import load_checkpoint, select_device
from .export import export
from .geometry import normalize_robot, robot_gh
from .model import ObsPointNet
from .run import validate_config
from .trainer import DUNETrain

DEFAULTS = dict(data_size=100000, data_range=[-25, -25, 25, 25], batch_size=1024,
                epoch=5000, valid_freq=20, save_freq=100, lr=5e-5, lr_decay=0.5,
                decay_freq=1500, seed=0, label_method="ecos", patience=20, min_delta=0.0)


def parse_options(argv=None):
    parser = argparse.ArgumentParser(description="Train and export a NeuPAN DUNE model (CUDA by default)")
    parser.add_argument("--config", type=Path, help="YAML with robot and optional train sections")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--resume", type=Path, help="Resume last.pt; --epochs is the total target")
    for name in ("length", "width", "wheelbase"):
        parser.add_argument(f"--{name}", type=float)
    for name in ("data-size", "batch-size", "valid-freq", "save-freq", "decay-freq", "seed", "patience"):
        parser.add_argument(f"--{name}", type=int)
    parser.add_argument("--epochs", dest="epoch", type=int)
    for name in ("lr", "lr-decay", "min-delta"):
        parser.add_argument(f"--{name}", type=float)
    parser.add_argument("--data-range", type=float, nargs=4, metavar=("XMIN", "YMIN", "XMAX", "YMAX"))
    parser.add_argument("--label-method", choices=("ecos", "rectangle"))
    parser.add_argument("--device", help="cuda (default), cuda:index, or cpu")
    parser.add_argument("--threads", type=int, default=1, help="PyTorch CPU threads")
    parser.add_argument("--cache-dir", type=Path, default=Path("training/.cache/datasets"))
    parser.add_argument("--no-cache", action="store_true")
    parser.add_argument("--no-export", action="store_true")
    parser.add_argument("--verify-with", type=Path, help="C++ neupan_verify_model executable")
    args = parser.parse_args(argv)
    try:
        document = yaml.safe_load(args.config.read_text()) if args.config else {}
        if not isinstance(document, dict):
            raise ValueError("Configuration must be a YAML mapping")
        saved = load_checkpoint(args.resume) if args.resume else None
        robot = dict(saved.get("robot_config") or {}) if saved else {}
        config = {**DEFAULTS, **(saved["config"] if saved else {})}
        robot.update(document.get("robot", {}))
        overrides = dict(document.get("train", {}))
        configured_device = overrides.pop("device", "cuda")
        if "epochs" in overrides:
            overrides["epoch"] = overrides.pop("epochs")
        unknown = set(overrides) - set(DEFAULTS)
        if unknown:
            raise ValueError(f"Unknown train configuration keys: {sorted(unknown)}")
        config.update(overrides)
        for key in DEFAULTS:
            value = getattr(args, key, None)
            if value is not None:
                config[key] = value
        for name in ("length", "width", "wheelbase"):
            value = getattr(args, name)
            if value is not None:
                robot[name] = value
        if robot.get("vertices") is not None and any(
                getattr(args, key) is not None for key in ("length", "width", "wheelbase")):
            raise ValueError("Dimension flags cannot override robot.vertices; edit the vertices instead")
        robot = normalize_robot(robot)
        if "vertices" in robot and config["label_method"] == "rectangle":
            raise ValueError("Use label_method: ecos for robot.vertices")
        if args.threads < 1:
            raise ValueError("--threads must be at least 1")
        args.device = args.device or configured_device
        args.output = args.output or (args.resume.parent if args.resume else None)
        if args.output is None:
            raise ValueError("--output is required for a new run")
        validate_config(config)
    except (ValueError, TypeError, OSError, yaml.YAMLError) as error:
        parser.error(str(error))
    return args, config, robot


def main():
    args, config, robot = parse_options()
    device = select_device(args.device)
    torch.set_num_threads(args.threads)
    torch.manual_seed(config["seed"])
    np.random.seed(config["seed"])
    g, h = robot_gh(robot)
    model = ObsPointNet(2, len(g)).to(device)
    trainer = DUNETrain(model, torch.from_numpy(g), torch.from_numpy(h), str(args.output))
    best = trainer.start(**config, cache_dir=None if args.no_cache else args.cache_dir,
                         resume=args.resume, robot_config=robot)
    if not args.no_export:
        export(best, str(args.output / "model.bin"), verify_with=args.verify_with)


if __name__ == "__main__":
    main()
