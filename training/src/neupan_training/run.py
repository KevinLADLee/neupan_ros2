"""Training lifecycle: deterministic evaluation, resumable checkpoints and reports."""

from pathlib import Path
import time

import numpy as np
import torch

from .artifacts import FORMAT_VERSION, atomic_save, cpu_state, load_checkpoint, write_json
from .data import TensorBatches, cached_dataset


def validate_config(config):
    for key in ("data_size", "batch_size", "epoch", "valid_freq", "save_freq", "decay_freq"):
        if not isinstance(config[key], int) or isinstance(config[key], bool) or config[key] < 1:
            raise ValueError(f"{key} must be a positive integer")
    if config["data_size"] < 2:
        raise ValueError("data_size must be at least 2 for a train/validation split")
    bounds = np.asarray(config["data_range"], dtype=float)
    if bounds.shape != (4,) or not np.isfinite(bounds).all() or np.any(bounds[:2] >= bounds[2:]):
        raise ValueError("data_range must be finite [xmin, ymin, xmax, ymax] with positive extent")
    for key in ("lr", "lr_decay"):
        if not np.isfinite(config[key]) or config[key] <= 0:
            raise ValueError(f"{key} must be positive and finite")
    if not isinstance(config["patience"], int) or config["patience"] < 0:
        raise ValueError("patience must be a nonnegative integer (0 disables early stopping)")
    if not np.isfinite(config["min_delta"]) or config["min_delta"] < 0:
        raise ValueError("min_delta must be finite and nonnegative")
    if not isinstance(config["seed"], int) or not 0 <= config["seed"] < 2**32:
        raise ValueError("seed must be an integer in [0, 2**32)")
    if config["label_method"] not in ("ecos", "rectangle"):
        raise ValueError("label_method must be ecos or rectangle")


def synchronize(device):
    if device.type == "cuda":
        torch.cuda.synchronize(device)


@torch.no_grad()
def evaluate(trainer, batches):
    trainer.model.eval()
    losses = trainer.train_one_epoch(batches, validate=True)
    errors, overestimates, violations, near_errors = [], [], [], []
    for points, _, distance in batches:
        points = points.squeeze(-1)
        mu = trainer.model(points)
        predicted = trainer.cal_distance(mu.unsqueeze(-1), points)
        error = (predicted - distance).abs()
        errors.append(error)
        overestimates.append((predicted - distance).clamp_min(0))
        violations.append((torch.linalg.vector_norm(mu @ trainer.G, dim=-1) - 1).clamp_min(0))
        near_errors.append(error[distance <= 0.5])
    error = torch.cat(errors)
    near = torch.cat(near_errors)
    values = torch.stack((error.mean(), error.max(), torch.cat(overestimates).max(),
                          torch.cat(violations).max(),
                          near.mean() if near.numel() else error.new_zeros(()))).cpu().tolist()
    return {"losses": list(losses), "total_loss": sum(losses),
            "distance_mae": values[0], "distance_max_error": values[1],
            "distance_max_overestimate": values[2], "max_dual_norm_violation": values[3],
            "near_boundary_mae": values[4] if near.numel() else None,
            "near_boundary_count": near.numel(), "samples": error.numel(),
            "rotation_evaluation": "mean over 0, pi/2, pi, 3pi/2"}


def rng_state(device):
    state = np.random.get_state()
    return {"torch": torch.get_rng_state(),
            "numpy": [state[0], state[1].tolist(), state[2], state[3], state[4]],
            "cuda": torch.cuda.get_rng_state_all() if device.type == "cuda" else []}


def restore_rng(state, device):
    torch.set_rng_state(state["torch"])
    numpy = state["numpy"]
    np.random.set_state((numpy[0], np.asarray(numpy[1], dtype=np.uint32), *numpy[2:]))
    if device.type == "cuda" and state["cuda"]:
        if len(state["cuda"]) != torch.cuda.device_count():
            raise ValueError("Exact CUDA resume requires the same number of visible GPUs")
        torch.cuda.set_rng_state_all(state["cuda"])


def run_training(trainer, config, cache_dir, resume, robot_config):
    validate_config(config)
    output = Path(trainer.checkpoint_path)
    output.mkdir(parents=True, exist_ok=True)
    if not resume and any((output / name).exists() for name in ("last.pt", "best.pt", "config.json")):
        raise ValueError("Output already contains a run; use --resume or a new output directory")
    device = trainer.G.device
    previous = load_checkpoint(resume) if resume else None
    if previous:
        if (output.resolve() != Path(resume).resolve().parent
                and any((output / name).exists() for name in ("last.pt", "best.pt", "config.json"))):
            raise ValueError("Resume destination already contains another run")
        for key, value in config.items():
            if key != "epoch" and previous["config"].get(key) != value:
                raise ValueError(f"Resume configuration mismatch: {key}")
        for name in ("G", "h"):
            if not torch.equal(previous["geometry"][name], trainer.geometry[name]):
                raise ValueError("Resume robot geometry does not match checkpoint")
        if config["epoch"] <= previous["epoch"]:
            raise ValueError("Requested total epochs must exceed the saved epoch")
        robot_config = previous.get("robot_config")

    started = time.perf_counter()
    tensors, data_key, hit = cached_dataset(
        trainer, config["data_size"], config["data_range"], config["seed"],
        config["label_method"], cache_dir)
    generation_seconds = time.perf_counter() - started
    if previous and previous["data_key"] != data_key:
        raise ValueError("Resume dataset identity differs (including generator/solver versions)")
    # CPU-only split generator does not consume the training RNG.
    order = torch.randperm(config["data_size"], generator=torch.Generator().manual_seed(config["seed"]))
    train_size = min(config["data_size"] - 1, max(1, int(config["data_size"] * 0.8)))
    train = TensorBatches([t[order[:train_size]].to(device) for t in tensors],
                          config["batch_size"], shuffle=True)
    valid = TensorBatches([t[order[train_size:]].to(device) for t in tensors],
                          config["batch_size"])
    trainer.optimizer.param_groups[0]["lr"] = config["lr"]
    completed, best_epoch, stale = 0, 0, 0
    best_score, patience_score = float("inf"), float("inf")
    best_state, best_metrics = None, None
    history = []
    if previous:
        for required in ("optimizer_state", "rng", "best_model_state", "history"):
            if required not in previous:
                raise ValueError(f"Checkpoint cannot resume: missing {required}")
        trainer.model.load_state_dict(previous["model_state"])
        trainer.optimizer.load_state_dict(previous["optimizer_state"])
        completed, best_epoch, stale = previous["epoch"], previous["best_epoch"], previous["stale"]
        best_score, patience_score = previous["best_score"], previous["patience_score"]
        best_state, best_metrics = previous["best_model_state"], previous["best_metrics"]
        history = previous["history"]
        restore_rng(previous["rng"], device)
    else:
        torch.manual_seed(config["seed"])
        np.random.seed(config["seed"])

    metadata = {"config": config, "robot": robot_config,
                "geometry": {k: v.tolist() for k, v in trainer.geometry.items()},
                "data_key": data_key, "device": str(device), "torch": str(torch.__version__),
                "cuda": torch.version.cuda,
                "gpu": torch.cuda.get_device_name(device) if device.type == "cuda" else None}
    environment = {key: metadata[key] for key in ("device", "torch", "cuda", "gpu")}
    write_json(metadata, output / "config.json")
    print(f"device={device}, data_cache={'hit' if hit else 'miss'}, "
          f"data_seconds={generation_seconds:.3f}, train={train.size}, valid={valid.size}")
    stopped = False
    for current in range(completed + 1, config["epoch"] + 1):
        trainer.model.train()
        synchronize(device)
        start = time.perf_counter()
        losses = trainer.train_one_epoch(train)
        synchronize(device)
        record = {"epoch": current, "train_losses": list(losses),
                  "train_seconds": time.perf_counter() - start,
                  "lr": trainer.optimizer.param_groups[0]["lr"]}
        if not np.isfinite(losses).all():
            raise RuntimeError(f"Non-finite training loss at epoch {current}")
        if current % config["decay_freq"] == 0:
            trainer.optimizer.param_groups[0]["lr"] *= config["lr_decay"]
        improved = False
        if current == 1 or current % config["valid_freq"] == 0 or current == config["epoch"]:
            metrics = evaluate(trainer, valid)
            score = metrics["total_loss"]
            if not np.isfinite(score):
                raise RuntimeError(f"Non-finite validation loss at epoch {current}")
            record["validation"] = metrics
            if score < best_score:
                best_score, best_epoch = score, current
                best_state, best_metrics = cpu_state(trainer.model), metrics
                improved = True
            if score < patience_score - config["min_delta"]:
                patience_score, stale = score, 0
            else:
                stale += 1
            stopped = bool(config["patience"] and stale >= config["patience"])
            print(f"epoch={current}/{config['epoch']} train={sum(losses):.6g} "
                  f"valid={score:.6g} best_epoch={best_epoch}")
        history.append(record)
        trainer.loss_of_epoch = sum(losses)
        trainer.loss_list.append(trainer.loss_of_epoch)
        if (current % config["save_freq"] == 0 or "validation" in record or stopped):
            checkpoint = {"format_version": FORMAT_VERSION, "model_state": cpu_state(trainer.model),
                          "optimizer_state": trainer.optimizer.state_dict(), "epoch": current,
                          "config": config, "geometry": trainer.geometry, "robot_config": robot_config,
                          "environment": environment,
                          "data_key": data_key, "rng": rng_state(device), "history": history,
                          "best_score": best_score, "best_epoch": best_epoch, "stale": stale,
                          "patience_score": patience_score, "best_model_state": best_state,
                          "best_metrics": best_metrics}
            atomic_save(checkpoint, output / "last.pt")
            if improved:
                atomic_save(checkpoint, output / "best.pt")
            if current % config["save_freq"] == 0:
                atomic_save(checkpoint, output / f"epoch_{current}.pt")
        if stopped:
            break

    # A resumed run can keep an earlier best, including when using a new directory.
    best_artifact = {"format_version": FORMAT_VERSION, "model_state": best_state,
                     "geometry": trainer.geometry, "config": config, "epoch": best_epoch,
                     "environment": environment,
                     "robot_config": robot_config, "metrics": best_metrics, "data_key": data_key}
    atomic_save(best_artifact, output / "best.pt")
    write_json({**metadata, "completed_epochs": current, "best_epoch": best_epoch,
                "early_stopped": stopped, "validation": best_metrics,
                "data_generation_seconds": generation_seconds, "cache_hit": hit,
                "history": history}, output / "report.json")
    print(f"Training complete: best={output / 'best.pt'}, resume={output / 'last.pt'}")
    return str(output / "best.pt")
