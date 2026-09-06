"""Portable, atomic training artifacts (no pickled model instances)."""

import json
import os
from pathlib import Path
import tempfile

import torch

FORMAT_VERSION = 1


def atomic_save(value, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=path.parent, suffix=".tmp", delete=False) as f:
        temporary = Path(f.name)
    try:
        torch.save(value, temporary)
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def write_json(value, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode="w", dir=path.parent, suffix=".tmp",
                                     delete=False) as f:
        temporary = Path(f.name)
        try:
            json.dump(value, f, indent=2, allow_nan=False)
            f.write("\n")
        except Exception:
            temporary.unlink(missing_ok=True)
            raise
    try:
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def cpu_state(model):
    return {name: tensor.detach().cpu().clone()
            for name, tensor in model.state_dict().items()}


def load_checkpoint(path):
    result = torch.load(path, map_location="cpu", weights_only=True)
    if not isinstance(result, dict) or result.get("format_version") != FORMAT_VERSION:
        raise ValueError("Expected a versioned training checkpoint; legacy weights can only be exported")
    for key in ("model_state", "geometry", "config", "epoch"):
        if key not in result:
            raise ValueError(f"Checkpoint is missing {key}")
    return result


def select_device(name="cuda"):
    device = torch.device(name)
    if device.type not in ("cpu", "cuda"):
        raise ValueError("Training supports cuda[:index] or cpu")
    if device.type == "cuda":
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA training requested but CUDA is unavailable. Install the default "
                               "CUDA environment and check the NVIDIA driver, or explicitly use --device cpu.")
        if device.index is not None and device.index >= torch.cuda.device_count():
            raise ValueError(f"CUDA device index {device.index} is unavailable")
    return device
