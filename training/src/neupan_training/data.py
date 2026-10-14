"""CPU label generation/cache and device-resident tensor batching."""

import hashlib
import json
from pathlib import Path

import cvxpy
import ecos
import numpy as np
import torch

from .artifacts import atomic_save


class TensorBatches:
    """Slice whole batches on their resident device; no per-point collation."""

    def __init__(self, tensors, batch_size, shuffle=False):
        self.tensors = tuple(tensors)
        self.batch_size = batch_size
        self.shuffle = shuffle
        self.size = len(self.tensors[0])
        if batch_size < 1 or self.size < 1:
            raise ValueError("batch_size and dataset size must be positive")

    def __len__(self):
        return (self.size + self.batch_size - 1) // self.batch_size

    def __iter__(self):
        order = (torch.randperm(self.size, device=self.tensors[0].device)
                 if self.shuffle else None)
        for start in range(0, self.size, self.batch_size):
            index = slice(start, start + self.batch_size)
            if order is not None:
                index = order[index]
            yield tuple(tensor[index] for tensor in self.tensors)


def rectangle_labels(points, g, h):
    """Exact dual labels for a four-face axis-aligned rectangle, including its interior.

    At the boundary the dual optimizer need not be unique; choose mu=0.
    G row order and positive normal scaling are preserved.
    """
    g, h = np.asarray(g, dtype=np.float64), np.asarray(h).reshape(-1)
    if g.shape != (4, 2) or h.shape != (4,) or not np.isfinite(g).all() or not np.isfinite(h).all():
        raise ValueError("rectangle labels require finite G[4,2] and h[4]")
    axes = np.argmax(np.abs(g), axis=1)
    scales = g[np.arange(4), axes]
    if np.any(scales == 0) or np.any(g[np.arange(4), 1 - axes] != 0):
        raise ValueError("rectangle labels require axis-aligned normals")
    lower, upper = np.empty(2), np.empty(2)
    for axis in range(2):
        positive = np.flatnonzero((axes == axis) & (scales > 0))
        negative = np.flatnonzero((axes == axis) & (scales < 0))
        if len(positive) != 1 or len(negative) != 1:
            raise ValueError("rectangle labels require one positive and negative normal per axis")
        upper[axis] = h[positive[0]] / scales[positive[0]]
        lower[axis] = h[negative[0]] / scales[negative[0]]
    if np.any(lower >= upper):
        raise ValueError("rectangle must have positive area")
    residual = points - np.clip(points, lower, upper)
    distances = np.linalg.norm(residual, axis=1)
    direction = np.divide(residual, distances[:, None], out=np.zeros_like(residual),
                          where=distances[:, None] > 0)
    labels = np.maximum(direction[:, axes] / scales, 0)
    return labels[..., None], distances


def cached_dataset(trainer, size, bounds, seed, method, cache_dir):
    descriptor = {
        "version": 1, "G": trainer.geometry["G"].tolist(),
        "h": trainer.geometry["h"].tolist(), "size": size,
        "bounds": list(bounds), "seed": seed, "method": method,
        "numpy": np.__version__, "cvxpy": cvxpy.__version__, "ecos": ecos.__version__,
    }
    key = hashlib.sha256(json.dumps(descriptor, sort_keys=True).encode()).hexdigest()
    path = Path(cache_dir) / f"{key}.pt" if cache_dir else None
    if path and path.exists():
        saved = torch.load(path, map_location="cpu", weights_only=True)
        if saved.get("descriptor") != descriptor:
            raise ValueError(f"Dataset cache metadata mismatch: {path}")
        tensors = saved["tensors"]
        expected = ((size, 2, 1), (size, trainer.G.shape[0], 1), (size,))
        if len(tensors) != 3 or any(tuple(t.shape) != shape or not torch.isfinite(t).all()
                                    for t, shape in zip(tensors, expected)):
            raise ValueError(f"Invalid dataset cache: {path}")
        return tensors, key, True
    dataset = trainer.generate_data_set(size, bounds, seed=seed, method=method)
    tensors = (dataset.input_data, dataset.label_data, dataset.distance_data)
    if path:
        atomic_save({"descriptor": descriptor, "tensors": tensors}, path)
    return tensors, key, False
