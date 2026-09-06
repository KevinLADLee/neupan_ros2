"""
Export a NeuPAN DUNE checkpoint to the NPTF binary format read by
neupan_core's MLP::load.

Usage:
    neupan-export <best.pt> <out.bin>

Geometry comes from a bound checkpoint or explicit legacy robot configuration.

Part of neupan_cpp, a C++ port of NeuPAN (Copyright (c) 2025 Ruihua Han),
distributed under the GNU General Public License v3 or later.
"""

import argparse
import hashlib
import shutil
import subprocess
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
import yaml

from .model import ObsPointNet
from .geometry import rectangle_gh, polygon_gh, normalize_robot
from .nptf import write_nptf
from .artifacts import load_checkpoint, write_json


def export(checkpoint: str, out_path: str, length=None, width=None,
           wheelbase=None, edge_dim=None, verify_with=None, vertices=None):
    """Export bound geometry, or explicitly describe/check the trained footprint."""
    saved = torch.load(checkpoint, map_location="cpu", weights_only=True)
    metadata = {}
    requested_geometry = None
    if vertices is not None:
        requested_geometry = polygon_gh(vertices)
    elif any(value is not None for value in (length, width, wheelbase)):
        if length is None or width is None:
            raise ValueError("Supply both length and width when checking geometry overrides")
        requested_geometry = rectangle_gh(length, width, wheelbase or 0.0)
    if "format_version" in saved:
        saved = load_checkpoint(checkpoint)
        state = saved["model_state"]
        G, h = (saved["geometry"][name].numpy() for name in ("G", "h"))
        if requested_geometry is not None:
            check_g, check_h = requested_geometry
            if (G.shape != check_g.shape or not np.allclose(G, check_g, atol=1e-6, rtol=0)
                    or not np.allclose(h, check_h, atol=1e-6, rtol=0)):
                raise ValueError("Export geometry differs from the geometry bound to training")
        metrics = saved.get("metrics")
        if metrics is None:
            metrics = next((row.get("validation") for row in saved.get("history", [])
                            if row["epoch"] == saved["epoch"]), None)
        metadata = {"epoch": saved["epoch"], "training": saved["config"],
                    "robot": saved.get("robot_config"),
                    "validation": metrics, "data_key": saved.get("data_key"),
                    "environment": saved.get("environment")}
    else:
        if requested_geometry is None:
            raise ValueError("Legacy weights have no geometry; supply --config or --length and --width explicitly")
        state = saved
        G, h = requested_geometry
        metadata["legacy_geometry_unverified"] = True
    if edge_dim is not None and edge_dim != len(G):
        raise ValueError("edge_dim differs from checkpoint geometry")
    if (G.ndim != 2 or G.shape[1] != 2 or h.shape != (len(G), 1)
            or not np.isfinite(G).all() or not np.isfinite(h).all()
            or any(not torch.isfinite(tensor).all() for tensor in state.values())):
        raise ValueError("Export requires finite weights and valid finite G/h geometry")
    edge_dim = len(G)
    model = ObsPointNet(2, edge_dim)
    model.load_state_dict(state)
    model.eval()

    records = []
    for i, layer in enumerate(model.MLP):
        prefix = f"L{i:02d}"
        if isinstance(layer, nn.Linear):
            records.append((f"{prefix}.linear.weight",
                            layer.weight.detach().numpy()))
            records.append((f"{prefix}.linear.bias",
                            layer.bias.detach().numpy()))
        elif isinstance(layer, nn.LayerNorm):
            records.append((f"{prefix}.ln.gamma",
                            layer.weight.detach().numpy()))
            records.append((f"{prefix}.ln.beta",
                            layer.bias.detach().numpy()))
        elif isinstance(layer, nn.Tanh):
            records.append((f"{prefix}.tanh", torch.zeros(0, 0).numpy()))
        elif isinstance(layer, nn.ReLU):
            records.append((f"{prefix}.relu", torch.zeros(0, 0).numpy()))
        else:
            raise TypeError(f"unsupported layer {type(layer)}")

    records.append(("meta.G", G))
    records.append(("meta.h", h))

    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    write_nptf(out_path, records)
    bounds = metadata.get("training", {}).get("data_range", [-25, -25, 25, 25])
    points = np.random.default_rng(0).uniform(bounds[:2], bounds[2:], (1024, 2)).astype(np.float32)
    with torch.no_grad():
        mu = model(torch.from_numpy(points)).numpy()
    fixture = str(out_path) + ".validation.nptf"
    write_nptf(fixture, [("points", points.T), ("mu", mu.T), ("meta.G", G), ("meta.h", h)])
    verifier = verify_with or shutil.which("neupan_verify_model")
    metadata.update({"geometry": {"G": G.tolist(), "h": h.tolist()},
                     "sha256": hashlib.sha256(Path(out_path).read_bytes()).hexdigest(),
                     "format": "NPTF", "cpp_verification": "not_run"})
    write_json(metadata, str(out_path) + ".json")
    if verifier:
        try:
            result = subprocess.run([str(verifier), str(out_path), fixture],
                                    text=True, capture_output=True)
        except OSError as error:
            metadata.update(cpp_verification="failed", cpp_output=str(error))
            write_json(metadata, str(out_path) + ".json")
            raise RuntimeError(f"Cannot run C++ verifier: {error}") from error
        metadata["cpp_verification"] = "passed" if result.returncode == 0 else "failed"
        metadata["cpp_output"] = result.stdout + result.stderr
        write_json(metadata, str(out_path) + ".json")
        if result.returncode:
            raise RuntimeError(f"C++ export verification failed: {metadata['cpp_output']}")
    write_json(metadata, str(out_path) + ".json")
    print(f"exported {len(records)} records to {out_path}; C++ verification: {metadata['cpp_verification']}")
    return str(out_path)


def main():
    parser = argparse.ArgumentParser(
        description="Export a trained DUNE model for neupan_core")
    parser.add_argument("checkpoint")
    parser.add_argument("output")
    parser.add_argument("--config", type=Path, help="Planner YAML describing the trained robot geometry")
    parser.add_argument("--length", type=float)
    parser.add_argument("--width", type=float)
    parser.add_argument("--wheelbase", type=float)
    parser.add_argument("--edge-dim", type=int)
    parser.add_argument("--verify-with", type=Path, help="Path to neupan_verify_model")
    args = parser.parse_args()
    geometry = dict(length=args.length, width=args.width, wheelbase=args.wheelbase)
    if args.config:
        if any(value is not None for value in geometry.values()):
            parser.error("Use either --config or dimension flags")
        try:
            document = yaml.safe_load(args.config.read_text())
            geometry = normalize_robot(document["robot"])
        except (KeyError, ValueError, TypeError, OSError, yaml.YAMLError) as error:
            parser.error(str(error))
    export(args.checkpoint, args.output, edge_dim=args.edge_dim,
           verify_with=args.verify_with, **geometry)


if __name__ == "__main__":
    main()
