"""
Export a NeuPAN DUNE checkpoint (.pth) to the NPTF binary format read by
neupan_core's MLP::load.

Usage:
    neupan-export <checkpoint.pth> <out.bin> --length 1.0 --width 0.6

length/width: the trained footprint; neupan_core refuses a mismatched robot.

Part of neupan_cpp, a C++ port of NeuPAN (Copyright (c) 2025 Ruihua Han),
distributed under the GNU General Public License v3 or later.
"""

import argparse
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

from .model import ObsPointNet
from .nptf import write_nptf


def rectangle_gh(length: float, width: float, wheelbase: float = 0.0):
    """G, h of a rectangle footprint; keep in step with Robot::diffRectangle."""
    sx = -(length - wheelbase) / 2.0
    sy = -width / 2.0
    v = np.array([[sx, sy],
                  [sx + length, sy],
                  [sx + length, sy + width],
                  [sx, sy + width]], dtype=np.float64)

    n = len(v)
    G = np.zeros((n, 2), dtype=np.float64)
    h = np.zeros((n, 1), dtype=np.float64)
    for i in range(n):
        pre, nxt = v[i], v[(i + 1) % n]
        diff = nxt - pre
        G[i] = (diff[1], -diff[0])
        h[i] = G[i, 0] * pre[0] + G[i, 1] * pre[1]
    return G, h


def export(checkpoint: str, out_path: str, length: float, width: float,
           wheelbase: float = 0.0, edge_dim: int = 4):
    model = ObsPointNet(2, edge_dim)
    state = torch.load(checkpoint, map_location="cpu", weights_only=True)
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

    G, h = rectangle_gh(length, width, wheelbase)
    if len(G) != edge_dim:
        raise ValueError(f"edge_dim {edge_dim} does not match the {len(G)}-edge "
                         "footprint")
    records.append(("meta.G", G))
    records.append(("meta.h", h))

    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    write_nptf(out_path, records)
    print(f"exported {len(records)} records to {out_path} "
          f"(footprint {length} x {width}, wheelbase {wheelbase})")


def main():
    parser = argparse.ArgumentParser(
        description="Export a trained DUNE model for neupan_core")
    parser.add_argument("checkpoint")
    parser.add_argument("output")
    parser.add_argument("--length", type=float, required=True)
    parser.add_argument("--width", type=float, required=True)
    parser.add_argument("--wheelbase", type=float, default=0.0)
    parser.add_argument("--edge-dim", type=int, default=4)
    args = parser.parse_args()
    export(args.checkpoint, args.output, args.length, args.width,
           args.wheelbase, args.edge_dim)


if __name__ == "__main__":
    main()
