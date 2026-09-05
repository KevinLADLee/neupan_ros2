"""Command-line entry point for training a rectangular differential robot."""

import argparse
from pathlib import Path

import torch

from .export import rectangle_gh
from .model import ObsPointNet
from .tensor import np_to_tensor
from .trainer import DUNETrain


def main():
    parser = argparse.ArgumentParser(description="Train a NeuPAN DUNE model")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--length", type=float, required=True)
    parser.add_argument("--width", type=float, required=True)
    parser.add_argument("--wheelbase", type=float, default=0.0)
    parser.add_argument("--data-size", type=int, default=100000)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--epochs", type=int, default=5000)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    torch.manual_seed(args.seed)
    args.output.mkdir(parents=True, exist_ok=True)

    robot_g, robot_h = rectangle_gh(
        args.length, args.width, args.wheelbase)
    model = ObsPointNet(input_dim=2, output_dim=robot_g.shape[0])
    trainer = DUNETrain(
        model,
        np_to_tensor(robot_g),
        np_to_tensor(robot_h),
        str(args.output),
    )
    trainer.start(
        data_size=args.data_size,
        batch_size=args.batch_size,
        epoch=args.epochs,
    )


if __name__ == "__main__":
    main()
