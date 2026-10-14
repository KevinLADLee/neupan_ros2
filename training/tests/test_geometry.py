"""Polygon config, upstream coefficients and geometry-bound training/export."""

import contextlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
import torch
import yaml

from neupan_training.artifacts import load_checkpoint
from neupan_training.cli import parse_options
from neupan_training.export import export
from neupan_training.geometry import polygon_gh, robot_gh


class PolygonTests(unittest.TestCase):
    def test_upstream_trapezoid_and_clockwise_normalization(self):
        vertices = [[-0.8, -1], [-1.8, 1], [1.8, 1], [0.8, -1]]
        g, h = polygon_gh(vertices)
        np.testing.assert_allclose(g, [[0, -1.6], [2, -1], [0, 3.6], [-2, -1]])
        np.testing.assert_allclose(h, [[1.6], [2.6], [3.6], [2.6]])
        other_g, other_h = polygon_gh([vertices[0], *vertices[:0:-1]])
        np.testing.assert_array_equal(g, other_g)
        np.testing.assert_array_equal(h, other_h)
        actual = robot_gh(dict(vertices=vertices, length=99, width=99))
        np.testing.assert_array_equal(actual[0], g)

    def test_invalid_polygons(self):
        star = [[np.cos(4 * np.pi * i / 5), np.sin(4 * np.pi * i / 5)] for i in range(5)]
        for vertices in ([], [[0, 0], [1, 0]], [[0, 0], [1, 0], [2, 0]],
                         [[0, 0], [1, 0], [0, 0]], [[0, 0], [np.nan, 0], [0, 1]],
                         [[0, 0], [2, 0], [1, .5], [2, 1], [0, 1]], star):
            with self.subTest(vertices=vertices), self.assertRaises(ValueError):
                polygon_gh(vertices)

    def test_polygon_cli_train_resume_and_export(self):
        self.polygon_cli_roundtrip("cpu")

    def test_polygon_gpu_train_resume_and_export(self):
        if not torch.cuda.is_available():
            if os.environ.get("NEUPAN_REQUIRE_CUDA"):
                self.fail("Polygon GPU test requires CUDA")
            self.skipTest("CUDA unavailable")
        self.polygon_cli_roundtrip("cuda")

    def polygon_cli_roundtrip(self, device):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = root / "polygon.yaml"
            # Five edges exercise the variable output dimension through the full pipeline.
            vertices = [[-.5, -.3], [.5, -.3], [.7, 0], [.5, .3], [-.5, .3]]
            config.write_text(yaml.safe_dump(dict(
                robot=dict(vertices=vertices, length=99, width=99),
                train=dict(data_size=12, data_range=[-1, -1, 1, 1], batch_size=4,
                           epoch=1, valid_freq=1, patience=0, label_method="ecos"))))
            command = [sys.executable, "-m", "neupan_training.cli", "--device", device, "--no-cache"]
            subprocess.run(command + ["--config", str(config), "--output", str(root / "run")],
                           check=True, capture_output=True, text=True)
            subprocess.run(command + ["--resume", str(root / "run/last.pt"), "--epochs", "2"],
                           check=True, capture_output=True, text=True)
            saved = load_checkpoint(root / "run/last.pt")
            self.assertEqual(saved["epoch"], 2)
            self.assertEqual(saved["robot_config"], {"vertices": vertices})
            g, h = polygon_gh(vertices)
            np.testing.assert_allclose(saved["geometry"]["G"], g)
            np.testing.assert_allclose(saved["geometry"]["h"], h)
            export(root / "run/best.pt", str(root / "verified.bin"), vertices=vertices,
                   verify_with=os.environ.get("NEUPAN_VERIFY_MODEL"))
            with self.assertRaisesRegex(ValueError, "geometry differs"):
                export(root / "run/best.pt", str(root / "wrong.bin"),
                       vertices=(np.asarray(vertices) * 2).tolist())
            # Original Python checkpoints contain only the state_dict.
            torch.save(saved["model_state"], root / "legacy.pth")
            subprocess.run([sys.executable, "-m", "neupan_training.export",
                            str(root / "legacy.pth"), str(root / "legacy.bin"),
                            "--config", str(config)], check=True, capture_output=True, text=True)
            metadata = json.loads((root / "legacy.bin.json").read_text())
            np.testing.assert_allclose(metadata["geometry"]["G"], g)
            for extra in (["--label-method", "rectangle"], ["--length", "1"]):
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    parse_options(["--config", str(config), "--output", str(root / "bad"), *extra])


if __name__ == "__main__":
    unittest.main()
