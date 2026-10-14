"""Artifact, data, GPU and end-to-end training contract tests."""

import contextlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import torch

from neupan_training.artifacts import load_checkpoint, select_device
from neupan_training.cli import parse_options
from neupan_training.data import TensorBatches, cached_dataset, rectangle_labels
from neupan_training.export import export, rectangle_gh
from neupan_training.model import ObsPointNet
from neupan_training.run import evaluate
from neupan_training.trainer import DUNETrain


class PipelineTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.settings = dict(data_size=23, data_range=[-1, -1, 1, 1], batch_size=7,
                             valid_freq=1, save_freq=2, decay_freq=2, seed=12,
                             label_method="rectangle", patience=0)
        self.robot = dict(length=0.8, width=0.5, wheelbase=0.2)

    def trainer(self, name="run", device="cpu"):
        torch.manual_seed(1)
        g, h = rectangle_gh(**self.robot)
        return DUNETrain(ObsPointNet().to(device), torch.from_numpy(g),
                         torch.from_numpy(h), str(self.root / name))

    def train(self, trainer, epochs=3, **options):
        with contextlib.redirect_stdout(io.StringIO()):
            return trainer.start(**{**self.settings, **options}, epoch=epochs,
                                 cache_dir=self.root / "cache", robot_config=self.robot)

    def test_cuda_is_the_default_and_cpu_is_explicit(self):
        args, _, _ = parse_options(["--length", "0.5", "--width", "0.5", "--output", "unused"])
        self.assertEqual(args.device, "cuda")
        with patch("torch.cuda.is_available", return_value=False):
            with self.assertRaisesRegex(RuntimeError, "--device cpu"):
                select_device()
            self.assertEqual(select_device("cpu"), torch.device("cpu"))

    def test_rectangle_labels_match_ecos_objective_and_feasibility(self):
        trainer = self.trainer()
        g, h = (trainer.geometry[key].numpy() for key in ("G", "h"))
        # Interior, edge, vertex, exterior and random points, with non-unit normals.
        points = np.vstack(([[0.1, 0], [0.5, 0], [0.5, 0.25], [-2, -2], [2, 0]],
                            np.random.default_rng(5).uniform(-3, 3, (40, 2))))
        labels, distances = rectangle_labels(points, g, h)
        for point, mu, distance in zip(points, labels, distances):
            oracle, _ = trainer.prob_solve(point.reshape(2, 1))
            self.assertAlmostEqual(distance, oracle, delta=2e-6)
            self.assertTrue((mu >= 0).all())
            self.assertLessEqual(np.linalg.norm(g.T @ mu), 1 + 1e-12)
            self.assertAlmostEqual(float((mu.T @ (g @ point[:, None] - h)).item()), distance, places=10)
        permutation = [2, 0, 3, 1]
        shuffled, d = rectangle_labels(points, g[permutation], h[permutation])
        np.testing.assert_allclose(shuffled, labels[:, permutation])
        np.testing.assert_allclose(d, distances)
        with self.assertRaises(ValueError):
            rectangle_labels(points, np.ones((4, 2)), h)

    def test_cache_hit_and_seed_invalidation(self):
        trainer = self.trainer()
        first, key, hit = cached_dataset(trainer, 23, [-1, -1, 1, 1], 0, "rectangle", self.root)
        self.assertFalse(hit)
        with patch.object(trainer, "generate_data_set", side_effect=AssertionError("cache missed")):
            second, key2, hit = cached_dataset(trainer, 23, [-1, -1, 1, 1], 0, "rectangle", self.root)
        self.assertTrue(hit)
        self.assertEqual(key, key2)
        for a, b in zip(first, second):
            torch.testing.assert_close(a, b, rtol=0, atol=0)
            self.assertTrue(a.is_contiguous())
        _, other_key, hit = cached_dataset(trainer, 23, [-1, -1, 1, 1], 1, "rectangle", self.root)
        self.assertFalse(hit)
        self.assertNotEqual(key, other_key)

    def test_validation_is_rng_independent_and_sample_weighted(self):
        trainer = self.trainer()
        dataset = trainer.generate_data_set(11, [-1, -1, 1, 1], seed=0, method="rectangle")
        tensors = (dataset.input_data, dataset.label_data, dataset.distance_data)
        trainer.model.eval()
        torch.manual_seed(7)
        before = torch.get_rng_state().clone()
        first = trainer.train_one_epoch(TensorBatches(tensors, 4), validate=True)
        self.assertTrue(torch.equal(before, torch.get_rng_state()))
        torch.manual_seed(19)
        second = trainer.train_one_epoch(TensorBatches(tensors, 11), validate=True)
        np.testing.assert_allclose(first, second, rtol=2e-6, atol=1e-7)
        points, labels, _ = tensors
        output = trainer.model(points.squeeze(-1)).unsqueeze(-1)
        expected = np.mean([[x.item() for x in trainer.cal_loss_fab(
            output, labels, points.squeeze(-1), theta=angle)]
                            for angle in (0, np.pi/2, np.pi, 3*np.pi/2)], axis=0)
        actual = trainer.cal_loss_fab(output, labels, points.squeeze(-1), deterministic=True)
        np.testing.assert_allclose([x.item() for x in actual], expected, rtol=2e-6, atol=1e-7)

    def test_resume_matches_uninterrupted_training(self):
        self.train(self.trainer("full"), epochs=4)
        self.train(self.trainer("split"), epochs=2)
        self.train(self.trainer("resumed"), epochs=4, resume=self.root / "split/last.pt")
        full = load_checkpoint(self.root / "full/last.pt")
        resumed = load_checkpoint(self.root / "resumed/last.pt")
        self.assertEqual(full["epoch"], 4)
        for key in full["model_state"]:
            torch.testing.assert_close(full["model_state"][key], resumed["model_state"][key], rtol=0, atol=0)
        self.assertEqual(full["optimizer_state"]["param_groups"], resumed["optimizer_state"]["param_groups"])
        for key, state in full["optimizer_state"]["state"].items():
            for name, tensor in state.items():
                torch.testing.assert_close(tensor, resumed["optimizer_state"]["state"][key][name], rtol=0, atol=0)
        with self.assertRaisesRegex(ValueError, "configuration mismatch"):
            self.train(self.trainer("bad"), epochs=5, resume=self.root / "full/last.pt", batch_size=8)

    def test_early_stopping_and_final_checkpoint(self):
        best = self.train(self.trainer(), epochs=8, patience=1, min_delta=1e10)
        last = load_checkpoint(self.root / "run/last.pt")
        self.assertEqual(last["epoch"], 2)
        self.assertLessEqual(load_checkpoint(best)["epoch"], 2)
        report = json.loads((self.root / "run/report.json").read_text())
        self.assertTrue(report["early_stopped"])
        self.assertEqual(len(report["history"]), 2)
        with self.assertRaisesRegex(ValueError, "already contains"):
            self.train(self.trainer())

    def test_metadata_bound_export_and_mismatch_rejection(self):
        best = self.train(self.trainer(), epochs=1)
        with contextlib.redirect_stdout(io.StringIO()):
            export(best, str(self.root / "model.bin"))
        manifest = json.loads((self.root / "model.bin.json").read_text())
        self.assertEqual(manifest["robot"], self.robot)
        self.assertEqual(manifest["epoch"], 1)
        with self.assertRaisesRegex(ValueError, "geometry differs"):
            export(best, str(self.root / "wrong.bin"), length=1, width=1)
        verifier = os.environ.get("NEUPAN_VERIFY_MODEL")
        if verifier:
            export(best, str(self.root / "verified.bin"), verify_with=verifier)
            manifest = json.loads((self.root / "verified.bin.json").read_text())
            self.assertEqual(manifest["cpp_verification"], "passed")

    def test_cli_train_and_resume_from_single_configuration(self):
        config = self.root / "robot.yaml"
        config.write_text("robot:\n  length: 0.8\n  width: 0.5\n  wheelbase: 0.2\n"
                          "train:\n  data_size: 11\n  batch_size: 3\n  epochs: 1\n"
                          "  label_method: rectangle\n  valid_freq: 1\n  patience: 0\n")
        command = [sys.executable, "-m", "neupan_training.cli", "--device", "cpu", "--cache-dir", str(self.root / "cache")]
        subprocess.run(command + ["--config", str(config), "--output", str(self.root / "cli")],
                       check=True, capture_output=True, text=True)
        subprocess.run(command + ["--resume", str(self.root / "cli/last.pt"), "--epochs", "2"],
                       check=True, capture_output=True, text=True)
        self.assertEqual(load_checkpoint(self.root / "cli/last.pt")["epoch"], 2)
        self.assertTrue((self.root / "cli/model.bin").exists())

    def test_gpu_training_and_resume(self):
        if not torch.cuda.is_available():
            if os.environ.get("NEUPAN_REQUIRE_CUDA"):
                self.fail("GPU baseline test requires CUDA")
            self.skipTest("CUDA unavailable; set NEUPAN_REQUIRE_CUDA=1 to require GPU verification")
        self.train(self.trainer("gpu_full", "cuda"), epochs=4)
        self.train(self.trainer("gpu_split", "cuda"), epochs=2)
        self.train(self.trainer("gpu_resume", "cuda"), epochs=4,
                   resume=self.root / "gpu_split/last.pt")
        full = load_checkpoint(self.root / "gpu_full/last.pt")
        resumed = load_checkpoint(self.root / "gpu_resume/last.pt")
        self.assertTrue(full["rng"]["cuda"])
        for key in full["model_state"]:
            torch.testing.assert_close(full["model_state"][key], resumed["model_state"][key], rtol=0, atol=0)


if __name__ == "__main__":
    unittest.main()
