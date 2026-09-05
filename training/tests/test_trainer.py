"""Numerical and training regressions; run with unittest discover -s training/tests."""

import copy
from functools import partial
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import torch
from torch.utils.data import DataLoader, TensorDataset

from neupan_training.export import rectangle_gh
from neupan_training.model import ObsPointNet
from neupan_training.tensor import np_to_tensor
from neupan_training.trainer import DUNETrain


def reference_losses(trainer, output_mu, label_mu, points, theta):
    """Original separate projections, kept as an independent numerical oracle."""
    rotation = torch.tensor(
        [[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]],
        dtype=points.dtype,
    )
    fa = (-rotation @ trainer.G.T @ output_mu).transpose(1, 2)
    fa_label = (-rotation @ trainer.G.T @ label_mu).transpose(1, 2)
    fb = fa @ points.unsqueeze(-1) + output_mu.transpose(1, 2) @ trainer.h
    fb_label = fa_label @ points.unsqueeze(-1) + label_mu.transpose(1, 2) @ trainer.h
    distance = torch.bmm(
        output_mu.transpose(1, 2), trainer.G @ points.unsqueeze(-1) - trainer.h
    ).reshape(-1)
    return distance, (fa - fa_label).square().mean(), (fb - fb_label).square().mean()


class TrainerTests(unittest.TestCase):
    def setUp(self):
        torch.manual_seed(7)
        np.random.seed(7)
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        g, h = map(np_to_tensor, rectangle_gh(0.8, 0.5, 0.2))
        self.trainer = DUNETrain(ObsPointNet(), g, h, self.directory.name)

    def loader(self, count=9, batch_size=4):
        return DataLoader(TensorDataset(
            torch.randn(count, 2, 1), torch.rand(count, 4, 1), torch.rand(count)
        ), batch_size=batch_size)

    def test_losses_and_gradients_match_original_projections(self):
        for edges in (3, 4, 7):
            for batch_size in (1, 8, 256):
                for dtype in (torch.float32, torch.float64):
                    with self.subTest(edges=edges, batch_size=batch_size, dtype=dtype):
                        self.trainer.G = torch.randn(edges, 2, dtype=dtype)
                        self.trainer.h = torch.randn(edges, 1, dtype=dtype)
                        points = torch.randn(batch_size, 2, dtype=dtype)
                        labels = torch.rand(batch_size, edges, 1, dtype=dtype)
                        output = torch.rand_like(labels, requires_grad=True)
                        theta = 1.234
                        expected = reference_losses(self.trainer, output, labels, points, theta)
                        actual = (self.trainer.cal_distance(output, points),
                                  *self.trainer.cal_loss_fab(output, labels, points, theta=theta))
                        for result, reference in zip(actual, expected):
                            torch.testing.assert_close(result, reference, atol=2e-6, rtol=2e-5)
                        expected_grad = torch.autograd.grad(
                            sum(x.sum() for x in expected), output, retain_graph=True)[0]
                        actual_grad = torch.autograd.grad(sum(x.sum() for x in actual), output)[0]
                        torch.testing.assert_close(actual_grad, expected_grad, atol=2e-6, rtol=2e-5)
                        self.assertEqual(actual[0].shape, (batch_size,))

    def test_training_update_matches_original(self):
        reference = DUNETrain(copy.deepcopy(self.trainer.model), self.trainer.G,
                              self.trainer.h, self.directory.name)
        loader = self.loader(count=8)
        theta = 0.6
        totals = np.zeros(4)
        for points, labels, distances in loader:
            reference.optimizer.zero_grad()
            points = points.squeeze(-1)
            output = reference.model(points).unsqueeze(-1)
            distance, fa, fb = reference_losses(reference, output, labels, points, theta)
            losses = (reference.loss_fn(output, labels),
                      reference.loss_fn(distance, distances), fa, fb)
            sum(losses).backward()
            reference.optimizer.step()
            totals += [loss.item() for loss in losses]
        with patch.object(self.trainer, "cal_loss_fab",
                          wraps=partial(self.trainer.cal_loss_fab, theta=theta)):
            actual = self.trainer.train_one_epoch(loader)
        np.testing.assert_allclose(actual, totals / len(loader), rtol=2e-5, atol=2e-6)
        for actual, expected in zip(self.trainer.model.parameters(), reference.model.parameters()):
            torch.testing.assert_close(actual, expected, atol=2e-6, rtol=2e-5)

    def test_validation_disables_autograd_and_preserves_optimizer(self):
        loader = self.loader()
        self.trainer.train_one_epoch(loader)
        parameters = [p.detach().clone() for p in self.trainer.model.parameters()]
        gradients = [p.grad.clone() for p in self.trainer.model.parameters()]
        steps = [state["step"].clone() for state in self.trainer.optimizer.state.values()]
        grad_modes = []
        hook = self.trainer.model.register_forward_hook(
            lambda module, inputs, output: grad_modes.append(output.requires_grad))
        self.addCleanup(hook.remove)
        self.trainer.model.eval()
        result = self.trainer.train_one_epoch(loader, validate=True)
        self.assertTrue(torch.is_grad_enabled())
        self.assertEqual(grad_modes, [False] * len(loader))
        self.assertTrue(np.isfinite(result).all())
        for parameter, before, gradient in zip(self.trainer.model.parameters(), parameters, gradients):
            torch.testing.assert_close(parameter, before, rtol=0, atol=0)
            torch.testing.assert_close(parameter.grad, gradient, rtol=0, atol=0)
        for state, step in zip(self.trainer.optimizer.state.values(), steps):
            torch.testing.assert_close(state["step"], step, rtol=0, atol=0)

    def test_singleton_training_and_inference(self):
        for batch_size in (1, 4):
            loader = self.loader(batch_size=batch_size)
            self.assertTrue(np.isfinite(self.trainer.train_one_epoch(loader)).all())
            points, labels, distances = list(loader)[-1]
            grad_modes = []
            hook = self.trainer.model.register_forward_hook(
                lambda module, inputs, output: grad_modes.append(output.requires_grad))
            try:
                losses, elapsed = self.trainer.test_one_epoch(
                    self.trainer.model, points, labels, distances, len(points))
            finally:
                hook.remove()
            self.assertEqual(grad_modes, [False])
            self.assertTrue(np.isfinite(losses).all())
            self.assertGreaterEqual(elapsed, 0)


if __name__ == "__main__":
    unittest.main()
