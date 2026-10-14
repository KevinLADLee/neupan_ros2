"""
DUNETrain is the class for training the DUNE model. It is used when you deploy the NeuPan algorithm on a new robot with a specific geometry.

Developed by Ruihua Han
Copyright (c) 2025 Ruihua Han <hanrh@connect.hku.hk>

NeuPAN planner is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

NeuPAN planner is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with NeuPAN planner. If not, see <https://www.gnu.org/licenses/>.
"""

import torch
from colorama import deinit

deinit()

from torch.utils.data import Dataset, DataLoader
import cvxpy as cp
from rich.console import Console
from rich.progress import Progress
from rich.live import Live
from torch.optim import Adam
import numpy as np
from .tensor import np_to_tensor, value_to_tensor
from .data import rectangle_labels
import pickle
import time
import os


class PointDataset(Dataset):
    def __init__(self, input_data, label_data, distance_data):
        """
        input_data: point p, [2, 1]
        label_data: mu, [G.shape[0], 1]
        distance_data: distance, scalar
        """

        self.input_data = (input_data if isinstance(input_data, torch.Tensor)
                           else torch.stack(input_data)).contiguous()
        self.label_data = (label_data if isinstance(label_data, torch.Tensor)
                           else torch.stack(label_data)).contiguous()
        self.distance_data = (distance_data if isinstance(distance_data, torch.Tensor)
                              else torch.stack(distance_data)).contiguous()

    def __len__(self):
        return len(self.input_data)

    def __getitem__(self, idx):
        input_sample = self.input_data[idx]
        label_sample = self.label_data[idx]
        distance_sample = self.distance_data[idx]

        return input_sample, label_sample, distance_sample


class DUNETrain:
    def __init__(self, model, robot_G, robot_h, checkpoint_path) -> None:

        self.geometry = {"G": robot_G.detach().cpu().double().clone(),
                         "h": robot_h.detach().cpu().double().clone()}
        parameter = next(model.parameters())
        self.G = robot_G.to(device=parameter.device, dtype=parameter.dtype)
        self.h = robot_h.to(device=parameter.device, dtype=parameter.dtype)
        self.model = model

        self.construct_problem()
        self.checkpoint_path = checkpoint_path

        self.loss_fn = torch.nn.MSELoss()

        self.optimizer = Adam(self.model.parameters(), lr=1e-4, weight_decay=1e-4)

        # for rich progress
        self.console = Console()
        self.progress = Progress(transient=False)
        self.live = Live(self.progress, console=self.console, auto_refresh=False)

        # loss
        self.loss_of_epoch = 0
        self.loss_list = []

    def construct_problem(self):
        """
        optimization problem (10):

        max mu^T * (G * p - h)
        s.t. ||G^T * mu|| <= 1
            mu >= 0
        """
        self.mu = cp.Variable((self.G.shape[0], 1), nonneg=True)
        self.p = cp.Parameter((2, 1))  # points

        g = self.geometry["G"].numpy()
        h = self.geometry["h"].numpy()
        cost = self.mu.T @ (g @ self.p - h)
        constraints = [cp.norm(g.T @ self.mu) <= 1]

        self.prob = cp.Problem(cp.Maximize(cost), constraints)

    def process_data(self, rand_p):
        distance_value, mu_value = self.prob_solve(rand_p)  # Adapted to be accessible
        return (
            np_to_tensor(rand_p),
            np_to_tensor(mu_value),
            value_to_tensor(distance_value),
        )

    def generate_data_set(self, data_size=10000, data_range=(-50, -50, 50, 50),
                          seed=None, method="ecos"):
        """
        generate dataset for training
        data_range: [low_x, low_y, high_x, high_y]
        """

        rand_p = np.random.default_rng(seed).uniform(
            low=data_range[:2], high=data_range[2:], size=(data_size, 2)
        )
        if method == "rectangle":
            labels, distances = rectangle_labels(
                rand_p, self.geometry["G"].numpy(), self.geometry["h"].numpy())
        elif method == "ecos":
            labels = np.empty((data_size, self.G.shape[0], 1))
            distances = np.empty(data_size)
            for i, point in enumerate(rand_p):
                distances[i], labels[i] = self.prob_solve(point.reshape(2, 1))
        else:
            raise ValueError(f"Unknown label method: {method}")
        return PointDataset(torch.from_numpy(rand_p[..., None]).float(),
                            torch.from_numpy(labels).float(),
                            torch.from_numpy(distances).float())

    def prob_solve(self, p_value):

        self.p.value = p_value
        self.prob.solve(solver=cp.ECOS)  # distance
        if (self.prob.status != cp.OPTIMAL or self.mu.value is None
                or not np.isfinite(self.prob.value) or not np.isfinite(self.mu.value).all()):
            raise RuntimeError(f"ECOS label solve failed at {p_value.ravel()}: {self.prob.status}")

        return self.prob.value, self.mu.value

    def start(
        self, data_size=100000, data_range=(-25, -25, 25, 25),
        batch_size=1024, epoch=5000, valid_freq=20, save_freq=100,
        lr=5e-5, lr_decay=0.5, decay_freq=1500, save_loss=False,
        seed=0, cache_dir=None, label_method="ecos", patience=20,
        min_delta=0.0, resume=None, robot_config=None, **kwargs,
    ):
        """Train exactly ``epoch`` total epochs, or resume to that total."""
        from .run import run_training
        if kwargs:
            raise TypeError(f"Unknown training options: {sorted(kwargs)}")
        config = dict(data_size=data_size, data_range=list(data_range),
                      batch_size=batch_size, epoch=epoch, valid_freq=valid_freq,
                      save_freq=save_freq, lr=lr, lr_decay=lr_decay,
                      decay_freq=decay_freq, seed=seed, label_method=label_method,
                      patience=patience, min_delta=min_delta)
        return run_training(self, config, cache_dir, resume, robot_config)

    def train_one_epoch(self, train_dataloader, validate=False):
        """
        loss:
            mu: mse between output mu and label mu
            objective function value (distance): mse between output distance and label distance
            fa: -mu^T * G * R^T  ==> lam^T
            fb: mu^T * G * R^T * p - mu^T * h  ==> lam^T * p + mu^T * h
        """

        # Accumulate detached metrics on-device; transfer only once per epoch.
        totals = torch.zeros(4, dtype=torch.float64, device=self.G.device)
        count = 0
        with torch.set_grad_enabled(not validate):
            for input_point, label_mu, label_distance in train_dataloader:
                input_point, label_mu, label_distance = (
                    tensor.to(self.G.device) for tensor in (input_point, label_mu, label_distance))
                if not validate:
                    self.optimizer.zero_grad(set_to_none=True)

                # Keep the batch dimension, including a final batch of one point.
                input_point = input_point.squeeze(-1)
                output_mu = self.model(input_point).unsqueeze(-1)

                distance = self.cal_distance(output_mu, input_point)
                mse_mu = self.loss_fn(output_mu, label_mu)
                mse_distance = self.loss_fn(distance, label_distance)
                mse_fa, mse_fb = self.cal_loss_fab(
                    output_mu, label_mu, input_point, deterministic=validate)

                if not validate:
                    loss = mse_mu + mse_distance + mse_fa + mse_fb
                    loss.backward()
                    self.optimizer.step()

                size = input_point.shape[0]
                totals += torch.stack((mse_mu, mse_distance, mse_fa, mse_fb)).detach() * size
                count += size

        if not count:
            raise ValueError("Cannot train or validate an empty dataset")
        return tuple((totals / count).tolist())

    def cal_loss_fab(self, output_mu, label_mu, input_point, theta=None, deterministic=False):
        """
        calculate the loss of fa and fb

        fa: -mu^T * G * R^T  ==> lam^T
        fb: mu^T * G * R^T * p - mu^T * h  ==> lam^T * p + mu^T * h
        """

        # fa and fb are linear in mu, so project the residual only once.
        delta_mu = (output_mu - label_mu).squeeze(-1)
        if deterministic:
            # Exact average over the fixed angles 0, pi/2, pi, 3pi/2.
            # Also equals the expectation over a uniform rotation. No RNG or
            # batch-dependent rotation choices enter validation/early stopping.
            normal_squared = (delta_mu @ self.G).square().sum(dim=-1)
            offset = (delta_mu @ self.h).squeeze(-1)
            return (normal_squared.mean() / 2,
                    (normal_squared * input_point.square().sum(dim=-1) / 2
                     + offset.square()).mean())
        if theta is None:
            theta = torch.rand((), device=self.G.device, dtype=self.G.dtype) * (2 * np.pi)
        else:
            theta = torch.as_tensor(theta, device=self.G.device, dtype=self.G.dtype)
        c, s = theta.cos(), theta.sin()
        R = torch.stack((c, -s, s, c)).reshape(2, 2)
        delta_fa = delta_mu @ (-R @ self.G.T).T
        delta_fb = (delta_fa * input_point).sum(dim=-1) + (delta_mu @ self.h).squeeze(-1)

        mse_lamt = delta_fa.square().mean()
        mse_lamtb = delta_fb.square().mean()

        return mse_lamt, mse_lamtb

    def cal_distance(self, mu, input_point):

        temp = input_point @ self.G.T - self.h.squeeze(-1)
        return (mu.squeeze(-1) * temp).sum(dim=-1)

    def print_loss(self, i, epoch, ml, dl, al, bl, vml, vdl, val, vbl, lr, file=None):

        if file is None:
            print(
                "Epoch {}/{}, learning rate {} \n"
                "---------------------------------\n"
                "Losses:\n"
                "  Mu Loss:          {} | Validate Mu Loss:          {}\n"
                "  Distance Loss:    {} | Validate Distance Loss:    {}\n"
                "  Fa Loss:          {} | Validate Fa Loss:          {}\n"
                "  Fb Loss:          {} | Validate Fb Loss:          {}\n".format(
                    i,
                    epoch,
                    lr,
                    str(ml).ljust(10),
                    str(vml).rjust(10),
                    str(dl).ljust(10),
                    str(vdl).rjust(10),
                    str(al).ljust(10),
                    str(val).rjust(10),
                    str(bl).ljust(10),
                    str(vbl).rjust(10),
                )
            )

        else:
            print(
                "Epoch {}/{} learning rate {} \n"
                "---------------------------------\n"
                "Losses:\n"
                "  Mu Loss:          {} | Validate Mu Loss:          {}\n"
                "  Distance Loss:    {} | Validate Distance Loss:    {}\n"
                "  Fa Loss:          {} | Validate Fa Loss:          {}\n"
                "  Fb Loss:          {} | Validate Fb Loss:          {}\n".format(
                    i,
                    epoch,
                    lr,
                    str(ml).ljust(10),
                    str(vml).rjust(10),
                    str(dl).ljust(10),
                    str(vdl).rjust(10),
                    str(al).ljust(10),
                    str(val).rjust(10),
                    str(bl).ljust(10),
                    str(vbl).rjust(10),
                ),
                file=file,
            )

    def test(self, model_pth, train_dict_kwargs=None, data_size_list=(1024,), **kwargs):
        from .artifacts import load_checkpoint
        from .model import ObsPointNet
        if train_dict_kwargs is None:
            saved = load_checkpoint(model_pth)
            if any(not torch.equal(saved["geometry"][key], self.geometry[key]) for key in ("G", "h")):
                raise ValueError("Test geometry differs from checkpoint")
            model = ObsPointNet(2, self.G.shape[0]).to(self.G.device)
            model.load_state_dict(saved["model_state"])
            data_range = saved["config"]["data_range"]
        else:
            with open(train_dict_kwargs, "rb") as f:
                train_dict = pickle.load(f)
            model = train_dict["model"].to(self.G.device)
            model.load_state_dict(torch.load(model_pth, map_location=self.G.device, weights_only=True))
            data_range = train_dict["data_range"]
        model.eval()

        print("dataset generating start ...")

        max_data_size = max(data_size_list)

        start_time = time.time()
        dataset = self.generate_data_set(max_data_size, data_range)
        data_generate_time = time.time() - start_time
        print(
            "data_size:", max_data_size, "dataset generating time: ", data_generate_time
        )

        for data_size in data_size_list:
            test_dataloader = DataLoader(dataset, batch_size=data_size)

            mu_loss_list = []
            distance_loss_list = []
            fa_loss_list = []
            fb_loss_list = []
            inference_time_list = []
            batch_sizes = []

            for input_point, label_mu, label_distance in test_dataloader:
                average_loss_list, inference_time = self.test_one_epoch(
                    model, input_point, label_mu, label_distance, data_size
                )

                mu_loss_list.append(average_loss_list[0])
                distance_loss_list.append(average_loss_list[1])
                fa_loss_list.append(average_loss_list[2])
                fb_loss_list.append(average_loss_list[3])
                inference_time_list.append(inference_time)
                batch_sizes.append(len(input_point))

            avg_mu_loss = np.average(mu_loss_list, weights=batch_sizes)
            avg_distance_loss = np.average(distance_loss_list, weights=batch_sizes)
            avg_fa_loss = np.average(fa_loss_list, weights=batch_sizes)
            avg_fb_loss = np.average(fb_loss_list, weights=batch_sizes)
            avg_inference_time = sum(inference_time_list) / len(inference_time_list)

            with open(os.path.dirname(model_pth) + "/test_results.txt", "a") as f:
                print(
                    "Model_name {}, Data_size {}, inference_time {} \n"
                    "---------------------------------\n"
                    "Losses:\n"
                    "  Mu Loss:          {} \n"
                    "  Distance Loss:    {} \n"
                    "  Fa Loss:          {} \n"
                    "  Fb Loss:          {} \n".format(
                        os.path.basename(model_pth),
                        data_size,
                        avg_inference_time,
                        str(avg_mu_loss).ljust(10),
                        str(avg_distance_loss).ljust(10),
                        str(avg_fa_loss).ljust(10),
                        str(avg_fb_loss).ljust(10),
                    ),
                    file=f,
                )

        print(
            "finish test, the results are saved in {}".format(
                os.path.dirname(model_pth) + "/test_results.txt"
            )
        )

    @torch.no_grad()
    def test_one_epoch(self, model, input_point, label_mu, label_distance, data_size):
        from .run import synchronize
        input_point, label_mu, label_distance = (
            tensor.to(self.G.device) for tensor in (input_point, label_mu, label_distance))
        input_point = input_point.squeeze(-1)

        synchronize(self.G.device)
        start_time = time.perf_counter()
        output_mu = model(input_point)
        synchronize(self.G.device)
        inference_time = time.perf_counter() - start_time

        output_mu = torch.unsqueeze(output_mu, 2)

        distance = self.cal_distance(output_mu, input_point)

        mse_mu = self.loss_fn(output_mu, label_mu)
        mse_distance = self.loss_fn(distance, label_distance)
        mse_fa, mse_fb = self.cal_loss_fab(output_mu, label_mu, input_point, deterministic=True)

        # loss = mse_mu.item() + mse_distance + mse_fa + mse_fb
        # average_loss_list = [mse_mu.item() / data_size, mse_distance.item() / data_size, mse_fa.item() / data_size, mse_fb.item() / data_size]

        loss_list = [mse_mu.item(), mse_distance.item(), mse_fa.item(), mse_fb.item()]

        return loss_list, inference_time
