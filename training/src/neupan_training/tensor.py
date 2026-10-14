"""Small tensor conversion helpers used by the offline trainer."""

import numpy as np
import torch

DEVICE = torch.device("cpu")
TENSOR_DTYPE = torch.float32


def np_to_tensor(array, requires_grad=False):
    if np.isscalar(array):
        output = torch.tensor(
            array, dtype=TENSOR_DTYPE, requires_grad=requires_grad)
    else:
        output = torch.from_numpy(np.asarray(array)).to(dtype=TENSOR_DTYPE)
        if requires_grad:
            output.requires_grad_()
    return output.to(DEVICE)


def value_to_tensor(value, requires_grad=False):
    if value is None:
        return None
    return torch.tensor(
        value, dtype=TENSOR_DTYPE, requires_grad=requires_grad,
        device=DEVICE)


def to_device(tensor):
    return tensor.to(DEVICE)
