"""
NPTF named-tensor container writer, shared by the export / dump tools.

Part of neupan_cpp, a C++ port of NeuPAN (Copyright (c) 2025 Ruihua Han),
distributed under the GNU General Public License v3 or later.
"""

import struct

import numpy as np

DTYPE_F32 = 0
DTYPE_F64 = 1


def write_nptf(path, records):
    """records: list of (name: str, array: np.ndarray)

    float32 arrays are stored as f32, everything else as f64.
    Arrays are stored 2-D row-major; 1-D arrays become column vectors.
    """
    with open(path, "wb") as f:
        f.write(b"NPTF")
        f.write(struct.pack("<II", 1, len(records)))
        for name, arr in records:
            arr = np.asarray(arr)
            if arr.ndim == 0:
                arr = arr.reshape(1, 1)
            elif arr.ndim == 1:
                arr = arr.reshape(-1, 1)
            elif arr.ndim != 2:
                raise ValueError(f"{name}: only 0/1/2-D arrays supported")

            if arr.dtype == np.float32:
                dtype = DTYPE_F32
            else:
                dtype = DTYPE_F64
                arr = arr.astype(np.float64)

            name_b = name.encode()
            f.write(struct.pack("<I", len(name_b)))
            f.write(name_b)
            f.write(struct.pack("<III", dtype, arr.shape[0], arr.shape[1]))
            f.write(np.ascontiguousarray(arr).tobytes())
