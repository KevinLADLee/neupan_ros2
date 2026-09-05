# NeuPAN training

This directory is an offline Python project. It is intentionally outside the
ROS 2 workspace and is not a runtime dependency of `neupan_core` or
`neupan_ros`.

```bash
python -m venv .venv
. .venv/bin/activate
pip install -e ./training

neupan-train --output runs/diff --length 0.5 --width 0.5
neupan-export runs/diff/model_5000.pth src/neupan_core/models/diff.bin \
  --length 0.5 --width 0.5
```

The exported NPTF binary stores the MLP tensors and the footprint matrices
needed by the native C++ runtime.
