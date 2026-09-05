# Bundled solver dependencies

This directory vendors the complete upstream release sources used by the
native NeuPAN runtime:

| Dependency | Version | Commit | License and notices |
| --- | --- | --- | --- |
| OSQP | v1.0.0 | `236713ce9a56c182ac3230d52108f952afce1523` | [Apache-2.0](osqp/LICENSE), [NOTICE](osqp/NOTICE) |
| osqp-eigen | v0.11.2 | `7587e6994dc194cf22511d909bf4cc5d5e0e4eb2` | [BSD-3-Clause](osqp-eigen/LICENSE) |
| QDLDL | v0.1.8 | `138fdac58b9cd1c4137ff1b99152c8108a6cff5b` | [Apache-2.0](qdldl/LICENSE) |
| AMD (included by OSQP) | OSQP release snapshot | — | [BSD-3-Clause](osqp/algebra/_common/lin_sys/qdldl/amd/LICENSE), [OSQP NOTICE](osqp/NOTICE) |

The source files are copied from the upstream release tags without changes.
The added `COLCON_IGNORE` markers keep their nested build metadata from being
misidentified as additional workspace packages. `CMakeLists.txt` and
`package.xml` define the NeuPAN-owned `neupan_solver_vendor` package, which
selects a static, CPU-only configuration.
QDLDL is OSQP's fixed transitive dependency and is also vendored so configuring
the solver never falls back to a network download.

In a colcon workspace, the package installs these license and notice files
under `install/neupan_solver_vendor/share/licenses/`. The standalone command
below uses `install/thirdparty/share/licenses/` instead. In both cases an
installed solver bundle keeps its redistribution notices when copied separately
from the source tree.

Build only these dependencies with:

```bash
./thirdparty/build.sh
```

When the repository is cloned as `ros2_ws/src/neupan_ros2`, a normal
workspace-level `colcon build` discovers `neupan_solver_vendor` and builds it
before `neupan_core`; running this standalone script is not required.
