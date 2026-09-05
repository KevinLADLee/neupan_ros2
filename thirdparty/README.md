# Bundled solver dependencies

This directory vendors the complete upstream release sources used by the
native NeuPAN runtime:

| Dependency | Version | Commit | License |
| --- | --- | --- | --- |
| OSQP | v1.0.0 | `236713ce9a56c182ac3230d52108f952afce1523` | Apache-2.0 |
| osqp-eigen | v0.11.2 | `7587e6994dc194cf22511d909bf4cc5d5e0e4eb2` | BSD-3-Clause |
| QDLDL | v0.1.8 | `138fdac58b9cd1c4137ff1b99152c8108a6cff5b` | Apache-2.0 |

The sources are copied from the upstream release tags without source changes.
`CMakeLists.txt` is the NeuPAN-owned superbuild that selects a static, CPU-only
configuration and installs both packages under `install/thirdparty`.
QDLDL is OSQP's fixed transitive dependency and is also vendored so configuring
the solver never falls back to a network download.

Build only these dependencies with:

```bash
./thirdparty/build.sh
```

`COLCON_IGNORE` prevents colcon from treating the vendored osqp-eigen
`package.xml` as a fourth workspace package.
