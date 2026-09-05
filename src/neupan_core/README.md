# neupan_core

ROS-independent C++17 implementation of NeuPAN for CPU deployment.

The public entry point is `neupan::NeuPANPlanner`. DUNE inference uses Eigen,
and NRMP is assembled directly as an OSQP quadratic program. The sparse QP
pattern is created once and subsequent planning cycles update only numerical
values, gradients and bounds.

The core performs no coordinate transform. State, obstacle points, obstacle
velocities and initial path must use one common right-handed planar world frame
with SI units. See the [coordinate-frame contract](../../docs/coordinate_frames.md).

Build options:

- `BUILD_TESTING=ON`: build C++ parity tests.
- `NEUPAN_BUILD_TOOLS=ON`: build benchmarks and developer tools.
- `NEUPAN_NATIVE=ON`: enable host-specific CPU instructions.
