/*
 * neupan_cpp: C++ port of the NeuPAN planner.
 *
 * Ported from NeuPAN (https://github.com/hanruihua/NeuPAN),
 * Copyright (c) 2025 Ruihua Han <hanrh@connect.hku.hk>.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version. See <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <string>
#include <vector>

#include "neupan/types.hpp"

namespace neupan {

struct TensorFile;

// Sequential MLP equivalent to neupan ObsPointNet (obs_point_net.py).
// Runs in float32 to match PyTorch numerics.
class MLP {
 public:
  enum class LayerType { Linear, LayerNorm, Tanh, ReLU };

  struct Layer {
    LayerType type;
    MatF W;      // Linear: (out, in)
    VecF b;      // Linear bias
    VecF gamma;  // LayerNorm scale
    VecF beta;   // LayerNorm shift
  };

  // Load from a NPTF weight file written by tools/export_dune_weights.py.
  // Record names: "L<i>.linear.weight" / "L<i>.linear.bias" /
  // "L<i>.ln.gamma" / "L<i>.ln.beta" / "L<i>.tanh" / "L<i>.relu".
  static MLP load(const std::string& path);
  // Same, from an already-parsed file (callers that also read other records).
  static MLP fromTensors(const TensorFile& tf);

  // X: (in_dim, N) column-per-sample -> (out_dim, N)
  MatF forward(const MatF& X) const;

  int inputDim() const;
  int outputDim() const;

  std::vector<Layer> layers;
  static constexpr float kLayerNormEps = 1e-5f;  // torch default
};

}  // namespace neupan
