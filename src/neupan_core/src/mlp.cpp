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

#include "neupan/mlp.hpp"

#include <stdexcept>

#include "neupan/tensor_io.hpp"

namespace neupan {

MLP MLP::load(const std::string& path) { return fromTensors(TensorFile::load(path)); }

MLP MLP::fromTensors(const TensorFile& tf) {
  MLP mlp;
  Layer cur{};
  bool pending = false;
  std::string pendingPrefix;

  auto flush = [&]() {
    if (pending) {
      mlp.layers.push_back(cur);
      cur = Layer{};
      pending = false;
    }
  };

  for (const auto& name : tf.order) {
    const auto dot = name.find('.');
    if (dot == std::string::npos)
      throw std::runtime_error("mlp: bad record name " + name);
    const std::string prefix = name.substr(0, dot);  // "L<i>"
    const std::string rest = name.substr(dot + 1);   // e.g. "linear.weight"

    // Non-layer records (e.g. the trained footprint) belong to other readers.
    if (prefix == "meta") continue;

    if (pending && prefix != pendingPrefix) flush();
    pendingPrefix = prefix;

    const Mat& m = tf.at(name);
    if (rest == "linear.weight") {
      cur.type = LayerType::Linear;
      cur.W = m.cast<float>();
      pending = true;
    } else if (rest == "linear.bias") {
      cur.b = m.cast<float>().reshaped().eval();
      pending = true;
    } else if (rest == "ln.gamma") {
      cur.type = LayerType::LayerNorm;
      cur.gamma = m.cast<float>().reshaped().eval();
      pending = true;
    } else if (rest == "ln.beta") {
      cur.beta = m.cast<float>().reshaped().eval();
      pending = true;
    } else if (rest == "tanh") {
      cur.type = LayerType::Tanh;
      pending = true;
    } else if (rest == "relu") {
      cur.type = LayerType::ReLU;
      pending = true;
    } else {
      throw std::runtime_error("mlp: unknown record " + name);
    }
  }
  flush();

  if (mlp.layers.empty())
    throw std::runtime_error("mlp: checkpoint has no layer records");
  return mlp;
}

MatF MLP::forward(const MatF& X) const {
  MatF h = X;
  for (const auto& layer : layers) {
    switch (layer.type) {
      case LayerType::Linear:
        h = (layer.W * h).colwise() + layer.b;
        break;
      case LayerType::LayerNorm: {
        // Per-sample (column) normalization, biased variance, torch semantics.
        const Eigen::Index dim = h.rows();
        for (Eigen::Index c = 0; c < h.cols(); ++c) {
          const float mean = h.col(c).mean();
          const float var = (h.col(c).array() - mean).square().sum() / dim;
          h.col(c) = (((h.col(c).array() - mean) /
                       std::sqrt(var + kLayerNormEps)) *
                          layer.gamma.array() +
                      layer.beta.array())
                         .matrix();
        }
        break;
      }
      case LayerType::Tanh:
        h = h.array().tanh().matrix();
        break;
      case LayerType::ReLU:
        h = h.cwiseMax(0.0f);
        break;
    }
  }
  return h;
}

int MLP::inputDim() const {
  for (const auto& l : layers)
    if (l.type == LayerType::Linear) return static_cast<int>(l.W.cols());
  return -1;
}

int MLP::outputDim() const {
  for (auto it = layers.rbegin(); it != layers.rend(); ++it)
    if (it->type == LayerType::Linear) return static_cast<int>(it->W.rows());
  return -1;
}

}  // namespace neupan
