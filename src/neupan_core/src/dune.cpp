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

#include "neupan/dune.hpp"

#include <algorithm>
#include <cstdio>
#include <limits>
#include <numeric>
#include <stdexcept>

#include "neupan/tensor_io.hpp"

namespace neupan {

namespace {

// mu is only a valid distance for the (G, h) the network was trained against.
void checkCheckpointGeometry(const std::string& path, const TensorFile& tf,
                             const Mat& G, const Vec& h) {
  if (!tf.has("meta.G") || !tf.has("meta.h")) {
    std::fprintf(stderr,
                 "[neupan] WARNING: %s carries no footprint metadata, so it "
                 "cannot be checked against the configured robot. Re-export "
                 "with neupan-export --config ROBOT.yaml to enable the check.\n",
                 path.c_str());
    return;
  }

  const Mat& mG = tf.at("meta.G");
  const Mat& mh = tf.at("meta.h");
  const bool shape_ok =
      mG.rows() == G.rows() && mG.cols() == G.cols() && mh.size() == h.size();
  if (!shape_ok || (mG - G).cwiseAbs().maxCoeff() > 1e-6 ||
      (mh.reshaped() - h).cwiseAbs().maxCoeff() > 1e-6)
    throw std::runtime_error(
        "dune: " + path +
        " was trained for a different robot footprint than the one "
        "configured; distances would be wrong. Retrain or fix robot "
        "vertices or length/width/wheelbase in the planner yaml.");
}

}  // namespace

DUNE::DUNE(const Robot& robot, const std::string& checkpoint_path)
    : G_(robot.G), h_(robot.h), T_(robot.T) {
  const TensorFile tf = TensorFile::load(checkpoint_path);
  mlp_ = MLP::fromTensors(tf);
  if (mlp_.outputDim() != G_.rows())
    throw std::runtime_error("dune: checkpoint edge_dim mismatch with robot G");
  checkCheckpointGeometry(checkpoint_path, tf, G_, h_);
}

DUNE::Output DUNE::forward(const std::vector<Mat2X>& point_flow,
                           const std::vector<Mat22>& R_list,
                           const std::vector<Mat2X>& obs_points_list,
                           std::size_t keep_num) const {
  const size_t stages = point_flow.size();  // T+1
  if (R_list.size() != stages || obs_points_list.size() != stages)
    throw std::invalid_argument("dune: list size mismatch");

  Eigen::Index total = 0;
  for (const auto& p : point_flow) total += p.cols();

  // Batched MLP over all stages in float32, as upstream.
  MatF all_points(2, total);
  Eigen::Index off = 0;
  for (const auto& p : point_flow) {
    all_points.middleCols(off, p.cols()) = p.cast<float>();
    off += p.cols();
  }
  const MatF total_mu = mlp_.forward(all_points);

  Output out;
  out.mu_list.reserve(stages);
  out.lam_list.reserve(stages);
  out.sort_point_list.reserve(stages);
  out.min_distance = std::numeric_limits<double>::infinity();

  off = 0;
  for (size_t i = 0; i < stages; ++i) {
    const Eigen::Index n = point_flow[i].cols();
    const Mat mu = total_mu.middleCols(off, n).cast<double>();
    off += n;

    const Mat2X lam = -R_list[i] * G_.transpose() * mu;

    // distance_k = mu_k . (G p0_k - h)
    Vec distance(n);
    const Mat gp = (G_ * point_flow[i]).colwise() - h_;
    for (Eigen::Index k = 0; k < n; ++k)
      distance(k) = mu.col(k).dot(gp.col(k));

    if (i == 0 && n > 0) out.min_distance = distance.minCoeff();

    std::vector<Eigen::Index> idx(n);
    std::iota(idx.begin(), idx.end(), 0);
    const auto kept = std::min<std::size_t>(keep_num, idx.size());
    const auto distance_order = [&](Eigen::Index a, Eigen::Index b) {
      if (distance(a) == distance(b)) return a < b;
      return distance(a) < distance(b);
    };
    if (kept < idx.size()) {
      std::partial_sort(idx.begin(), idx.begin() + kept, idx.end(),
                        distance_order);
    } else {
      std::sort(idx.begin(), idx.end(), distance_order);
    }

    Mat mu_sorted(mu.rows(), static_cast<Eigen::Index>(kept));
    Mat2X lam_sorted(2, static_cast<Eigen::Index>(kept));
    Mat2X pts_sorted(2, static_cast<Eigen::Index>(kept));
    for (Eigen::Index k = 0; k < static_cast<Eigen::Index>(kept); ++k) {
      mu_sorted.col(k) = mu.col(idx[k]);
      lam_sorted.col(k) = lam.col(idx[k]);
      pts_sorted.col(k) = obs_points_list[i].col(idx[k]);
    }

    out.mu_list.push_back(std::move(mu_sorted));
    out.lam_list.push_back(std::move(lam_sorted));
    out.sort_point_list.push_back(std::move(pts_sorted));
  }

  return out;
}

}  // namespace neupan
