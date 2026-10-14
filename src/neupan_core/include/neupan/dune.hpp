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

#include <limits>
#include <string>
#include <vector>

#include "neupan/mlp.hpp"
#include "neupan/robot.hpp"
#include "neupan/types.hpp"

namespace neupan {

// Port of neupan/blocks/dune.py: maps the point flow (robot frame) to the
// latent distance features mu and lambda, sorted by objective distance.
class DUNE {
 public:
  DUNE(const Robot& robot, const std::string& checkpoint_path);

  struct Output {
    std::vector<Mat> mu_list;          // T+1 of (E, N), distance-sorted
    std::vector<Mat2X> lam_list;       // T+1 of (2, N)
    std::vector<Mat2X> sort_point_list;  // T+1 of (2, N), global frame
    double min_distance = 0.0;         // min over stage-0 points
  };

  // point_flow / obs_points: T+1 matrices of (2, N); R_list: T+1 rotations.
  Output forward(const std::vector<Mat2X>& point_flow,
                 const std::vector<Mat22>& R_list,
                 const std::vector<Mat2X>& obs_points_list,
                 std::size_t keep_num =
                     std::numeric_limits<std::size_t>::max()) const;

  const MLP& model() const { return mlp_; }

 private:
  MLP mlp_;
  Mat G_;
  Vec h_;
  int T_;
};

}  // namespace neupan
