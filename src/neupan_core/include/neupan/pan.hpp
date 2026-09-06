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

#include <optional>
#include <string>
#include <vector>

#include "neupan/dune.hpp"
#include "neupan/nrmp.hpp"
#include "neupan/robot.hpp"
#include "neupan/types.hpp"

namespace neupan {

// Port of neupan/blocks/pan.py: the proximal alternating-minimization
// outer loop coupling DUNE and NRMP.
class PAN {
 public:
  struct Options {
    int iter_num = 2;
    int dune_max_num = 100;
    double iter_threshold = 0.1;
  };

  PAN(const Robot& robot, const std::string& dune_checkpoint,
      const NRMPParams& nrmp_params, const Options& opts);

  struct Output {
    Mat3X opt_s;
    Mat2X opt_u;
    Vec opt_d;
    double min_distance;  // min DUNE distance at stage 0 (inf if no points)
    Mat2X dune_points;    // downsampled obstacle points fed to DUNE (stage 0)
    Mat2X nrmp_points;    // closest points used by NRMP (visualization)
    // False: no solve converged, so opt_u is the nominal with no avoidance.
    bool solved = false;
    int solver_status = -1;  // raw OsqpEigen::Status of the last solve
  };

  // points/point_velocities: paired (2, N) global-frame columns. An empty
  // velocity matrix means static points.
  Output forward(Mat3X nom_s, Mat2X nom_u, const Mat3X& ref_s,
                 const Vec& ref_us, const Mat2X& points,
                 const Mat2X& point_velocities = Mat2X(2, 0));

  void reset() { prev_.reset(); }
  NRMP& nrmp() { return nrmp_; }

 private:
  struct PointFlow {
    std::vector<Mat2X> point_flow;
    std::vector<Mat22> R;
  };
  PointFlow generatePointFlow(
      const Mat3X& nom_s,
      const std::vector<Mat2X>& predicted_obstacle_points) const;
  bool stopCriteria(const Mat3X& nom_s, const Mat2X& nom_u,
                    const DUNE::Output* dune_out);

  Robot robot_;
  // DUNE is absent in navigation-only mode. This also removes the model-file
  // dependency from CPU deployments configured with either point limit zero.
  std::optional<DUNE> dune_;
  NRMP nrmp_;
  Options opts_;
  bool no_obs_;

  // previous iteration values for the stop criterion (persists across
  // frames, as upstream)
  struct Prev {
    Mat3X nom_s;
    Mat2X nom_u;
    std::vector<Mat> mu_list;
    std::vector<Mat2X> lam_list;
  };
  std::optional<Prev> prev_;
};

}  // namespace neupan
