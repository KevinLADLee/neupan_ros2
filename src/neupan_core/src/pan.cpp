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

#include "neupan/pan.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "neupan/obstacle_prediction.hpp"

namespace neupan {

namespace {

NRMPParams effectiveNrmpParams(const NRMPParams& params,
                               const PAN::Options& opts) {
  NRMPParams effective = params;
  // If DUNE is disabled, its zero fa/fb obstacle subproblem is separable from
  // (s,u). Eliminating it leaves the navigation optimum exactly unchanged.
  if (opts.dune_max_num == 0) effective.max_num = 0;
  return effective;
}

}  // namespace

PAN::PAN(const Robot& robot, const std::string& dune_checkpoint,
         const NRMPParams& nrmp_params, const Options& opts)
    : robot_(robot),
      nrmp_(robot, effectiveNrmpParams(nrmp_params, opts)),
      opts_(opts),
      no_obs_(nrmp_params.max_num == 0 || opts.dune_max_num == 0) {
  if (opts_.dune_max_num < 0)
    throw std::invalid_argument("pan: dune_max_num must be >= 0");
  if (opts_.iter_num < 1)
    throw std::invalid_argument("pan: iter_num must be >= 1");
  if (!no_obs_) dune_.emplace(robot_, dune_checkpoint);
}

PAN::PointFlow PAN::generatePointFlow(const Mat3X& nom_s,
                                      const std::vector<Mat2X>& predicted) const {
  PointFlow pf;
  pf.point_flow.reserve(robot_.T + 1);
  pf.R.reserve(robot_.T + 1);

  for (int i = 0; i <= robot_.T; ++i) {
    const Vec2 trans = nom_s.col(i).head<2>();
    const double theta = nom_s(2, i);
    Mat22 R;
    R << std::cos(theta), -std::sin(theta), std::sin(theta), std::cos(theta);

    pf.point_flow.push_back(
        R.transpose() * (predicted[i].colwise() - trans));
    pf.R.push_back(R);
  }
  return pf;
}

bool PAN::stopCriteria(const Mat3X& nom_s, const Mat2X& nom_u,
                       const DUNE::Output* dune_out) {
  if (!prev_) {
    prev_ = Prev{nom_s, nom_u, {}, {}};
    if (dune_out) {
      prev_->mu_list = dune_out->mu_list;
      prev_->lam_list = dune_out->lam_list;
    }
    return false;
  }

  double diff;
  if (!dune_out || prev_->mu_list.empty()) {
    diff = (nom_s - prev_->nom_s).squaredNorm() +
           (nom_u - prev_->nom_u).squaredNorm();
  } else {
    const int effect_num =
        std::min({static_cast<int>(dune_out->mu_list[0].cols()),
                  static_cast<int>(prev_->mu_list[0].cols()),
                  nrmp_.params().max_num});
    // upstream: norm(cat(mu_list)[:, :effect_num] - prev) / effect_num
    double mu_sq = 0.0, lam_sq = 0.0;
    for (size_t i = 0; i < dune_out->mu_list.size(); ++i) {
      mu_sq += (dune_out->mu_list[i].leftCols(effect_num) -
                prev_->mu_list[i].leftCols(effect_num))
                   .squaredNorm();
      lam_sq += (dune_out->lam_list[i].leftCols(effect_num) -
                 prev_->lam_list[i].leftCols(effect_num))
                    .squaredNorm();
    }
    diff = mu_sq / (static_cast<double>(effect_num) * effect_num) +
           lam_sq / (static_cast<double>(effect_num) * effect_num);
  }

  prev_ = Prev{nom_s, nom_u, {}, {}};
  if (dune_out) {
    prev_->mu_list = dune_out->mu_list;
    prev_->lam_list = dune_out->lam_list;
  }
  return diff < opts_.iter_threshold;
}

PAN::Output PAN::forward(Mat3X nom_s, Mat2X nom_u, const Mat3X& ref_s,
                         const Vec& ref_us, const Mat2X& points,
                         const Mat2X& point_velocities) {
  if (point_velocities.cols() != 0 &&
      point_velocities.cols() != points.cols())
    throw std::invalid_argument(
        "pan: obstacle positions and velocities must have equal columns");

  Output out;
  out.min_distance = std::numeric_limits<double>::infinity();
  // If the very first NRMP solve fails to converge we keep these
  // rather than propagating a garbage iterate from OSQP.
  out.opt_s = nom_s;
  out.opt_u = nom_u;

  // Obstacle motion is independent of the alternating updates to the robot's
  // nominal trajectory. Predict and paired-downsample once per control frame,
  // then only redo the robot-frame transform inside the PAN loop.
  std::vector<Mat2X> predicted_obstacle_points;
  if (!no_obs_ && points.cols() > 0)
    predicted_obstacle_points = predictObstaclePoints(
        points, point_velocities, robot_.T, robot_.dt, opts_.dune_max_num);

  for (int i = 0; i < opts_.iter_num; ++i) {
    std::vector<Mat> fa_list;
    std::vector<Vec> fb_list;
    std::optional<DUNE::Output> dune_out;

    if (!no_obs_ && points.cols() > 0) {
      const PointFlow pf =
          generatePointFlow(nom_s, predicted_obstacle_points);
      dune_out = dune_->forward(
          pf.point_flow, pf.R, predicted_obstacle_points,
          static_cast<std::size_t>(nrmp_.params().max_num));
      out.min_distance = dune_out->min_distance;
      out.dune_points = predicted_obstacle_points[0];
      nrmp_.buildFaFb(dune_out->mu_list, dune_out->lam_list,
                      dune_out->sort_point_list, fa_list, fb_list);
      out.nrmp_points = dune_out->sort_point_list[0].leftCols(
          std::min<int>(nrmp_.params().max_num,
                        static_cast<int>(dune_out->sort_point_list[0].cols())));
    } else {
      nrmp_.buildFaFb({}, {}, {}, fa_list, fb_list);
    }

    const NRMP::Result res =
        nrmp_.solve(nom_s, nom_u, ref_s, ref_us, fa_list, fb_list);

    out.solver_status = res.status;

    // A non-converged OSQP solve can return a point that violates the hard
    // speed/acceleration box. Keep the last accepted plan instead.
    if (!res.success) break;
    out.solved = true;

    nom_s = res.s;
    nom_u = res.u;
    out.opt_s = res.s;
    out.opt_u = res.u;
    out.opt_d = res.d;

    if (stopCriteria(nom_s, nom_u, dune_out ? &*dune_out : nullptr)) break;
  }
  return out;
}

}  // namespace neupan
