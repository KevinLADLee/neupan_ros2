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

#include <gtest/gtest.h>

#include <cstdio>

#include "neupan/nrmp.hpp"
#include "neupan/robot.hpp"
#include "neupan/tensor_io.hpp"

using namespace neupan;

namespace {
const std::string kData = NEUPAN_TEST_DATA_DIR;
}

// Replays every dumped NRMP call (each frame, each PAN iteration) against
// the original cvxpy/ECOS reference. In addition to the deployed action and
// objective, retain bounds on the complete primal solution and feasibility.
TEST(Nrmp, SolveMatchesCvxpy) {
  constexpr double kBoxTolerance = 5e-4;
  const TensorFile tf = TensorFile::load(kData + "/frames.nptf");

  const int T = static_cast<int>(tf.at("meta.T")(0, 0));
  const int n_frames = static_cast<int>(tf.at("meta.n_frames")(0, 0));

  const Robot robot =
      Robot::diffRectangle(T, tf.at("meta.dt")(0, 0), Vec2(8, 1), Vec2(8, 3),
                           1.6, 2.0);

  NRMPParams params;
  params.max_num = static_cast<int>(tf.at("meta.max_num")(0, 0));
  params.q_s = tf.at("meta.q_s").col(0);
  params.p_u = tf.at("meta.p_u")(0, 0);
  params.eta = tf.at("meta.eta")(0, 0);
  params.d_max = tf.at("meta.d_max")(0, 0);
  params.d_min = tf.at("meta.d_min")(0, 0);
  params.ro_obs = tf.at("meta.ro_obs")(0, 0);
  params.bk = tf.at("meta.bk")(0, 0);

  NRMP nrmp(robot, params);

  double max_action_diff = 0.0, max_obj_rel = 0.0, max_u0_scale = 0.0;
  double max_state_diff = 0.0, max_control_diff = 0.0, max_distance_diff = 0.0;
  double max_dynamics_residual = 0.0;
  int n_solves = 0;

  for (int f = 0; f < n_frames; ++f) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "f%02d", f);
    const int n_iters =
        static_cast<int>(tf.at(std::string(buf) + ".n_iters")(0, 0));

    for (int it = 0; it < n_iters; ++it) {
      std::snprintf(buf, sizeof(buf), "f%02d_it%d", f, it);
      const std::string pre = buf;

      const Mat3X nom_s = tf.at(pre + ".nom_s");
      const Mat2X nom_u = tf.at(pre + ".nom_u");
      const Mat3X ref_s = tf.at(pre + ".ref_s");
      const Vec ref_us = tf.at(pre + ".ref_us").col(0);

      // Rebuild fa/fb from the dumped (sorted) DUNE outputs, exercising
      // buildFaFb the same way the upstream NRMP consumes mu/lam/points.
      std::vector<Mat> mu_list;
      std::vector<Mat2X> lam_list, sp_list;
      for (int t = 0; t <= T; ++t) {
        std::snprintf(buf, sizeof(buf), "_s%02d", t);
        mu_list.push_back(tf.at(pre + ".mu" + buf));
        lam_list.push_back(tf.at(pre + ".lam" + buf));
        sp_list.push_back(tf.at(pre + ".sp" + buf));
      }
      std::vector<Mat> fa_list;
      std::vector<Vec> fb_list;
      nrmp.buildFaFb(mu_list, lam_list, sp_list, fa_list, fb_list);

      const NRMP::Result res =
          nrmp.solve(nom_s, nom_u, ref_s, ref_us, fa_list, fb_list);
      ASSERT_TRUE(res.success) << pre;
      ++n_solves;

      const Mat3X s_py = tf.at(pre + ".opt_s");
      const Mat2X u_py = tf.at(pre + ".opt_u");
      const Vec d_py = tf.at(pre + ".opt_d").row(0).transpose();

      max_state_diff = std::max(
          max_state_diff, (res.s - s_py).cwiseAbs().maxCoeff());
      max_control_diff = std::max(
          max_control_diff, (res.u - u_py).cwiseAbs().maxCoeff());
      max_distance_diff = std::max(
          max_distance_diff, (res.d - d_py).cwiseAbs().maxCoeff());
      EXPECT_LT((res.s.col(0) - nom_s.col(0)).cwiseAbs().maxCoeff(),
                kBoxTolerance)
          << pre;
      EXPECT_LE(res.u.cwiseAbs().rowwise().maxCoeff()(0),
                robot.max_speed(0) + kBoxTolerance)
          << pre;
      EXPECT_LE(res.u.cwiseAbs().rowwise().maxCoeff()(1),
                robot.max_speed(1) + kBoxTolerance)
          << pre;
      EXPECT_GE(res.d.minCoeff(), params.d_min - kBoxTolerance) << pre;
      EXPECT_LE(res.d.maxCoeff(), params.d_max + kBoxTolerance) << pre;
      for (int t = 0; t < T; ++t) {
        Mat33 A;
        Mat32 B;
        Vec3 C;
        robot.linearize(nom_s.col(t), nom_u.col(t), A, B, C);
        max_dynamics_residual = std::max(
            max_dynamics_residual,
            (res.s.col(t + 1) - A * res.s.col(t) - B * res.u.col(t) - C)
                .cwiseAbs()
                .maxCoeff());
        if (t + 1 < T)
          EXPECT_TRUE(((res.u.col(t + 1) - res.u.col(t)).cwiseAbs().array() <=
                       (robot.acce_bound.array() + kBoxTolerance))
                          .all())
              << pre;
      }

      // action = first control column
      const double action_diff =
          (res.u.col(0) - u_py.col(0)).cwiseAbs().maxCoeff();
      max_action_diff = std::max(max_action_diff, action_diff);
      max_u0_scale = std::max(max_u0_scale, u_py.col(0).cwiseAbs().maxCoeff());

      const double obj_cpp = res.objective;
      const double obj_py = nrmp.objective(s_py, u_py, d_py, nom_s, ref_s,
                                           ref_us, fa_list, fb_list);
      const double obj_rel =
          std::abs(obj_cpp - obj_py) / std::max(1.0, std::abs(obj_py));
      max_obj_rel = std::max(max_obj_rel, obj_rel);

      EXPECT_LT(action_diff, 0.05 * std::max(1.0, max_u0_scale)) << pre;
      EXPECT_LT(obj_rel, 0.01) << pre;
      // C++ optimum must not be worse than the reference solution.
      EXPECT_LT(obj_cpp, obj_py + 0.01 * std::max(1.0, std::abs(obj_py)))
          << pre;
    }
  }

  std::printf(
      "NRMP: %d solves, max action diff %.4f (scale %.3f), "
      "max objective rel diff %.3e; full s/u/d %.4f/%.4f/%.4f, "
      "dynamics residual %.3e\n",
      n_solves, max_action_diff, max_u0_scale, max_obj_rel, max_state_diff,
      max_control_diff, max_distance_diff, max_dynamics_residual);

  // ECOS and OSQP can choose different points within solver tolerance, in
  // particular for later angular controls. These limits detect formulation or
  // indexing regressions without pretending the solvers are bit-identical.
  EXPECT_LT(max_state_diff, 0.1);
  EXPECT_LT(max_control_diff, 0.6);
  EXPECT_LT(max_distance_diff, 1e-3);
  EXPECT_LT(max_dynamics_residual, 1e-5);
}
