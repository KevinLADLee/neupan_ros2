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

#include <algorithm>
#include <cstdio>

#include "neupan/dune.hpp"
#include "neupan/mlp.hpp"
#include "neupan/robot.hpp"
#include "neupan/tensor_io.hpp"

using namespace neupan;

namespace {

const std::string kData = NEUPAN_TEST_DATA_DIR;
const std::string kModel = std::string(NEUPAN_MODEL_DIR) + "/diff_default.bin";

Robot lonRobot() {
  // example/LON/planner.yaml: diff, max_speed [8,1], max_acce [8,3],
  // length 1.6, width 2.0, T=10, dt=0.1
  return Robot::diffRectangle(10, 0.1, Vec2(8, 1), Vec2(8, 3), 1.6, 2.0);
}

}  // namespace

TEST(Dune, MlpMatchesPyTorch) {
  const MLP mlp = MLP::load(kModel);
  EXPECT_EQ(mlp.inputDim(), 2);
  EXPECT_EQ(mlp.outputDim(), 4);

  const TensorFile tf = TensorFile::load(kData + "/dune_mlp.nptf");
  const MatF points = tf.at("points").cast<float>();
  const Mat mu_py = tf.at("mu");

  const MatF mu_cpp = mlp.forward(points);
  ASSERT_EQ(mu_cpp.rows(), mu_py.rows());
  ASSERT_EQ(mu_cpp.cols(), mu_py.cols());

  const double max_diff =
      (mu_cpp.cast<double>() - mu_py).cwiseAbs().maxCoeff();
  std::printf("MLP max|mu_cpp - mu_py| = %.3e over %ld points\n", max_diff,
              static_cast<long>(mu_py.cols()));
  EXPECT_LT(max_diff, 1e-4);  // M1 acceptance
}

TEST(Dune, RobotGhMatchesUpstream) {
  const TensorFile tf = TensorFile::load(kData + "/frames.nptf");
  const Robot robot = lonRobot();
  EXPECT_LT((robot.G - tf.at("meta.G")).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LT((robot.h - tf.at("meta.h")).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LT((robot.max_speed - tf.at("meta.max_speed")).cwiseAbs().maxCoeff(),
            1e-12);
  EXPECT_LT(
      (robot.acce_bound - tf.at("meta.acce_bound")).cwiseAbs().maxCoeff(),
      1e-12);
}

TEST(Dune, ForwardMatchesPython) {
  const TensorFile tf = TensorFile::load(kData + "/frames.nptf");
  const Robot robot = lonRobot();
  const DUNE dune(robot, kModel);

  const int T = static_cast<int>(tf.at("meta.T")(0, 0));
  const int n_frames = static_cast<int>(tf.at("meta.n_frames")(0, 0));

  double max_mu_diff = 0.0, max_lam_diff = 0.0, max_pt_diff = 0.0;

  for (int f = 0; f < n_frames; ++f) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "f%02d", f);
    const int n_iters =
        static_cast<int>(tf.at(std::string(buf) + ".n_iters")(0, 0));

    for (int it = 0; it < n_iters; ++it) {
      std::snprintf(buf, sizeof(buf), "f%02d_it%d", f, it);
      const std::string pre = buf;

      std::vector<Mat2X> pf, op;
      std::vector<Mat22> R;
      for (int t = 0; t <= T; ++t) {
        std::snprintf(buf, sizeof(buf), "_s%02d", t);
        pf.push_back(tf.at(pre + ".pf" + buf));
        R.push_back(tf.at(pre + ".R" + buf));
        op.push_back(tf.at(pre + ".op" + buf));
      }

      const DUNE::Output out = dune.forward(pf, R, op);

      EXPECT_NEAR(out.min_distance, tf.at(pre + ".min_dist")(0, 0), 1e-4);
      for (int t = 0; t <= T; ++t) {
        std::snprintf(buf, sizeof(buf), "_s%02d", t);
        max_mu_diff = std::max(
            max_mu_diff, (out.mu_list[t] - tf.at(pre + ".mu" + buf))
                             .cwiseAbs()
                             .maxCoeff());
        max_lam_diff = std::max(
            max_lam_diff, (out.lam_list[t] - tf.at(pre + ".lam" + buf))
                              .cwiseAbs()
                              .maxCoeff());
        max_pt_diff = std::max(
            max_pt_diff, (out.sort_point_list[t] - tf.at(pre + ".sp" + buf))
                             .cwiseAbs()
                             .maxCoeff());
      }
    }
  }

  std::printf("DUNE forward: max mu diff %.3e, lam diff %.3e, point diff %.3e\n",
              max_mu_diff, max_lam_diff, max_pt_diff);
  EXPECT_LT(max_mu_diff, 1e-4);
  EXPECT_LT(max_lam_diff, 1e-3);
  // identical sorting => identical point selection
  EXPECT_LT(max_pt_diff, 1e-9);
}

TEST(Dune, TopKMatchesFullSortPrefix) {
  const TensorFile tf = TensorFile::load(kData + "/frames.nptf");
  const Robot robot = lonRobot();
  const DUNE dune(robot, kModel);
  const int T = static_cast<int>(tf.at("meta.T")(0, 0));

  std::vector<Mat2X> point_flow;
  std::vector<Mat2X> obstacle_points;
  std::vector<Mat22> rotations;
  for (int t = 0; t <= T; ++t) {
    char suffix[16];
    std::snprintf(suffix, sizeof(suffix), "_s%02d", t);
    point_flow.push_back(tf.at(std::string("f00_it0.pf") + suffix));
    rotations.push_back(tf.at(std::string("f00_it0.R") + suffix));
    obstacle_points.push_back(tf.at(std::string("f00_it0.op") + suffix));
  }

  const DUNE::Output full =
      dune.forward(point_flow, rotations, obstacle_points);
  const DUNE::Output top =
      dune.forward(point_flow, rotations, obstacle_points, 10);

  EXPECT_DOUBLE_EQ(top.min_distance, full.min_distance);
  for (int t = 0; t <= T; ++t) {
    const Eigen::Index kept = std::min<Eigen::Index>(10, full.mu_list[t].cols());
    ASSERT_EQ(top.mu_list[t].cols(), kept);
    EXPECT_LT((top.mu_list[t] - full.mu_list[t].leftCols(kept))
                  .cwiseAbs()
                  .maxCoeff(),
              1e-12);
    EXPECT_LT((top.lam_list[t] - full.lam_list[t].leftCols(kept))
                  .cwiseAbs()
                  .maxCoeff(),
              1e-12);
    EXPECT_LT((top.sort_point_list[t] -
               full.sort_point_list[t].leftCols(kept))
                  .cwiseAbs()
                  .maxCoeff(),
              1e-12);
  }
}
