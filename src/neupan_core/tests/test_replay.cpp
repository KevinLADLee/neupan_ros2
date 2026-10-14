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

#include <chrono>
#include <cstdio>

#include "neupan/neupan_planner.hpp"
#include "neupan/tensor_io.hpp"

using namespace neupan;

namespace {
const std::string kData = NEUPAN_TEST_DATA_DIR;
const std::string kModel = std::string(NEUPAN_MODEL_DIR) + "/diff_default.bin";
}

// End-to-end replay against the Python reference: feed the recorded state
// sequence, compare actions. M2 acceptance: per-frame action inf-norm diff
// < 5% of the velocity scale, mean frame time < 5 ms.
TEST(Replay, EndToEndMatchesPython) {
  const TensorFile tf = TensorFile::load(kData + "/replay.nptf");
  const int n_frames = static_cast<int>(tf.at("meta.n_frames")(0, 0));
  const Mat2X points = tf.at("points");

  NeuPANPlanner planner =
      NeuPANPlanner::fromYaml(kData + "/replay_planner.yaml", kModel);

  const double scale = planner.robot().max_speed.maxCoeff() / 2.0;  // 4.0

  double max_action_diff = 0.0, sum_action_diff = 0.0;
  double total_us = 0.0, max_us = 0.0;
  int mismatched_flags = 0, outliers = 0;

  for (int f = 0; f < n_frames; ++f) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "r%02d", f);
    const std::string pre = buf;

    const Vec3 state = tf.at(pre + ".state").col(0);
    const Vec2 action_py = tf.at(pre + ".action").col(0);
    const bool stop_py = tf.at(pre + ".stop")(0, 0) > 0.5;
    const bool arrive_py = tf.at(pre + ".arrive")(0, 0) > 0.5;

    NeuPANPlanner::Info info;
    const auto t0 = std::chrono::steady_clock::now();
    const Vec2 action = planner.forward(state, points, info);
    const double us = std::chrono::duration<double, std::micro>(
                          std::chrono::steady_clock::now() - t0)
                          .count();
    total_us += us;
    max_us = std::max(max_us, us);

    const double diff = (action - action_py).cwiseAbs().maxCoeff();
    max_action_diff = std::max(max_action_diff, diff);
    sum_action_diff += diff;

    // 5% per frame; up to 2 frames may fall in (5%, 10%]: at decision
    // boundaries the PAM stop criterion can run one iteration more or less
    // than the reference (anticipated in the migration plan).
    EXPECT_LT(diff, 0.10 * scale) << pre << " action_cpp=(" << action(0)
                                  << "," << action(1) << ") action_py=("
                                  << action_py(0) << "," << action_py(1)
                                  << ")";
    if (diff >= 0.05 * scale) ++outliers;
    if (info.stop != stop_py || info.arrive != arrive_py) ++mismatched_flags;

    EXPECT_NEAR(info.min_distance, tf.at(pre + ".min_dist")(0, 0), 0.05)
        << pre;

    // Strict open-loop comparison: sync the hidden nominal control sequence
    // to the Python reference so solver tolerance differences don't get
    // amplified through the cur_vel_array feedback.
    planner.setCurVelArray(tf.at(pre + ".opt_u"));
  }

  EXPECT_EQ(mismatched_flags, 0);
  EXPECT_LE(outliers, 2);
  EXPECT_LT(sum_action_diff / n_frames, 0.02 * scale);

  const double mean_ms = total_us / n_frames / 1000.0;
  std::printf(
      "replay: %d frames, action diff mean %.4f / max %.4f (%.1f%% of scale "
      "%.1f), %d outlier(s) >5%%, mean %.2f ms/frame, max %.2f ms\n",
      n_frames, sum_action_diff / n_frames, max_action_diff,
      100.0 * max_action_diff / scale, scale, outliers, mean_ms,
      max_us / 1000.0);
  EXPECT_LT(mean_ms, 5.0);  // M2 acceptance on the dev machine
}

TEST(Replay, ExplicitZeroObstacleVelocityPreservesStaticApi) {
  const TensorFile tf = TensorFile::load(kData + "/replay.nptf");
  const Mat2X points = tf.at("points");
  const Vec3 state = tf.at("r00.state").col(0);

  NeuPANPlanner legacy =
      NeuPANPlanner::fromYaml(kData + "/replay_planner.yaml", kModel);
  NeuPANPlanner dynamic =
      NeuPANPlanner::fromYaml(kData + "/replay_planner.yaml", kModel);

  NeuPANPlanner::Info legacy_info;
  NeuPANPlanner::Info dynamic_info;
  const Vec2 legacy_action = legacy.forward(state, points, legacy_info);
  const Mat2X zero_velocity = Mat2X::Zero(2, points.cols());
  const Vec2 dynamic_action =
      dynamic.forward(state, points, zero_velocity, dynamic_info);

  EXPECT_LT((legacy_action - dynamic_action).norm(), 1e-12);
  EXPECT_LT((legacy_info.opt_s - dynamic_info.opt_s).norm(), 1e-12);
  EXPECT_LT((legacy_info.opt_u - dynamic_info.opt_u).norm(), 1e-12);
  EXPECT_DOUBLE_EQ(legacy_info.min_distance, dynamic_info.min_distance);
}
