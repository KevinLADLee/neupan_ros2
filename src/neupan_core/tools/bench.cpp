/*
 * neupan_cpp: C++ port of the NeuPAN planner.
 *
 * Benchmark: replays the dumped reference frames through DUNE and NRMP and
 * reports per-call wall time.
 *
 * Ported from NeuPAN (https://github.com/hanruihua/NeuPAN),
 * Copyright (c) 2025 Ruihua Han <hanrh@connect.hku.hk>.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version. See <https://www.gnu.org/licenses/>.
 */

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <string>
#include <vector>

#include "neupan/dune.hpp"
#include "neupan/nrmp.hpp"
#include "neupan/robot.hpp"
#include "neupan/tensor_io.hpp"

using namespace neupan;
using Clock = std::chrono::steady_clock;

namespace {

double usSince(Clock::time_point t0) {
  return std::chrono::duration<double, std::micro>(Clock::now() - t0).count();
}

void stats(const char* name, std::vector<double>& us) {
  std::sort(us.begin(), us.end());
  double sum = 0;
  for (double v : us) sum += v;
  std::printf("%-28s n=%4zu  mean %8.1f us   median %8.1f us   p95 %8.1f us\n",
              name, us.size(), sum / us.size(), us[us.size() / 2],
              us[static_cast<size_t>(us.size() * 0.95)]);
}

}  // namespace

int main() {
  const std::string data = NEUPAN_TEST_DATA_DIR;
  const std::string model = std::string(NEUPAN_MODEL_DIR) + "/diff_default.bin";
  const TensorFile tf = TensorFile::load(data + "/frames.nptf");

  const int T = static_cast<int>(tf.at("meta.T")(0, 0));
  const int n_frames = static_cast<int>(tf.at("meta.n_frames")(0, 0));

  const Robot robot = Robot::diffRectangle(T, tf.at("meta.dt")(0, 0),
                                           Vec2(8, 1), Vec2(8, 3), 1.6, 2.0);
  const DUNE dune(robot, model);

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

  struct Call {
    std::vector<Mat2X> pf, op, lam, sp;
    std::vector<Mat22> R;
    std::vector<Mat> mu;
    Mat3X nom_s, ref_s;
    Mat2X nom_u;
    Vec ref_us;
  };
  std::vector<Call> calls;

  char buf[64];
  for (int f = 0; f < n_frames; ++f) {
    std::snprintf(buf, sizeof(buf), "f%02d", f);
    const int n_iters =
        static_cast<int>(tf.at(std::string(buf) + ".n_iters")(0, 0));
    for (int it = 0; it < n_iters; ++it) {
      std::snprintf(buf, sizeof(buf), "f%02d_it%d", f, it);
      const std::string pre = buf;
      Call c;
      c.nom_s = tf.at(pre + ".nom_s");
      c.nom_u = tf.at(pre + ".nom_u");
      c.ref_s = tf.at(pre + ".ref_s");
      c.ref_us = tf.at(pre + ".ref_us").col(0);
      for (int t = 0; t <= T; ++t) {
        std::snprintf(buf, sizeof(buf), "_s%02d", t);
        c.pf.push_back(tf.at(pre + ".pf" + buf));
        c.R.push_back(tf.at(pre + ".R" + buf));
        c.op.push_back(tf.at(pre + ".op" + buf));
        c.mu.push_back(tf.at(pre + ".mu" + buf));
        c.lam.push_back(tf.at(pre + ".lam" + buf));
        c.sp.push_back(tf.at(pre + ".sp" + buf));
      }
      calls.push_back(std::move(c));
    }
  }

  const Eigen::Index pts_per_call = calls[0].pf[0].cols() * (T + 1);
  std::printf("replaying %zu dumped PAM iterations, %ld points per DUNE call\n",
              calls.size(), static_cast<long>(pts_per_call));

  constexpr int kRepeat = 100;
  std::vector<double> dune_us, nrmp_us, frame_us;

  // warm-up (first OSQP init included here, excluded from stats)
  {
    std::vector<Mat> fa;
    std::vector<Vec> fb;
    nrmp.buildFaFb(calls[0].mu, calls[0].lam, calls[0].sp, fa, fb);
    (void)nrmp.solve(calls[0].nom_s, calls[0].nom_u, calls[0].ref_s,
                     calls[0].ref_us, fa, fb);
  }

  for (int rep = 0; rep < kRepeat; ++rep) {
    for (const Call& c : calls) {
      const auto t0 = Clock::now();
      const DUNE::Output out =
          dune.forward(c.pf, c.R, c.op,
                       static_cast<std::size_t>(params.max_num));
      dune_us.push_back(usSince(t0));

      const auto t1 = Clock::now();
      std::vector<Mat> fa;
      std::vector<Vec> fb;
      nrmp.buildFaFb(out.mu_list, out.lam_list, out.sort_point_list, fa, fb);
      const NRMP::Result res =
          nrmp.solve(c.nom_s, c.nom_u, c.ref_s, c.ref_us, fa, fb);
      nrmp_us.push_back(usSince(t1));
      frame_us.push_back(usSince(t0));
      if (!res.success) std::printf("warning: solve failed\n");
    }
  }

  stats("DUNE forward + top-M", dune_us);
  stats("NRMP buildFaFb + solve", nrmp_us);
  stats("PAM iteration (DUNE+NRMP)", frame_us);
  std::printf("\nnote: one planner frame = iter_num (2) PAM iterations\n");
  return 0;
}
