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

// A (season d=0 config) must be rejected at load; B (upstream margins) must pass.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <iterator>
#include <string>
#include <vector>

#include "neupan/neupan_planner.hpp"

using namespace neupan;

namespace {

constexpr double kObstacleX = 3.0;     // obstacle centre on the reference line
constexpr double kObstacleHalf = 0.2;  // half width/depth of the point box
constexpr double kGoalX = 6.0;
constexpr int kMaxSteps = 300;  // 30 s at step_time 0.1

// Square outline centred at (kObstacleX, offset_y), a point every 5 cm.
Mat2X obstaclePoints(double offset_y) {
  std::vector<Vec2> pts;
  for (double t = -kObstacleHalf; t <= kObstacleHalf + 1e-9; t += 0.05) {
    pts.emplace_back(kObstacleX - kObstacleHalf, offset_y + t);
    pts.emplace_back(kObstacleX + kObstacleHalf, offset_y + t);
    pts.emplace_back(kObstacleX + t, offset_y - kObstacleHalf);
    pts.emplace_back(kObstacleX + t, offset_y + kObstacleHalf);
  }
  Mat2X out(2, static_cast<int>(pts.size()));
  for (size_t i = 0; i < pts.size(); ++i) out.col(static_cast<int>(i)) = pts[i];
  return out;
}

struct Result {
  bool arrived = false;
  int steps = 0;
  int stop_frames = 0;
  double travelled_x = 0.0;   // how far along the reference the robot got
  double max_detour_y = 0.0;  // peak lateral excursion: the detour itself
  double min_gap = 1e9;       // closest approach to the obstacle box
};

Result rollout(const std::string& yaml, const std::string& model,
               double offset_y, bool verbose) {
  const Mat2X points = obstaclePoints(offset_y);
  NeuPANPlanner planner = NeuPANPlanner::fromYaml(yaml, model);
  const double dt = planner.config().step_time;

  Vec3 state(0.0, 0.0, 0.0);
  planner.updateInitialPathFromGoal(state, Vec3(kGoalX, 0.0, 0.0));

  Result r;
  for (int i = 0; i < kMaxSteps; ++i) {
    NeuPANPlanner::Info info;
    const Vec2 u = planner.forward(state, points, info);
    ++r.steps;
    if (info.stop) ++r.stop_frames;
    if (verbose && i % 10 == 0) {
      std::printf("    t=%4.1f s  pose=(%5.2f,%5.2f,%5.2f)  u=(%5.2f,%5.2f)  "
                  "min_dist=%6.3f %s\n",
                  i * dt, state(0), state(1), state(2), u(0), u(1),
                  info.min_distance, info.stop ? "STOP" : "");
    }
    if (info.arrive) {
      r.arrived = true;
      break;
    }

    // Unicycle integration; the planner already clamps to the speed box.
    state(0) += u(0) * std::cos(state(2)) * dt;
    state(1) += u(0) * std::sin(state(2)) * dt;
    state(2) += u(1) * dt;

    r.travelled_x = state(0);
    r.max_detour_y = std::max(r.max_detour_y, std::abs(state(1)));

    const double gap = std::hypot(
        std::max(std::abs(state(0) - kObstacleX) - kObstacleHalf, 0.0),
        std::max(std::abs(state(1) - offset_y) - kObstacleHalf, 0.0));
    r.min_gap = std::min(r.min_gap, gap);
  }
  return r;
}

void report(const char* name, const Result& r) {
  std::printf(
      "  %-26s arrive=%-4s steps=%3d stop=%3d  x=%5.2f  detour=%5.2f  "
      "gap=%5.2f\n",
      name, r.arrived ? "yes" : "NO", r.steps, r.stop_frames, r.travelled_x,
      r.max_detour_y, r.min_gap);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) {
    std::printf("usage: %s <rejected.yaml> <valid.yaml> <model.bin> "
                "[verbose_offset]\n",
                argv[0]);
    return 2;
  }
  const std::string cfg_a = argv[1], cfg_b = argv[2], model = argv[3];

  std::printf("scenario: goal (%.1f, 0); obstacle box half=%.2f m at x=%.1f, "
              "swept laterally; robot 0.5x0.5 diff\n",
              kGoalX, kObstacleHalf, kObstacleX);
  std::printf("A (must be rejected) = %s\nB (must still pass) = %s\n\n",
              cfg_a.c_str(), cfg_b.c_str());

  bool a_rejected = false;
  try {
    NeuPANPlanner::fromYaml(cfg_a, model);
    std::printf("A loaded without complaint - the clearance guard is gone\n\n");
  } catch (const std::exception& e) {
    // Only the config guard counts; a typo'd path must not pass as rejection.
    a_rejected = std::string(e.what()).find("neupan config") != std::string::npos;
    std::printf("A %s: %s\n\n",
                a_rejected ? "rejected by the config guard"
                           : "failed for an unrelated reason",
                e.what());
  }

  const double offsets[] = {0.0, 0.3, 0.4, 0.5, 0.6, 0.8};
  int b_arrived = 0;
  try {
    for (double off : offsets) {
      std::printf("obstacle offset %.2f m:\n", off);
      const Result b = rollout(cfg_b, model, off, false);
      report("B (upstream margins)", b);
      b_arrived += b.arrived;
    }

    std::printf("\nB reached the goal in %d/%zu runs; offsets 0.0 and 0.3 "
                "block the reference line outright and need a global replan\n",
                b_arrived, std::size(offsets));

    if (argc > 4) {
      const double off = std::atof(argv[4]);
      std::printf("\ntrace at offset %.2f m, config B:\n", off);
      rollout(cfg_b, model, off, true);
    }
  } catch (const std::exception& e) {
    std::printf("B failed to run: %s\n", e.what());
    return 1;
  }
  // B must still slide past the obstacle it used to clear.
  return (a_rejected && b_arrived >= 4) ? 0 : 1;
}
