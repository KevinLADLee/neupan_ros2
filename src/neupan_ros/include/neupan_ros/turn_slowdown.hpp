#pragma once

#include <algorithm>
#include <cmath>
#include "neupan/types.hpp"

namespace neupan_ros {

// Compare the optimized body's future orientation with measured yaw. This
// also works in reverse: neither the original straight path nor velocity
// bearing describes body alignment on a reverse avoidance manoeuvre.
inline double turnSpeedScale(const neupan::Mat3X& trajectory, double yaw,
                             double step_time, double lookahead,
                             double start_angle, double stop_angle) {
  if (trajectory.cols() < 2) return 1.0;
  const auto index = std::clamp<Eigen::Index>(
      static_cast<Eigen::Index>(std::ceil(lookahead / step_time)),
      1, trajectory.cols() - 1);
  const double delta = trajectory(2, index) - yaw;
  const double error = std::abs(std::atan2(std::sin(delta), std::cos(delta)));
  return std::clamp((stop_angle - error) / (stop_angle - start_angle), 0.0, 1.0);
}

}  // namespace neupan_ros
