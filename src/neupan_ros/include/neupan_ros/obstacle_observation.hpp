/*
 * Copyright (c) 2026 NeuPAN ROS2 contributors.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once

#include <cstddef>

#include "neupan/types.hpp"

namespace neupan_ros {

enum class ObstacleFormat { LaserScan, XYZ, XYZI, XYZIV };

const char* toString(ObstacleFormat format);

// A single obstacle measurement after ROS message decoding. Positions and
// velocities are paired column-wise and expressed in the same coordinate frame.
struct ObstacleObservation {
  neupan::Mat2X points = neupan::Mat2X(2, 0);
  neupan::Mat2X velocities = neupan::Mat2X(2, 0);
  ObstacleFormat format = ObstacleFormat::LaserScan;
  std::size_t discarded_points = 0;

  bool hasVelocity() const { return format == ObstacleFormat::XYZIV; }
};

// Apply an SE(2) transform in place. Positions rotate and translate; velocity
// vectors only rotate.
void transformObservation(ObstacleObservation& observation,
                          const neupan::Vec3& target_from_source);

}  // namespace neupan_ros
