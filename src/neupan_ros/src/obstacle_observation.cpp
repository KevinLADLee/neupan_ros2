/*
 * Copyright (c) 2026 NeuPAN ROS2 contributors.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "neupan_ros/obstacle_observation.hpp"

#include <cmath>
#include <stdexcept>

namespace neupan_ros {

const char* toString(ObstacleFormat format) {
  switch (format) {
    case ObstacleFormat::LaserScan:
      return "LaserScan";
    case ObstacleFormat::XYZ:
      return "XYZ";
    case ObstacleFormat::XYZI:
      return "XYZI";
    case ObstacleFormat::XYZIV:
      return "XYZIV";
  }
  throw std::invalid_argument("unknown obstacle format");
}

void transformObservation(ObstacleObservation& observation,
                          const neupan::Vec3& target_from_source) {
  const double c = std::cos(target_from_source(2));
  const double s = std::sin(target_from_source(2));
  neupan::Mat22 rotation;
  rotation << c, -s, s, c;

  observation.points = rotation * observation.points;
  observation.points.colwise() += target_from_source.head<2>();
  observation.velocities = rotation * observation.velocities;
}

}  // namespace neupan_ros
