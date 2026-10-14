/*
 * Copyright (c) 2026 NeuPAN ROS2 contributors.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "neupan_ros/laser_scan_preprocessor.hpp"

#include <cmath>
#include <stdexcept>

namespace neupan_ros {

ObstacleObservation preprocessLaserScan(
    const sensor_msgs::msg::LaserScan& msg,
    const LaserScanPreprocessorConfig& config) {
  if (config.downsample < 1)
    throw std::invalid_argument("LaserScan downsample must be >= 1");

  ObstacleObservation observation;
  observation.format = ObstacleFormat::LaserScan;

  const Eigen::Index beam_count =
      static_cast<Eigen::Index>(msg.ranges.size());
  const Eigen::Index candidate_count =
      (beam_count + config.downsample - 1) / config.downsample;
  observation.points.resize(2, candidate_count);

  const double angle_increment =
      msg.angle_increment != 0.0F
          ? msg.angle_increment
          : (beam_count > 1
                 ? (msg.angle_max - msg.angle_min) / (beam_count - 1)
                 : 0.0);

  Eigen::Index output_index = 0;
  for (Eigen::Index i = 0; i < beam_count; i += config.downsample) {
    const double range = msg.ranges[static_cast<std::size_t>(i)];
    const double angle =
        config.flip_angle ? msg.angle_max - i * angle_increment
                          : msg.angle_min + i * angle_increment;
    if (!std::isfinite(range) || range < config.range_min ||
        range > config.range_max || angle <= config.angle_min ||
        angle >= config.angle_max) {
      ++observation.discarded_points;
      continue;
    }

    observation.points.col(output_index) << range * std::cos(angle),
        range * std::sin(angle);
    ++output_index;
  }

  observation.points.conservativeResize(Eigen::NoChange, output_index);
  observation.velocities = neupan::Mat2X::Zero(2, output_index);
  return observation;
}

}  // namespace neupan_ros
