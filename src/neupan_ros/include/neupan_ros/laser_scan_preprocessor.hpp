/*
 * Copyright (c) 2026 NeuPAN ROS2 contributors.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>

#include "neupan_ros/obstacle_observation.hpp"

namespace neupan_ros {

struct LaserScanPreprocessorConfig {
  double angle_min = -3.14;
  double angle_max = 3.14;
  double range_min = 0.0;
  double range_max = 5.0;
  int downsample = 1;
  bool flip_angle = false;
};

// Convert valid LaserScan beams to planar obstacle points in
// msg.header.frame_id. This function does not query TF. Static zero velocities
// are materialized in the same frame to preserve the column-pair invariant.
ObstacleObservation preprocessLaserScan(
    const sensor_msgs::msg::LaserScan& msg,
    const LaserScanPreprocessorConfig& config);

}  // namespace neupan_ros
