/*
 * Copyright (c) 2026 NeuPAN ROS2 contributors.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "neupan_ros/obstacle_observation.hpp"

namespace neupan_ros {

// Decode the common PointCloud2 obstacle layouts in msg.header.frame_id without
// querying TF or changing coordinates.
// Supported layouts are XYZ, XYZI and XYZIV. In this project XYZIV is defined
// unambiguously as x/y/z/intensity/vx/vy, with V a planar velocity vector.
ObstacleObservation preprocessPointCloud(
    const sensor_msgs::msg::PointCloud2& msg);

}  // namespace neupan_ros
