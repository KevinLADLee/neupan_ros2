/*
 * Copyright (c) 2026 NeuPAN ROS2 contributors.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "neupan_ros/pointcloud_preprocessor.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace neupan_ros {
namespace {

bool hasField(const sensor_msgs::msg::PointCloud2& msg,
              const std::string& name) {
  for (const auto& field : msg.fields)
    if (field.name == name) return true;
  return false;
}

bool hasFloat32Field(const sensor_msgs::msg::PointCloud2& msg,
                     const std::string& name) {
  for (const auto& field : msg.fields)
    if (field.name == name &&
        field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
        field.count == 1)
      return true;
  return false;
}

void requireFloat32(const sensor_msgs::msg::PointCloud2& msg,
                    const std::string& name) {
  if (!hasFloat32Field(msg, name))
    throw std::invalid_argument("PointCloud2 field '" + name +
                                "' must be scalar FLOAT32");
}

}  // namespace

ObstacleObservation preprocessPointCloud(
    const sensor_msgs::msg::PointCloud2& msg) {
  requireFloat32(msg, "x");
  requireFloat32(msg, "y");
  requireFloat32(msg, "z");

  ObstacleObservation result;
  const bool has_intensity = hasField(msg, "intensity");

  const bool has_vx = hasField(msg, "vx");
  const bool has_vy = hasField(msg, "vy");
  if (has_vx != has_vy)
    throw std::invalid_argument(
        "XYZIV PointCloud2 requires both vx and vy fields");
  const bool has_velocity = has_vx;
  if (has_velocity && !has_intensity)
    throw std::invalid_argument(
        "XYZIV PointCloud2 requires an intensity field");
  if (has_velocity) {
    requireFloat32(msg, "vx");
    requireFloat32(msg, "vy");
  }

  const std::size_t count =
      static_cast<std::size_t>(msg.width) * msg.height;
  result.points.resize(2, static_cast<Eigen::Index>(count));
  result.velocities.resize(2, static_cast<Eigen::Index>(count));
  Eigen::Index output_index = 0;

  const auto append = [&](double px, double py, double pvx, double pvy) {
    result.points.col(output_index) << px, py;
    result.velocities.col(output_index) << pvx, pvy;
    ++output_index;
  };

  sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z(msg, "z");

  if (has_velocity) {
    sensor_msgs::PointCloud2ConstIterator<float> vx(msg, "vx");
    sensor_msgs::PointCloud2ConstIterator<float> vy(msg, "vy");
    for (; x != x.end(); ++x, ++y, ++z, ++vx, ++vy) {
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z) ||
          !std::isfinite(*vx) || !std::isfinite(*vy)) {
        ++result.discarded_points;
        continue;
      }
      append(*x, *y, *vx, *vy);
    }
  } else {
    for (; x != x.end(); ++x, ++y, ++z) {
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
        ++result.discarded_points;
        continue;
      }
      append(*x, *y, 0.0, 0.0);
    }
  }

  if (output_index != static_cast<Eigen::Index>(count)) {
    result.points.conservativeResize(Eigen::NoChange, output_index);
    result.velocities.conservativeResize(Eigen::NoChange, output_index);
  }
  result.format = has_velocity ? ObstacleFormat::XYZIV
                               : (has_intensity ? ObstacleFormat::XYZI
                                                : ObstacleFormat::XYZ);
  return result;
}

}  // namespace neupan_ros
