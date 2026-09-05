/*
 * Copyright (c) 2026 NeuPAN ROS2 contributors.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "neupan_ros/pointcloud_preprocessor.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

#include <sensor_msgs/msg/point_field.hpp>

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

std::size_t requireFloat32(const sensor_msgs::msg::PointCloud2& msg,
                          const std::string& name) {
  if (!hasFloat32Field(msg, name))
    throw std::invalid_argument("PointCloud2 field '" + name +
                                "' must be scalar FLOAT32");
  for (const auto& field : msg.fields) {
    if (field.name != name) continue;
    if (field.offset > msg.point_step ||
        msg.point_step - field.offset < sizeof(float))
      throw std::invalid_argument("PointCloud2 field '" + name +
                                  "' extends beyond point_step");
    return field.offset;
  }
  throw std::invalid_argument("PointCloud2 field '" + name + "' is missing");
}

float readFloat32(const uint8_t* data, bool bigendian) {
  // Byte assembly handles both endiannesses and unaligned field offsets.
  uint32_t bits = 0;
  for (int i = 0; i < 4; ++i)
    bits |= static_cast<uint32_t>(data[i]) << (8 * (bigendian ? 3 - i : i));
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

}  // namespace

ObstacleObservation preprocessPointCloud(
    const sensor_msgs::msg::PointCloud2& msg) {
  const auto x_offset = requireFloat32(msg, "x");
  const auto y_offset = requireFloat32(msg, "y");
  const auto z_offset = requireFloat32(msg, "z");

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
  const auto vx_offset = has_velocity ? requireFloat32(msg, "vx") : 0;
  const auto vy_offset = has_velocity ? requireFloat32(msg, "vy") : 0;

  if (static_cast<uint64_t>(msg.width) * msg.point_step > msg.row_step ||
      static_cast<uint64_t>(msg.row_step) * msg.height != msg.data.size())
    throw std::invalid_argument("PointCloud2 has inconsistent row_step or data size");

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

  for (uint32_t row = 0; row < msg.height; ++row) {
    for (uint32_t col = 0; col < msg.width; ++col) {
      const auto* point = msg.data.data() +
                          static_cast<std::size_t>(row) * msg.row_step +
                          static_cast<std::size_t>(col) * msg.point_step;
      const float x = readFloat32(point + x_offset, msg.is_bigendian);
      const float y = readFloat32(point + y_offset, msg.is_bigendian);
      const float z = readFloat32(point + z_offset, msg.is_bigendian);
      const float vx = has_velocity
                           ? readFloat32(point + vx_offset, msg.is_bigendian)
                           : 0.0F;
      const float vy = has_velocity
                           ? readFloat32(point + vy_offset, msg.is_bigendian)
                           : 0.0F;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
          !std::isfinite(vx) || !std::isfinite(vy)) {
        ++result.discarded_points;
        continue;
      }
      append(x, y, vx, vy);
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
