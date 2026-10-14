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

#include "neupan/robot.hpp"

#include <cmath>
#include <stdexcept>

namespace neupan {

namespace {

// Preserve upstream winding/edge order, but reject degenerate and self-crossing
// input too: every vertex must lie on the interior side of every edge.
int convexOrientation(const Mat2X& v) {
  const Eigen::Index n = v.cols();
  if (n < 3) throw std::invalid_argument("robot: polygon needs >= 3 vertices");
  if (!v.allFinite()) throw std::invalid_argument("robot: vertices must be finite");
  const double scale = (v.rowwise().maxCoeff() - v.rowwise().minCoeff()).maxCoeff();
  if (scale <= 0.0 || !std::isfinite(scale))
    throw std::invalid_argument("robot: degenerate polygon");
  const Mat2X local = (v.colwise() - v.col(0)) / scale;
  constexpr double tolerance = 1e-12;
  const auto cross = [](const Vec2& a, const Vec2& b) {
    return a.x() * b.y() - a.y() * b.x();
  };
  double area = 0.0;
  for (Eigen::Index i = 0; i < n; ++i) {
    area += cross(local.col(i), local.col((i + 1) % n));
    for (Eigen::Index j = i + 1; j < n; ++j)
      if ((local.col(i) - local.col(j)).norm() <= tolerance)
        throw std::invalid_argument("robot: repeated polygon vertex");
  }
  if (std::abs(area) <= tolerance)
    throw std::invalid_argument("robot: polygon has zero area");
  const int direction = area > 0.0 ? 1 : -1;
  for (Eigen::Index i = 0; i < n; ++i) {
    const Vec2 edge = local.col((i + 1) % n) - local.col(i);
    for (Eigen::Index j = 0; j < n; ++j)
      if (direction * cross(edge, local.col(j) - local.col(i)) < -tolerance)
        throw std::invalid_argument("robot: polygon is not convex and ordered");
  }
  return direction;
}

}  // namespace

void genInequalFromVertex(const Mat2X& vertices, Mat& G, Vec& h) {
  Mat2X v = vertices;
  if (convexOrientation(v) < 0) {
    // CW input: keep first vertex, reverse the rest (as upstream does).
    Mat2X r(2, v.cols());
    r.col(0) = v.col(0);
    for (Eigen::Index i = 1; i < v.cols(); ++i)
      r.col(i) = v.col(v.cols() - i);
    v = r;
  }

  const Eigen::Index num = v.cols();
  G.resize(num, 2);
  h.resize(num);

  for (Eigen::Index i = 0; i < num; ++i) {
    const Vec2 pre = v.col(i);
    const Vec2 next = v.col((i + 1) % num);
    const Vec2 diff = next - pre;
    G(i, 0) = diff.y();
    G(i, 1) = -diff.x();
    h(i) = G(i, 0) * pre.x() + G(i, 1) * pre.y();
  }
}

Robot::Robot(Kinematics kinematics, int receding, double step_time,
             Vec2 max_speed_in, Vec2 max_acce, const Mat2X& vertices_in)
    : kinematics(kinematics),
      T(receding),
      dt(step_time),
      max_speed(std::move(max_speed_in)),
      acce_bound(max_acce * step_time),
      vertices(vertices_in) {
  genInequalFromVertex(vertices, G, h);
}

Robot Robot::diffRectangle(int receding, double step_time, Vec2 max_speed,
                           Vec2 max_acce, double length, double width,
                           double wheelbase) {
  if (!std::isfinite(length) || !std::isfinite(width) ||
      !std::isfinite(wheelbase) || length <= 0.0 || width <= 0.0)
    throw std::invalid_argument("robot: rectangle dimensions must be finite and positive");
  const double sx = -(length - wheelbase) / 2.0;
  const double sy = -width / 2.0;
  Mat2X v(2, 4);
  v.col(0) << sx, sy;
  v.col(1) << sx + length, sy;
  v.col(2) << sx + length, sy + width;
  v.col(3) << sx, sy + width;
  return Robot(Kinematics::Diff, receding, step_time, std::move(max_speed),
               std::move(max_acce), v);
}

void Robot::linearize(const Vec3& nom_s_t, const Vec2& nom_u_t, Mat33& A,
                      Mat32& B, Vec3& C) const {
  const double phi = nom_s_t(2);
  const double v = nom_u_t(0);

  switch (kinematics) {
    case Kinematics::Diff:
      A << 1, 0, -v * dt * std::sin(phi),  //
          0, 1, v * dt * std::cos(phi),    //
          0, 0, 1;
      B << std::cos(phi) * dt, 0,  //
          std::sin(phi) * dt, 0,   //
          0, dt;
      C << phi * v * std::sin(phi) * dt, -phi * v * std::cos(phi) * dt, 0;
      break;
  }
}

}  // namespace neupan
