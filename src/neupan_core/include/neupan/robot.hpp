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

#pragma once

#include "neupan/types.hpp"

namespace neupan {

enum class Kinematics { Diff };  // v0.2: Omni, later: Acker

// Port of neupan/robot/robot.py: robot geometry (convex hull G x <= h),
// velocity/acceleration bounds, and linearized kinematics.
class Robot {
 public:
  Robot(Kinematics kinematics, int receding, double step_time, Vec2 max_speed,
        Vec2 max_acce, const Mat2X& vertices);

  // Rectangle footprint as in robot.cal_vertices_from_length_width.
  static Robot diffRectangle(int receding, double step_time, Vec2 max_speed,
                             Vec2 max_acce, double length, double width,
                             double wheelbase = 0.0);

  // Linearized dynamics s_{t+1} = A s_t + B u_t + C around (nom_s_t, nom_u_t).
  void linearize(const Vec3& nom_s_t, const Vec2& nom_u_t, Mat33& A, Mat32& B,
                 Vec3& C) const;

  Kinematics kinematics;
  int T;
  double dt;
  Vec2 max_speed;   // speed_bound
  Vec2 acce_bound;  // max_acce * dt
  Mat2X vertices;   // (2, N) CCW
  Mat G;            // (E, 2)
  Vec h;            // (E,)
};

// Port of neupan.util.gen_inequal_from_vertex: convex polygon (2, N),
// CCW-normalized, to inequality G x <= h. Throws if not convex.
void genInequalFromVertex(const Mat2X& vertices, Mat& G, Vec& h);

}  // namespace neupan
