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

#include <Eigen/Dense>
#include <vector>

namespace neupan {

using Mat = Eigen::MatrixXd;
using Vec = Eigen::VectorXd;
using Mat2X = Eigen::Matrix2Xd;
using Mat3X = Eigen::Matrix3Xd;
using Mat22 = Eigen::Matrix2d;
using Mat33 = Eigen::Matrix3d;
using Mat32 = Eigen::Matrix<double, 3, 2>;
using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;

using MatF = Eigen::MatrixXf;
using VecF = Eigen::VectorXf;

}  // namespace neupan
