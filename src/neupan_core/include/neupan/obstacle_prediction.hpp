/*
 * Constant-velocity obstacle prediction used by the native NeuPAN runtime.
 */

#pragma once

#include <vector>

#include "neupan/types.hpp"

namespace neupan {

// Port of PAN.generate_point_flow's obstacle-motion part. Positions and
// velocities are paired column-wise and use one common world frame. Positions
// are metres and velocities are metres per second. An empty velocity matrix
// means static points. Uniform downsampling uses exactly the same indices for
// both arrays.
std::vector<Mat2X> predictObstaclePoints(const Mat2X& points,
                                         const Mat2X& velocities,
                                         int horizon, double step_time,
                                         int max_points);

}  // namespace neupan
