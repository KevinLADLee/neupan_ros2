#include "neupan/obstacle_prediction.hpp"

#include <algorithm>
#include <stdexcept>
#include <vector>

namespace neupan {

std::vector<Mat2X> predictObstaclePoints(const Mat2X& points,
                                         const Mat2X& velocities,
                                         int horizon, double step_time,
                                         int max_points) {
  if (horizon < 0 || step_time <= 0.0 || max_points < 0)
    throw std::invalid_argument("obstacle prediction: invalid horizon/time/limit");
  if (velocities.cols() != 0 && velocities.cols() != points.cols())
    throw std::invalid_argument(
        "obstacle prediction: positions and velocities must have equal columns");

  const int n = static_cast<int>(points.cols());
  const int kept = std::min(n, max_points);
  Mat2X sampled_points(2, kept);
  Mat2X sampled_velocities = Mat2X::Zero(2, kept);

  for (int i = 0; i < kept; ++i) {
    // Matches neupan.util.downsample_decimation:
    // np.linspace(0, n-1, kept).astype(int).
    const int index = kept == n
                          ? i
                          : (kept == 1 ? 0
                                       : static_cast<int>(
                                             static_cast<double>(i) * (n - 1) /
                                             (kept - 1)));
    sampled_points.col(i) = points.col(index);
    if (velocities.cols() != 0)
      sampled_velocities.col(i) = velocities.col(index);
  }

  std::vector<Mat2X> prediction;
  prediction.reserve(static_cast<std::size_t>(horizon + 1));
  for (int t = 0; t <= horizon; ++t)
    prediction.push_back(sampled_points +
                         static_cast<double>(t) * step_time * sampled_velocities);
  return prediction;
}

}  // namespace neupan
