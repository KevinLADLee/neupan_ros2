#include <gtest/gtest.h>
#include "neupan_ros/turn_slowdown.hpp"

TEST(TurnSlowdown, UsesAvoidanceHeadingAndWrapsYaw) {
  constexpr double pi = 3.141592653589793;
  neupan::Mat3X path = neupan::Mat3X::Zero(3, 31);
  const auto scale = [&](double yaw) {
    return neupan_ros::turnSpeedScale(path, yaw, .1, 1., pi / 18, pi / 6);
  };
  path(2, 10) = pi / 9;
  EXPECT_NEAR(scale(0), .5, 1e-12);
  EXPECT_NEAR(scale(pi / 9), 1., 1e-12);  // Measured body catches up.
  path(2, 10) = -pi / 9;
  EXPECT_NEAR(scale(0), .5, 1e-12);
  path(2, 10) = pi / 4;
  EXPECT_DOUBLE_EQ(scale(0), 0.);
  path(2, 10) = -pi + .01;
  EXPECT_DOUBLE_EQ(scale(pi - .01), 1.);
  path.row(0).setConstant(-1.);  // Reversing doesn't invert body alignment.
  EXPECT_DOUBLE_EQ(scale(pi - .01), 1.);
  path.resize(3, 0);
  EXPECT_DOUBLE_EQ(scale(0), 1.);
}
