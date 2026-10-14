#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include <sensor_msgs/msg/laser_scan.hpp>

#include "neupan_ros/laser_scan_preprocessor.hpp"

namespace {

sensor_msgs::msg::LaserScan makeScan() {
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = -1.0F;
  scan.angle_max = 1.0F;
  scan.angle_increment = 0.5F;
  scan.ranges = {1.0F, std::numeric_limits<float>::infinity(), 2.0F, 3.0F,
                 6.0F};
  return scan;
}

}  // namespace

TEST(LaserScanPreprocessor, FiltersAndDownsamplesIntoPairedMatrices) {
  neupan_ros::LaserScanPreprocessorConfig config;
  config.angle_min = -1.1;
  config.angle_max = 1.1;
  config.range_max = 5.0;
  config.downsample = 2;

  const auto observation =
      neupan_ros::preprocessLaserScan(makeScan(), config);

  ASSERT_EQ(observation.points.cols(), 2);
  ASSERT_EQ(observation.velocities.cols(), observation.points.cols());
  EXPECT_TRUE(observation.velocities.isZero(0.0));
  EXPECT_EQ(observation.discarded_points, 1U);
  EXPECT_STREQ(neupan_ros::toString(observation.format), "LaserScan");
  EXPECT_NEAR(observation.points(0, 0), std::cos(-1.0), 1e-6);
  EXPECT_NEAR(observation.points(1, 0), std::sin(-1.0), 1e-6);
  EXPECT_NEAR(observation.points(0, 1), 2.0, 1e-12);
  EXPECT_NEAR(observation.points(1, 1), 0.0, 1e-12);
}

TEST(LaserScanPreprocessor, FlipAngleReversesBeamDirection) {
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = -0.5F;
  scan.angle_max = 0.5F;
  scan.angle_increment = 0.5F;
  scan.ranges = {1.0F, 1.0F, 1.0F};

  neupan_ros::LaserScanPreprocessorConfig config;
  config.angle_min = -1.0;
  config.angle_max = 1.0;
  config.flip_angle = true;

  const auto observation = neupan_ros::preprocessLaserScan(scan, config);
  ASSERT_EQ(observation.points.cols(), 3);
  EXPECT_NEAR(observation.points(1, 0), std::sin(0.5), 1e-6);
  EXPECT_NEAR(observation.points(1, 2), std::sin(-0.5), 1e-6);
}

TEST(LaserScanPreprocessor, RejectsInvalidDownsample) {
  neupan_ros::LaserScanPreprocessorConfig config;
  config.downsample = 0;
  EXPECT_THROW(neupan_ros::preprocessLaserScan(makeScan(), config),
               std::invalid_argument);
}
