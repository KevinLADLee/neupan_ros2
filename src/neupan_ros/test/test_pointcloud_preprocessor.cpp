#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "neupan_ros/pointcloud_preprocessor.hpp"

namespace {

sensor_msgs::msg::PointCloud2 makeXyzCloud() {
  sensor_msgs::msg::PointCloud2 msg;
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(
      3, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
      sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(2);
  sensor_msgs::PointCloud2Iterator<float> x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> z(msg, "z");
  *x = 1.0F;
  *y = 2.0F;
  *z = 0.0F;
  ++x;
  ++y;
  ++z;
  *x = std::numeric_limits<float>::quiet_NaN();
  *y = 3.0F;
  *z = 0.0F;
  return msg;
}

sensor_msgs::msg::PointCloud2 makeXyziCartesianCloud() {
  sensor_msgs::msg::PointCloud2 msg;
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(
      6, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
      sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
      sensor_msgs::msg::PointField::FLOAT32, "vx", 1,
      sensor_msgs::msg::PointField::FLOAT32, "vy", 1,
      sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(1);
  sensor_msgs::PointCloud2Iterator<float> x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> z(msg, "z");
  sensor_msgs::PointCloud2Iterator<float> intensity(msg, "intensity");
  sensor_msgs::PointCloud2Iterator<float> vx(msg, "vx");
  sensor_msgs::PointCloud2Iterator<float> vy(msg, "vy");
  *x = 1.0F;
  *y = 2.0F;
  *z = 0.5F;
  *intensity = 42.0F;
  *vx = 0.5F;
  *vy = -0.25F;
  return msg;
}

sensor_msgs::msg::PointCloud2 makeXyziCloud() {
  sensor_msgs::msg::PointCloud2 msg;
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
      sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
      sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(1);
  sensor_msgs::PointCloud2Iterator<float> x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> z(msg, "z");
  sensor_msgs::PointCloud2Iterator<float> intensity(msg, "intensity");
  *x = -1.0F;
  *y = 0.25F;
  *z = 2.0F;
  *intensity = 123.0F;
  return msg;
}

}  // namespace

TEST(PointCloudPreprocessor, XyzIsStaticAndDropsNonFinitePoints) {
  const auto cloud = neupan_ros::preprocessPointCloud(makeXyzCloud());
  ASSERT_EQ(cloud.points.cols(), 1);
  EXPECT_FALSE(cloud.hasVelocity());
  EXPECT_STREQ(neupan_ros::toString(cloud.format), "XYZ");
  EXPECT_EQ(cloud.discarded_points, 1U);
  EXPECT_DOUBLE_EQ(cloud.points(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(cloud.points(1, 0), 2.0);
  EXPECT_TRUE(cloud.velocities.isZero(0.0));
}

TEST(PointCloudPreprocessor, XyziWithCartesianVelocityPreservesVector) {
  const auto cloud =
      neupan_ros::preprocessPointCloud(makeXyziCartesianCloud());
  EXPECT_TRUE(cloud.hasVelocity());
  EXPECT_STREQ(neupan_ros::toString(cloud.format), "XYZIV");
  EXPECT_DOUBLE_EQ(cloud.velocities(0, 0), 0.5);
  EXPECT_DOUBLE_EQ(cloud.velocities(1, 0), -0.25);
}

TEST(PointCloudPreprocessor, XyziIsAcceptedAsStatic) {
  const auto cloud = neupan_ros::preprocessPointCloud(makeXyziCloud());
  EXPECT_FALSE(cloud.hasVelocity());
  EXPECT_STREQ(neupan_ros::toString(cloud.format), "XYZI");
  EXPECT_DOUBLE_EQ(cloud.points(0, 0), -1.0);
  EXPECT_DOUBLE_EQ(cloud.points(1, 0), 0.25);
  EXPECT_TRUE(cloud.velocities.isZero(0.0));
}

TEST(PointCloudPreprocessor, TransformTranslatesPointsAndOnlyRotatesVelocity) {
  auto cloud = neupan_ros::preprocessPointCloud(makeXyziCartesianCloud());
  neupan_ros::transformObservation(
      cloud, neupan::Vec3(10.0, 20.0, std::acos(-1.0) / 2.0));

  EXPECT_NEAR(cloud.points(0, 0), 8.0, 1e-12);
  EXPECT_NEAR(cloud.points(1, 0), 21.0, 1e-12);
  EXPECT_NEAR(cloud.velocities(0, 0), 0.25, 1e-12);
  EXPECT_NEAR(cloud.velocities(1, 0), 0.5, 1e-12);
}

TEST(PointCloudPreprocessor, RejectsIncompleteCartesianVelocityPair) {
  auto msg = makeXyzCloud();
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
      sensor_msgs::msg::PointField::FLOAT32, "vx", 1,
      sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(1);
  EXPECT_THROW(neupan_ros::preprocessPointCloud(msg), std::invalid_argument);
}

TEST(PointCloudPreprocessor, RejectsVelocityWithoutIntensity) {
  sensor_msgs::msg::PointCloud2 msg;
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(
      5, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
      sensor_msgs::msg::PointField::FLOAT32, "vx", 1,
      sensor_msgs::msg::PointField::FLOAT32, "vy", 1,
      sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(1);
  EXPECT_THROW(neupan_ros::preprocessPointCloud(msg), std::invalid_argument);
}
