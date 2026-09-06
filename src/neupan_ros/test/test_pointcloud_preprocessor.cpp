#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
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

namespace {

sensor_msgs::msg::PointCloud2 makePaddedCloud(bool dynamic, bool bigendian) {
  sensor_msgs::msg::PointCloud2 msg;
  const std::vector<std::string> names = dynamic
      ? std::vector<std::string>{"x", "y", "z", "intensity", "vx", "vy"}
      : std::vector<std::string>{"x", "y", "z"};
  for (std::size_t i = 0; i < names.size(); ++i) {
    sensor_msgs::msg::PointField field;
    field.name = names[i];
    field.offset = 1 + 4 * i;  // Deliberately unaligned.
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    msg.fields.push_back(field);
  }
  msg.width = 2;
  msg.height = 2;
  msg.point_step = 4 * names.size() + 4;
  msg.row_step = msg.width * msg.point_step + 12;
  msg.is_bigendian = bigendian;
  msg.data.resize(msg.height * msg.row_step, 0);
  for (uint32_t row = 0; row < msg.height; ++row) {
    for (uint32_t col = 0; col < msg.width; ++col) {
      const float index = row * msg.width + col;
      const float values[] = {1 + index, -2 - index, 0, 42, index / 4, -0.5F};
      for (std::size_t field = 0; field < names.size(); ++field) {
        uint32_t bits;
        std::memcpy(&bits, &values[field], sizeof(bits));
        const auto offset = row * msg.row_step + col * msg.point_step +
                            msg.fields[field].offset;
        for (int byte = 0; byte < 4; ++byte)
          msg.data[offset + byte] = bits >> (8 * (bigendian ? 3 - byte : byte));
      }
    }
  }
  return msg;
}

}  // namespace

TEST(PointCloudPreprocessor, OrganizedRowsSkipPaddingAndDecodeUnalignedFields) {
  for (bool dynamic : {false, true}) {
    for (bool bigendian : {false, true}) {
      SCOPED_TRACE(::testing::Message() << "dynamic=" << dynamic
                                       << " bigendian=" << bigendian);
      const auto cloud =
          neupan_ros::preprocessPointCloud(makePaddedCloud(dynamic, bigendian));
      ASSERT_EQ(cloud.points.cols(), 4);
      ASSERT_EQ(cloud.velocities.cols(), 4);
      EXPECT_EQ(cloud.discarded_points, 0U);
      for (int i = 0; i < 4; ++i) {
        EXPECT_DOUBLE_EQ(cloud.points(0, i), 1 + i);
        EXPECT_DOUBLE_EQ(cloud.points(1, i), -2 - i);
        EXPECT_DOUBLE_EQ(cloud.velocities(0, i), dynamic ? i / 4.0 : 0.0);
        EXPECT_DOUBLE_EQ(cloud.velocities(1, i), dynamic ? -0.5 : 0.0);
      }
    }
  }
}

TEST(PointCloudPreprocessor, RejectsMalformedLayoutBeforeReadingData) {
  auto msg = makePaddedCloud(true, false);
  msg.data.pop_back();
  EXPECT_THROW(neupan_ros::preprocessPointCloud(msg), std::invalid_argument);
  msg = makePaddedCloud(true, false);
  msg.row_step = msg.width * msg.point_step - 1;
  msg.data.resize(msg.row_step * msg.height);
  EXPECT_THROW(neupan_ros::preprocessPointCloud(msg), std::invalid_argument);
  msg = makePaddedCloud(true, false);
  msg.point_step = 0;
  EXPECT_THROW(neupan_ros::preprocessPointCloud(msg), std::invalid_argument);
  for (const auto offset : {25U, std::numeric_limits<uint32_t>::max()}) {
    msg = makePaddedCloud(true, false);
    msg.fields.back().offset = offset;
    EXPECT_THROW(neupan_ros::preprocessPointCloud(msg), std::invalid_argument);
  }
}

TEST(PointCloudPreprocessor, AcceptsEmptyCloudWithDeclaredFields) {
  auto msg = makePaddedCloud(false, false);
  msg.width = 0;
  msg.height = 1;
  msg.row_step = 0;
  msg.data.clear();
  const auto cloud = neupan_ros::preprocessPointCloud(msg);
  EXPECT_EQ(cloud.points.cols(), 0);
  EXPECT_EQ(cloud.velocities.cols(), 0);
}
