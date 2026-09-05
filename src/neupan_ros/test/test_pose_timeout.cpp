#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>

#include <class_loader/class_loader.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;

namespace {

class PoseTimeoutTest : public ::testing::Test {
 protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }

  rclcpp::NodeOptions options(double timeout) {
    rclcpp::NodeOptions result;
    result.arguments({"--ros-args", "-r", "__ns:=/pose_timeout_test"});
    result.parameter_overrides({
        {"config_file", std::string(NEUPAN_TEST_CONFIG)},
        {"planning_frame", "timeout_odom"},
        {"base_frame", "timeout_base"},
        {"obstacle_source", "pointcloud"},
        {"pose_timeout", timeout},
    });
    return result;
  }
};

TEST_F(PoseTimeoutTest, RejectsInvalidTimeout) {
  class_loader::ClassLoader loader(NEUPAN_NODE_LIBRARY);
  const auto classes = loader.getAvailableClasses<rclcpp_components::NodeFactory>();
  ASSERT_EQ(classes.size(), 1U);
  auto factory = loader.createInstance<rclcpp_components::NodeFactory>(classes[0]);
  for (double timeout : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()})
    EXPECT_THROW(factory->create_node_instance(options(timeout)), std::runtime_error);
}

TEST_F(PoseTimeoutTest, StopsOnStalePoseWithFreshObstaclesAndResumesOnNewTf) {
  class_loader::ClassLoader loader(NEUPAN_NODE_LIBRARY);
  const auto classes = loader.getAvailableClasses<rclcpp_components::NodeFactory>();
  ASSERT_EQ(classes.size(), 1U);
  auto factory = loader.createInstance<rclcpp_components::NodeFactory>(classes[0]);
  constexpr double timeout = 0.3;
  auto planner = factory->create_node_instance(options(timeout));
  auto driver = std::make_shared<rclcpp::Node>("driver", "/pose_timeout_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(planner.get_node_base_interface());
  executor.add_node(driver);

  auto cloud_pub = driver->create_publisher<sensor_msgs::msg::PointCloud2>("obstacles", 10);
  auto goal_pub = driver->create_publisher<geometry_msgs::msg::PoseStamped>("neupan_goal", 10);
  geometry_msgs::msg::Twist last_command;
  std::string last_status;
  std::size_t command_count = 0, moving_count = 0;
  auto command_sub = driver->create_subscription<geometry_msgs::msg::Twist>(
      "neupan_cmd_vel", 10, [&](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        last_command = *msg;
        ++command_count;
        if (std::abs(msg->linear.x) > 1e-6 || std::abs(msg->angular.z) > 1e-6)
          ++moving_count;
      });
  auto diagnostic_sub = driver->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "neupan_diagnostics", 10,
      [&](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
        if (!msg->status.empty()) last_status = msg->status.front().message;
      });
  tf2_ros::TransformBroadcaster broadcaster(driver);
  tf2_ros::StaticTransformBroadcaster static_broadcaster(driver);
  geometry_msgs::msg::TransformStamped reference_tf;
  reference_tf.header.frame_id = "timeout_map";
  reference_tf.child_frame_id = "timeout_odom";
  reference_tf.header.stamp = driver->now() - rclcpp::Duration::from_seconds(20.0);
  reference_tf.transform.rotation.w = 1.0;
  static_broadcaster.sendTransform(reference_tf);

  // Obstacles already use planning_frame, so stale robot TF cannot make this
  // stream stale indirectly through a failed sensor transform.
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "timeout_odom";
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(0);
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "timeout_map";  // Static reference TF must remain valid.
  goal.pose.position.x = 10.0;
  goal.pose.orientation.w = 1.0;
  geometry_msgs::msg::TransformStamped robot_tf;
  robot_tf.header.frame_id = "timeout_odom";
  robot_tf.child_frame_id = "timeout_base";
  robot_tf.transform.rotation.w = 1.0;

  const auto pump = [&](bool publish_pose) {
    const auto stamp = driver->now();
    cloud.header.stamp = stamp;
    goal.header.stamp = stamp;
    cloud_pub->publish(cloud);
    goal_pub->publish(goal);
    if (publish_pose) {
      robot_tf.header.stamp = stamp;
      broadcaster.sendTransform(robot_tf);
    }
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  };
  const auto wait_for = [&](const std::function<bool()>& condition, bool publish_pose) {
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    do {
      pump(publish_pose);
      if (condition()) return true;
    } while (std::chrono::steady_clock::now() < deadline);
    return false;
  };

  ASSERT_TRUE(wait_for([&] { return command_count > 0 &&
      last_status.find("no fresh robot pose") != std::string::npos; }, false));
  EXPECT_DOUBLE_EQ(last_command.linear.x, 0.0);
  ASSERT_TRUE(wait_for([&] { return last_command.linear.x > 0.01 &&
      last_status == "tracking"; }, true));

  const auto last_pose_stamp = rclcpp::Time(robot_tf.header.stamp);
  // Keep publishing obstacles/goals while allowing only robot TF to expire.
  while ((driver->now() - last_pose_stamp).seconds() < timeout + 0.15)
    pump(false);
  ASSERT_TRUE(wait_for([&] { return last_status.find("no fresh robot pose") !=
      std::string::npos && last_command.linear.x == 0.0; }, false));
  EXPECT_DOUBLE_EQ(last_command.angular.z, 0.0);
  const auto stopped_count = command_count;
  const auto moving_at_stop = moving_count;
  for (int i = 0; i < 15; ++i) pump(false);
  EXPECT_GT(command_count, stopped_count);  // A zero command on every cycle.
  EXPECT_EQ(moving_count, moving_at_stop);

  ASSERT_TRUE(wait_for([&] { return last_command.linear.x > 0.01 &&
      last_status == "tracking"; }, true));
}

}  // namespace
