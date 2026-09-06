#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <set>
#include <rclcpp/parameter_map.hpp>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "neupan_sim/simulation.hpp"

namespace {

using neupan_sim::CircleObstacle;
using neupan_sim::Pose2;
using neupan_sim::SegmentObstacle;
using neupan_sim::Simulation;
using neupan_sim::SimulationConfig;
using neupan_sim::Vec2;

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

std::vector<CircleObstacle> parseCircles(
    const std::vector<double>& values, bool dynamic) {
  const std::size_t stride = dynamic ? 5 : 3;
  if (values.size() % stride != 0)
    throw std::invalid_argument("invalid flattened circle obstacle list");
  std::vector<CircleObstacle> circles;
  circles.reserve(values.size() / stride);
  for (std::size_t i = 0; i < values.size(); i += stride) {
    CircleObstacle circle{{values[i], values[i + 1]}, values[i + 2], {}};
    if (dynamic) circle.velocity = {values[i + 3], values[i + 4]};
    circles.push_back(circle);
  }
  return circles;
}

std::vector<SegmentObstacle> parseSegments(
    const std::vector<double>& values) {
  if (values.size() % 4 != 0)
    throw std::invalid_argument("invalid flattened segment obstacle list");
  std::vector<SegmentObstacle> segments;
  segments.reserve(values.size() / 4);
  for (std::size_t i = 0; i < values.size(); i += 4)
    segments.push_back({{values[i], values[i + 1]},
                        {values[i + 2], values[i + 3]}});
  return segments;
}

std::vector<Vec2> parseWaypoints(const std::vector<double>& values) {
  if (values.size() < 4 || values.size() % 2 != 0)
    throw std::invalid_argument("path_waypoints requires at least two x/y pairs");
  std::vector<Vec2> waypoints;
  waypoints.reserve(values.size() / 2);
  for (std::size_t i = 0; i < values.size(); i += 2)
    waypoints.push_back({values[i], values[i + 1]});
  return waypoints;
}

void addDiagnostic(diagnostic_msgs::msg::DiagnosticStatus& status,
                   std::string key, const std::string& value) {
  diagnostic_msgs::msg::KeyValue item;
  item.key = std::move(key);
  item.value = value;
  status.values.push_back(std::move(item));
}

void addDiagnostic(diagnostic_msgs::msg::DiagnosticStatus& status,
                   std::string key, double value) {
  addDiagnostic(status, std::move(key), std::to_string(value));
}

}  // namespace

class NeupanSimNode final : public rclcpp::Node {
 public:
  explicit NeupanSimNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
                         bool external_step = false) : Node("neupan_sim", options) {
    world_frame_ = declare_parameter<std::string>("world_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    laser_frame_ = declare_parameter<std::string>("laser_frame", "laser_link");
    if (world_frame_.empty() || base_frame_.empty() || laser_frame_.empty() ||
        world_frame_ == base_frame_ || world_frame_ == laser_frame_ || base_frame_ == laser_frame_)
      throw std::invalid_argument("simulator frame IDs must be nonempty and distinct");
    const auto color = declare_parameter<std::vector<double>>("marker_color", {0.1, 0.8, 0.3});
    status_position_ = declare_parameter<std::vector<double>>("status_position", std::vector<double>{});
    robot_label_ = declare_parameter<std::string>("robot_label", "");
    show_world_ = declare_parameter<bool>("show_world", true);
    if (color.size() != 3 || std::any_of(color.begin(), color.end(), [](double x) {
          return !std::isfinite(x) || x < 0.0 || x > 1.0;
        }) || (!status_position_.empty() && (status_position_.size() != 2 ||
          !std::isfinite(status_position_[0]) || !std::isfinite(status_position_[1]))))
      throw std::invalid_argument("invalid marker color or status position");
    marker_color_.r = color[0]; marker_color_.g = color[1]; marker_color_.b = color[2];
    marker_color_.a = 1.0F;
    paused_ = declare_parameter<bool>("start_paused", false);
    start_service_ = create_service<std_srvs::srv::Trigger>(
        "start", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          paused_ = false;
          response->success = true;
          response->message = "simulation running";
        });
    physics_rate_ = declare_parameter<double>("physics_rate", 100.0);
    sensor_rate_ = declare_parameter<double>("sensor_rate", 20.0);
    diagnostics_rate_ = declare_parameter<double>("diagnostics_rate", 10.0);
    path_spacing_ = declare_parameter<double>("path_spacing", 0.1);
    scenario_name_ =
        declare_parameter<std::string>("scenario_name", "simulation");

    const auto initial = declare_parameter<std::vector<double>>(
        "initial_pose", {0.0, 0.0, 0.0});
    const auto path_values = declare_parameter<std::vector<double>>(
        "path_waypoints", {0.0, 0.0, 6.0, 0.0});
    const auto bounds = declare_parameter<std::vector<double>>(
        "world_bounds", {-2.0, 8.0, -4.0, 4.0});
    const auto robot_size = declare_parameter<std::vector<double>>(
        "robot_size", {0.5, 0.5});
    const auto robot_vertices = declare_parameter<std::vector<double>>(
        "robot_vertices", std::vector<double>{});
    const auto speed_limits = declare_parameter<std::vector<double>>(
        "speed_limits", {2.0, 1.5});
    const auto acceleration_limits = declare_parameter<std::vector<double>>(
        "acceleration_limits", {2.0, 2.0});
    const auto laser_pose = declare_parameter<std::vector<double>>(
        "laser_pose", {0.15, 0.0, 0.0});
    const auto static_values = declare_parameter<std::vector<double>>(
        "static_circles", std::vector<double>{});
    const auto dynamic_values = declare_parameter<std::vector<double>>(
        "dynamic_circles", std::vector<double>{});
    const auto segment_values = declare_parameter<std::vector<double>>(
        "segments", std::vector<double>{});

    scan_angle_min_ = declare_parameter<double>("scan_angle_min", -M_PI);
    scan_angle_max_ = declare_parameter<double>("scan_angle_max", M_PI);
    scan_angle_increment_ = declare_parameter<double>(
        "scan_angle_increment", M_PI / 180.0);
    scan_range_min_ = declare_parameter<double>("scan_range_min", 0.05);
    scan_range_max_ = declare_parameter<double>("scan_range_max", 8.0);

    SimulationConfig config;
    config.command_timeout =
        declare_parameter<double>("command_timeout", 0.25);
    config.goal_tolerance =
        declare_parameter<double>("goal_tolerance", 0.12);
    config.simulation_timeout =
        declare_parameter<double>("simulation_timeout", 30.0);
    config.integration_substeps =
        declare_parameter<int>("integration_substeps", 2);

    if (physics_rate_ <= 0.0 || sensor_rate_ <= 0.0 ||
        diagnostics_rate_ <= 0.0 || sensor_rate_ > physics_rate_ ||
        diagnostics_rate_ > physics_rate_ || path_spacing_ <= 0.0 ||
        initial.size() != 3 || bounds.size() != 4 ||
        robot_size.size() != 2 || speed_limits.size() != 2 ||
        acceleration_limits.size() != 2 || laser_pose.size() != 3 ||
        scan_angle_increment_ <= 0.0 || scan_angle_max_ <= scan_angle_min_ ||
        scan_range_min_ < 0.0 || scan_range_max_ <= scan_range_min_) {
      throw std::invalid_argument("invalid neupan_sim parameters");
    }

    config.min_x = bounds[0];
    config.max_x = bounds[1];
    config.min_y = bounds[2];
    config.max_y = bounds[3];
    config.robot_length = robot_size[0];
    config.robot_width = robot_size[1];
    if (!robot_vertices.empty() &&
        (robot_vertices.size() < 6 || robot_vertices.size() % 2 != 0))
      throw std::invalid_argument("robot_vertices must be [x1, y1, x2, y2, ...], >= 3 vertices");
    for (std::size_t i = 0; i < robot_vertices.size(); i += 2)
      config.robot_vertices.push_back({robot_vertices[i], robot_vertices[i + 1]});
    config.max_linear_speed = speed_limits[0];
    config.max_angular_speed = speed_limits[1];
    config.max_linear_acceleration = acceleration_limits[0];
    config.max_angular_acceleration = acceleration_limits[1];
    laser_pose_ = {laser_pose[0], laser_pose[1], laser_pose[2]};
    path_waypoints_ = parseWaypoints(path_values);

    auto circles = parseCircles(static_values, false);
    auto moving_circles = parseCircles(dynamic_values, true);
    circles.insert(circles.end(), moving_circles.begin(), moving_circles.end());
    simulation_ = std::make_unique<Simulation>(
        config, Pose2{initial[0], initial[1], initial[2]},
        path_waypoints_.back(), std::move(circles),
        parseSegments(segment_values));

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "neupan_cmd_vel", rclcpp::QoS(1),
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
          simulation_->setCommand({msg->linear.x, msg->angular.z});
        });
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    // The simulator is a deterministic local source, so offer reliable sensor
    // streams. Best-effort planner subscriptions remain compatible, while RViz
    // can use either reliability policy without silently losing the display.
    const auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
    scan_pub_ =
        create_publisher<sensor_msgs::msg::LaserScan>("scan", sensor_qos);
    obstacle_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "obstacles", sensor_qos);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "initial_path", rclcpp::QoS(1).transient_local());
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "neupan_sim/markers", rclcpp::QoS(1).transient_local());
    diagnostics_pub_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "neupan_sim/diagnostics", rclcpp::QoS(1).transient_local());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    physics_period_ = 1.0 / physics_rate_;
    sensor_period_ = 1.0 / sensor_rate_;
    diagnostics_period_ = 1.0 / diagnostics_rate_;
    if (!external_step) timer_ = create_wall_timer(std::chrono::duration<double>(physics_period_),
                               [this] { update(); });

    const auto stamp = now();
    publishPath(stamp);
    publishState(stamp);
    publishDiagnostics(stamp);
    publishMarkers(stamp);
    RCLCPP_INFO(get_logger(),
                "continuous NeuPAN simulator ready: scenario='%s', path=%zu "
                "waypoints, circles=%zu, segments=%zu",
                scenario_name_.c_str(), path_waypoints_.size(),
                simulation_->circles().size(), simulation_->segments().size());
  }

  Simulation* simulation() { return simulation_.get(); }
  bool started() const { return !paused_; }
  double physicsPeriod() const { return physics_period_; }

  void prepareTick() {
    // Publish sensors from the preceding physics snapshot. Its matching TF was
    // broadcast one timer tick earlier, avoiding cross-topic DDS ordering races.
    sensor_accumulator_ += physics_period_;
    if (sensor_accumulator_ + 1e-12 >= sensor_period_) {
      sensor_accumulator_ = std::fmod(sensor_accumulator_, sensor_period_);
      publishSensors(last_state_stamp_);
    }

  }

  void finishTick() {
    const auto stamp = now();
    publishState(stamp);

    diagnostics_accumulator_ += physics_period_;
    path_accumulator_ += physics_period_;
    if (diagnostics_accumulator_ + 1e-12 >= diagnostics_period_) {
      diagnostics_accumulator_ =
          std::fmod(diagnostics_accumulator_, diagnostics_period_);
      publishDiagnostics(stamp);
      publishMarkers(stamp);
    }
    // Republish for volatile global-path subscribers that start late.
    if (path_accumulator_ >= 1.0) {
      path_accumulator_ = std::fmod(path_accumulator_, 1.0);
      publishPath(stamp);
    }

    if (simulation_->result() != last_result_) {
      last_result_ = simulation_->result();
      if (last_result_ == neupan_sim::Result::GoalReached) {
        RCLCPP_INFO(get_logger(),
                    "scenario complete ('%s'): time=%.3f s, path=%.3f m, "
                    "minimum_clearance=%.3f m",
                    scenario_name_.c_str(), simulation_->elapsedTime(),
                    simulation_->pathLength(), simulation_->minimumClearance());
      } else if (last_result_ != neupan_sim::Result::Running) {
        RCLCPP_ERROR(get_logger(),
                     "scenario failed ('%s', %s): time=%.3f s, "
                     "goal_distance=%.3f m, minimum_clearance=%.3f m",
                     scenario_name_.c_str(), neupan_sim::resultName(last_result_),
                     simulation_->elapsedTime(), simulation_->goalDistance(),
                     simulation_->minimumClearance());
      }
    }
  }

 private:
  void update() {
    prepareTick();
    if (!paused_) simulation_->step(physics_period_);
    finishTick();
  }

  void publishState(const rclcpp::Time& stamp) {
    const auto& pose = simulation_->pose();
    const auto& velocity = simulation_->velocity();
    const auto orientation = yawToQuaternion(pose.yaw);

    geometry_msgs::msg::TransformStamped map_to_base;
    map_to_base.header.stamp = stamp;
    map_to_base.header.frame_id = world_frame_;
    map_to_base.child_frame_id = base_frame_;
    map_to_base.transform.translation.x = pose.x;
    map_to_base.transform.translation.y = pose.y;
    map_to_base.transform.rotation = orientation;
    tf_broadcaster_->sendTransform(map_to_base);

    geometry_msgs::msg::TransformStamped base_to_laser;
    base_to_laser.header.stamp = stamp;
    base_to_laser.header.frame_id = base_frame_;
    base_to_laser.child_frame_id = laser_frame_;
    base_to_laser.transform.translation.x = laser_pose_.x;
    base_to_laser.transform.translation.y = laser_pose_.y;
    base_to_laser.transform.rotation = yawToQuaternion(laser_pose_.yaw);
    tf_broadcaster_->sendTransform(base_to_laser);

    nav_msgs::msg::Odometry odom;
    odom.header = map_to_base.header;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = pose.x;
    odom.pose.pose.position.y = pose.y;
    odom.pose.pose.orientation = orientation;
    odom.twist.twist.linear.x = velocity.linear;
    odom.twist.twist.angular.z = velocity.angular;
    odom_pub_->publish(odom);
    last_state_stamp_ = stamp;

    if (trajectory_.empty() ||
        std::hypot(trajectory_.back().x - pose.x,
                   trajectory_.back().y - pose.y) >= 0.01) {
      trajectory_.push_back({pose.x, pose.y});
    }
  }

  void publishSensors(const rclcpp::Time& stamp) {
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = stamp;
    scan.header.frame_id = laser_frame_;
    scan.angle_min = static_cast<float>(scan_angle_min_);
    scan.angle_max = static_cast<float>(scan_angle_max_);
    scan.angle_increment = static_cast<float>(scan_angle_increment_);
    scan.scan_time = static_cast<float>(sensor_period_);
    scan.time_increment = 0.0F;
    scan.range_min = static_cast<float>(scan_range_min_);
    scan.range_max = static_cast<float>(scan_range_max_);

    const auto beam_count = static_cast<std::size_t>(
        std::floor((scan_angle_max_ - scan_angle_min_) /
                   scan_angle_increment_)) +
                            1;
    scan.ranges.resize(beam_count, std::numeric_limits<float>::infinity());

    struct CloudPoint {
      float x;
      float y;
      float intensity;
      float vx;
      float vy;
    };
    std::vector<CloudPoint> points;
    points.reserve(beam_count);
    const Pose2 sensor_pose =
        neupan_sim::composePose(simulation_->pose(), laser_pose_);
    const double c = std::cos(sensor_pose.yaw);
    const double s = std::sin(sensor_pose.yaw);

    last_peer_hits_ = 0;
    for (std::size_t i = 0; i < beam_count; ++i) {
      const double local_angle = scan_angle_min_ + i * scan_angle_increment_;
      const auto hit = simulation_->raycast(sensor_pose, local_angle,
                                            scan_range_min_, scan_range_max_);
      if (!hit.hit) continue;
      if (hit.peer) { ++last_peer_hits_; ++total_peer_hits_; }
      scan.ranges[i] = static_cast<float>(hit.range);
      const double dx = hit.point.x - sensor_pose.x;
      const double dy = hit.point.y - sensor_pose.y;
      points.push_back({static_cast<float>(c * dx + s * dy),
                        static_cast<float>(-s * dx + c * dy),
                        hit.dynamic ? 1.0F : 0.0F,
                        static_cast<float>(c * hit.velocity.x +
                                           s * hit.velocity.y),
                        static_cast<float>(-s * hit.velocity.x +
                                           c * hit.velocity.y)});
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = scan.header;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
        6, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
        sensor_msgs::msg::PointField::FLOAT32, "z", 1,
        sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
        sensor_msgs::msg::PointField::FLOAT32, "vx", 1,
        sensor_msgs::msg::PointField::FLOAT32, "vy", 1,
        sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> px(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> py(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> pz(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> intensity(cloud, "intensity");
    sensor_msgs::PointCloud2Iterator<float> pvx(cloud, "vx");
    sensor_msgs::PointCloud2Iterator<float> pvy(cloud, "vy");
    for (const auto& point : points) {
      *px = point.x;
      *py = point.y;
      *pz = 0.0F;
      *intensity = point.intensity;
      *pvx = point.vx;
      *pvy = point.vy;
      ++px;
      ++py;
      ++pz;
      ++intensity;
      ++pvx;
      ++pvy;
    }
    last_scan_hits_ = points.size();
    scan_pub_->publish(scan);
    obstacle_cloud_pub_->publish(cloud);
  }

  void publishPath(const rclcpp::Time& stamp) {
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = world_frame_;
    for (std::size_t segment = 0; segment + 1 < path_waypoints_.size();
         ++segment) {
      const Vec2 first = path_waypoints_[segment];
      const Vec2 second = path_waypoints_[segment + 1];
      const double dx = second.x - first.x;
      const double dy = second.y - first.y;
      const double length = std::hypot(dx, dy);
      if (length <= 1e-12) continue;
      const auto samples = std::max<std::size_t>(
          1, static_cast<std::size_t>(std::ceil(length / path_spacing_)));
      const double heading = std::atan2(dy, dx);
      for (std::size_t i = 0; i < samples; ++i) {
        const double ratio = static_cast<double>(i) / samples;
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = first.x + ratio * dx;
        pose.pose.position.y = first.y + ratio * dy;
        pose.pose.orientation = yawToQuaternion(heading);
        path.poses.push_back(std::move(pose));
      }
    }
    geometry_msgs::msg::PoseStamped goal;
    goal.header = path.header;
    goal.pose.position.x = path_waypoints_.back().x;
    goal.pose.position.y = path_waypoints_.back().y;
    if (path.poses.empty())
      throw std::runtime_error("path_waypoints contains no nonzero segment");
    goal.pose.orientation = path.poses.back().pose.orientation;
    path.poses.push_back(std::move(goal));
    path_pub_->publish(path);
  }

  void publishDiagnostics(const rclcpp::Time& stamp) {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = stamp;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "neupan_sim/scenario";
    status.hardware_id = "continuous_2d";
    status.message = neupan_sim::resultName(simulation_->result());
    status.level = simulation_->result() == neupan_sim::Result::Running ||
                           simulation_->result() ==
                               neupan_sim::Result::GoalReached
                       ? diagnostic_msgs::msg::DiagnosticStatus::OK
                       : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    addDiagnostic(status, "result", status.message);
    addDiagnostic(status, "paused", paused_ ? "true" : "false");
    addDiagnostic(status, "elapsed_time", simulation_->elapsedTime());
    addDiagnostic(status, "goal_distance", simulation_->goalDistance());
    addDiagnostic(status, "path_length", simulation_->pathLength());
    addDiagnostic(status, "clearance", simulation_->clearance());
    addDiagnostic(status, "minimum_clearance",
                  simulation_->minimumClearance());
    addDiagnostic(status, "actual_linear_speed",
                  simulation_->velocity().linear);
    addDiagnostic(status, "actual_angular_speed",
                  simulation_->velocity().angular);
    addDiagnostic(status, "command_age", simulation_->commandAge());
    addDiagnostic(status, "scan_hits", std::to_string(last_scan_hits_));
    addDiagnostic(status, "peer_count", std::to_string(simulation_->peerCount()));
    addDiagnostic(status, "peer_hits", std::to_string(last_peer_hits_));
    addDiagnostic(status, "total_peer_hits", std::to_string(total_peer_hits_));
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(array);
  }

  visualization_msgs::msg::Marker marker(
      const rclcpp::Time& stamp, std::string ns, int id, int type) const {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = world_frame_;
    marker.ns = std::move(ns);
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.color.a = 1.0F;
    return marker;
  }

  void publishMarkers(const rclcpp::Time& stamp) {
    visualization_msgs::msg::MarkerArray array;
    const auto& cfg = simulation_->config();
    if (show_world_) {
      int id = 0;
      int velocity_id = 0;
      for (const auto& obstacle : simulation_->circles()) {
        auto item = marker(stamp, "circles", id++,
                           visualization_msgs::msg::Marker::CYLINDER);
        item.pose.position.x = obstacle.center.x;
        item.pose.position.y = obstacle.center.y;
        item.scale.x = item.scale.y = 2.0 * obstacle.radius;
        item.scale.z = 0.4;
        const bool dynamic =
            std::hypot(obstacle.velocity.x, obstacle.velocity.y) > 1e-10;
        item.color.r = dynamic ? 0.9F : 0.4F;
        item.color.g = dynamic ? 0.2F : 0.4F;
        item.color.b = dynamic ? 0.1F : 0.4F;
        array.markers.push_back(std::move(item));
        if (dynamic) {
          auto velocity = marker(stamp, "obstacle_velocity", velocity_id++,
                                 visualization_msgs::msg::Marker::ARROW);
          velocity.scale.x = 0.04;
          velocity.scale.y = 0.09;
          velocity.scale.z = 0.12;
          velocity.color.r = 1.0F;
          velocity.color.g = 0.6F;
          geometry_msgs::msg::Point first;
          first.x = obstacle.center.x;
          first.y = obstacle.center.y;
          geometry_msgs::msg::Point second = first;
          second.x += obstacle.velocity.x;
          second.y += obstacle.velocity.y;
          velocity.points.push_back(first);
          velocity.points.push_back(second);
          array.markers.push_back(std::move(velocity));
        }
      }

      auto floor = marker(stamp, "world", 0,
                          visualization_msgs::msg::Marker::CUBE);
      floor.pose.position.x = 0.5 * (cfg.min_x + cfg.max_x);
      floor.pose.position.y = 0.5 * (cfg.min_y + cfg.max_y);
      floor.pose.position.z = -0.035;
      floor.scale.x = cfg.max_x - cfg.min_x;
      floor.scale.y = cfg.max_y - cfg.min_y;
      floor.scale.z = 0.02;
      floor.color.r = 0.12F;
      floor.color.g = 0.14F;
      floor.color.b = 0.17F;
      floor.color.a = 0.65F;
      array.markers.push_back(std::move(floor));

      auto segments = marker(stamp, "segments", 0,
                             visualization_msgs::msg::Marker::LINE_LIST);
      segments.scale.x = 0.07;
      segments.color.r = 0.55F;
      segments.color.g = 0.58F;
      segments.color.b = 0.62F;
      for (const auto& segment : simulation_->segments()) {
        geometry_msgs::msg::Point first;
        first.x = segment.first.x;
        first.y = segment.first.y;
        geometry_msgs::msg::Point second;
        second.x = segment.second.x;
        second.y = segment.second.y;
        segments.points.push_back(first);
        segments.points.push_back(second);
      }
      array.markers.push_back(std::move(segments));

      auto boundary = marker(stamp, "world", 1,
                             visualization_msgs::msg::Marker::LINE_STRIP);
      boundary.scale.x = 0.04;
      boundary.color.r = 0.35F;
      boundary.color.g = 0.42F;
      boundary.color.b = 0.50F;
      const std::array<Vec2, 5> boundary_points{{
          {cfg.min_x, cfg.min_y},
          {cfg.max_x, cfg.min_y},
          {cfg.max_x, cfg.max_y},
          {cfg.min_x, cfg.max_y},
          {cfg.min_x, cfg.min_y},
      }};
      for (const Vec2 point : boundary_points) {
        geometry_msgs::msg::Point output;
        output.x = point.x;
        output.y = point.y;
        boundary.points.push_back(output);
      }
      array.markers.push_back(std::move(boundary));
    }

    auto robot = marker(stamp, "robot", 0,
                        visualization_msgs::msg::Marker::CUBE);
    robot.pose.position.x = simulation_->pose().x;
    robot.pose.position.y = simulation_->pose().y;
    robot.pose.orientation = yawToQuaternion(simulation_->pose().yaw);
    robot.scale.x = cfg.robot_length;
    robot.scale.y = cfg.robot_width;
    robot.scale.z = 0.2;
    robot.color = marker_color_;
    if (!cfg.robot_vertices.empty()) {
      robot.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
      robot.scale.x = robot.scale.y = robot.scale.z = 1.0;
      for (std::size_t i = 1; i + 1 < cfg.robot_vertices.size(); ++i) {
        const auto& a = cfg.robot_vertices[0];
        const auto& b = cfg.robot_vertices[i];
        const auto& c = cfg.robot_vertices[i + 1];
        const bool ccw = (b.x-a.x)*(c.y-a.y) - (b.y-a.y)*(c.x-a.x) >= 0.0;
        for (const std::size_t j : {std::size_t(0), ccw ? i : i + 1, ccw ? i + 1 : i}) {
          geometry_msgs::msg::Point point;
          point.x = cfg.robot_vertices[j].x;
          point.y = cfg.robot_vertices[j].y;
          point.z = 0.1;
          robot.points.push_back(point);
          robot.colors.push_back(marker_color_);
        }
      }
    }
    array.markers.push_back(std::move(robot));

    auto heading = marker(stamp, "heading", 0, visualization_msgs::msg::Marker::ARROW);
    heading.pose.position.x = simulation_->pose().x;
    heading.pose.position.y = simulation_->pose().y;
    heading.pose.position.z = 0.13;
    heading.pose.orientation = yawToQuaternion(simulation_->pose().yaw);
    heading.scale.x = 0.30; heading.scale.y = 0.045; heading.scale.z = 0.055;
    heading.color.r = heading.color.g = heading.color.b = 1.0F;
    array.markers.push_back(std::move(heading));
    if (!robot_label_.empty()) {
      auto label = marker(stamp, "robot_label", 0, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      label.pose.position.x = simulation_->pose().x;
      label.pose.position.y = simulation_->pose().y + 0.48;
      label.pose.position.z = 0.2;
      label.scale.z = 0.14;
      label.color = marker_color_;
      label.text = get_namespace();
      if (!label.text.empty() && label.text.front() == '/') label.text.erase(0, 1);
      array.markers.push_back(std::move(label));
    }

    auto goal = marker(stamp, "goal", 0,
                       visualization_msgs::msg::Marker::SPHERE);
    goal.pose.position.x = simulation_->goal().x;
    goal.pose.position.y = simulation_->goal().y;
    goal.scale.x = goal.scale.y = goal.scale.z = 0.2;
    goal.color = marker_color_;
    goal.color.a = 0.45F;
    array.markers.push_back(std::move(goal));

    auto trace = marker(stamp, "trajectory", 0,
                        visualization_msgs::msg::Marker::LINE_STRIP);
    trace.scale.x = 0.03;
    trace.color = marker_color_;
    trace.color.a = 0.8F;
    for (const auto& point : trajectory_) {
      geometry_msgs::msg::Point output;
      output.x = point.x;
      output.y = point.y;
      trace.points.push_back(output);
    }
    array.markers.push_back(std::move(trace));

    auto status = marker(stamp, "status", 0,
                         visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    status.pose.position.x = simulation_->peerCount() ? simulation_->pose().x : cfg.min_x + 0.3;
    status.pose.position.y = simulation_->peerCount() ? simulation_->pose().y + 0.7 : cfg.max_y - 0.35;
    status.pose.position.z = 0.4;
    status.scale.z = 0.24;
    const bool failed =
        simulation_->result() == neupan_sim::Result::Collision ||
        simulation_->result() == neupan_sim::Result::TimedOut;
    status.color.r = failed ? 1.0F : 0.1F;
    status.color.g = failed ? 0.1F : 0.8F;
    status.color.b = 0.1F;
    std::ostringstream text;
    text << scenario_name_ << " | NeuPAN: " << std::fixed
         << std::setprecision(2) << neupan_sim::resultName(simulation_->result())
         << "  t=" << simulation_->elapsedTime() << "s"
         << "  goal=" << simulation_->goalDistance() << "m"
         << "  min_clearance=" << simulation_->minimumClearance() << "m";
    if (!status_position_.empty()) {
      status.pose.position.x = status_position_[0];
      status.pose.position.y = status_position_[1];
      status.scale.z = 0.18;
      status.color = marker_color_;
      text.str(""); text.clear();
      text << robot_label_ << "\n" << neupan_sim::resultName(simulation_->result())
           << "\n" << simulation_->elapsedTime() << "s/"
           << simulation_->minimumClearance() << "m";
    }
    status.text = text.str();
    array.markers.push_back(std::move(status));
    marker_pub_->publish(array);
  }

  std::unique_ptr<Simulation> simulation_;
  std::vector<Vec2> path_waypoints_;
  std::vector<Vec2> trajectory_;
  std::string scenario_name_;
  std::string world_frame_, base_frame_, laser_frame_;
  std_msgs::msg::ColorRGBA marker_color_;
  std::vector<double> status_position_;
  std::string robot_label_;
  bool show_world_ = true;
  bool paused_ = false;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  Pose2 laser_pose_;
  neupan_sim::Result last_result_ = neupan_sim::Result::Running;
  double physics_rate_ = 100.0;
  double sensor_rate_ = 20.0;
  double diagnostics_rate_ = 10.0;
  double physics_period_ = 0.01;
  double sensor_period_ = 0.05;
  double diagnostics_period_ = 0.1;
  double sensor_accumulator_ = 0.0;
  double diagnostics_accumulator_ = 0.0;
  double path_accumulator_ = 0.0;
  double path_spacing_ = 0.1;
  double scan_angle_min_ = -M_PI;
  double scan_angle_max_ = M_PI;
  double scan_angle_increment_ = M_PI / 180.0;
  double scan_range_min_ = 0.05;
  double scan_range_max_ = 8.0;
  std::size_t last_scan_hits_ = 0;
  std::size_t last_peer_hits_ = 0, total_peer_hits_ = 0;
  rclcpp::Time last_state_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      obstacle_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#ifndef NEUPAN_FLEET_MAIN
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NeupanSimNode>());
  rclcpp::shutdown();
  return 0;
}

#else
// One executor owns every body: command callbacks never interleave a physics tick.
class FleetNode final : public rclcpp::Node {
 public:
  FleetNode() : Node("neupan_fleet") {
    const auto names = declare_parameter<std::vector<std::string>>("robot_names", std::vector<std::string>{});
    const auto files = declare_parameter<std::vector<std::string>>("simulator_files", std::vector<std::string>{});
    if (names.size() < 2 || names.size() != files.size())
      throw std::invalid_argument("fleet needs >= 2 robot_names and matching simulator_files");
    std::set<std::string> unique_names, frames;
    for (std::size_t i = 0; i < names.size(); ++i) {
      if (names[i].empty() || names[i].find('/') != std::string::npos || !unique_names.insert(names[i]).second)
        throw std::invalid_argument("fleet robot names must be unique namespace components");
      const auto fqn = "/" + names[i] + "/neupan_sim";
      auto options = rclcpp::NodeOptions().use_global_arguments(false);
      options.arguments({"--ros-args", "-r", "__ns:=/" + names[i]});
      options.parameter_overrides(rclcpp::parameters_from_map(
          rclcpp::parameter_map_from_yaml_file(files[i], fqn.c_str()), fqn.c_str()));
      auto node = std::make_shared<NeupanSimNode>(options, true);
      for (const auto* key : {"base_frame", "laser_frame"}) {
        if (!frames.insert(node->get_parameter(key).as_string()).second)
          throw std::invalid_argument("fleet base/laser frames must be unique");
      }
      if (!nodes_.empty()) {
        for (const auto* key : {"world_frame", "world_bounds", "static_circles",
                                "dynamic_circles", "segments", "physics_rate"}) {
          if (node->get_parameter(key).get_parameter_value() !=
              nodes_.front()->get_parameter(key).get_parameter_value())
            throw std::invalid_argument(std::string("fleet has inconsistent shared parameter: ") + key);
        }
      }
      simulations_.push_back(node->simulation());
      nodes_.push_back(node);
    }
    if (frames.count(nodes_.front()->get_parameter("world_frame").as_string()))
      throw std::invalid_argument("world frame cannot be a robot frame");
    Simulation::synchronizeRobots(simulations_);
    timer_ = create_wall_timer(std::chrono::duration<double>(nodes_.front()->physicsPeriod()), [this] {
      for (const auto& node : nodes_) node->prepareTick();
      if (std::all_of(nodes_.begin(), nodes_.end(), [](const auto& node) { return node->started(); }))
        Simulation::stepTogether(simulations_, nodes_.front()->physicsPeriod());
      for (const auto& node : nodes_) node->finishTick();
    });
    RCLCPP_INFO(get_logger(), "shared world ready: %zu robots", nodes_.size());
  }
  const auto& nodes() const { return nodes_; }
 private:
  std::vector<std::shared_ptr<NeupanSimNode>> nodes_;
  std::vector<Simulation*> simulations_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto fleet = std::make_shared<FleetNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(fleet);
    for (const auto& node : fleet->nodes()) executor.add_node(node);
    executor.spin();
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("neupan_fleet"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
#endif
