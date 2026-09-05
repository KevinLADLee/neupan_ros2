/*
 * neupan_ros: native ROS2 wrapper for the CPU-oriented NeuPAN C++ core.
 *
 * Ported from neupan_ros (https://github.com/hanruihua/neupan_ros),
 * Copyright (c) 2025 Ruihua Han <hanrh@connect.hku.hk>.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version. See <https://www.gnu.org/licenses/>.
 */

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "neupan/neupan_planner.hpp"
#include "neupan_ros/laser_scan_preprocessor.hpp"
#include "neupan_ros/obstacle_observation.hpp"
#include "neupan_ros/pointcloud_preprocessor.hpp"

using namespace std::chrono_literals;

namespace neupan_ros {
namespace {

enum class ObstacleSource { Scan, PointCloud, Auto };
enum class ReferenceSource { None, Path, Waypoints, Goal };

ObstacleSource parseObstacleSource(const std::string& value) {
  if (value == "scan") return ObstacleSource::Scan;
  if (value == "pointcloud") return ObstacleSource::PointCloud;
  if (value == "auto") return ObstacleSource::Auto;
  throw std::invalid_argument(
      "obstacle_source must be 'scan', 'pointcloud', or 'auto'");
}

const char* obstacleSourceName(ObstacleSource source) {
  switch (source) {
    case ObstacleSource::Scan:
      return "scan";
    case ObstacleSource::PointCloud:
      return "pointcloud";
    case ObstacleSource::Auto:
      return "auto";
  }
  return "unknown";
}

double quatToYaw(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.z * q.z + q.y * q.y));
}

geometry_msgs::msg::Quaternion yawToQuat(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

}  // namespace

class NeuPANNode final : public rclcpp::Node {
 public:
  explicit NeuPANNode(const rclcpp::NodeOptions& options)
      : Node("neupan_node", options) {
    const auto config_file = declare_parameter<std::string>("config_file", "");
    const auto dune_checkpoint =
        declare_parameter<std::string>("dune_checkpoint", "");
    planning_frame_ =
        declare_parameter<std::string>("planning_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    pose_timeout_ = declare_parameter<double>("pose_timeout", 0.5);
    marker_size_ = declare_parameter<double>("marker_size", 0.05);
    marker_z_ = declare_parameter<double>("marker_z", 1.0);
    scan_config_.angle_min =
        declare_parameter<double>("scan_angle_min", -3.14);
    scan_config_.angle_max =
        declare_parameter<double>("scan_angle_max", 3.14);
    scan_config_.range_min =
        declare_parameter<double>("scan_range_min", 0.0);
    scan_config_.range_max =
        declare_parameter<double>("scan_range_max", 5.0);
    scan_config_.downsample = declare_parameter<int>("scan_downsample", 1);
    scan_config_.flip_angle = declare_parameter<bool>("flip_angle", false);
    include_initial_path_direction_ =
        declare_parameter<bool>("include_initial_path_direction", false);
    scan_timeout_ = declare_parameter<double>("scan_timeout", 0.5);
    obstacle_source_ = parseObstacleSource(
        declare_parameter<std::string>("obstacle_source", "auto"));
    pointcloud_timeout_ = declare_parameter<double>("pointcloud_timeout", 0.5);
    compensate_obstacle_latency_ =
        declare_parameter<bool>("compensate_obstacle_latency", true);
    solver_fail_grace_ = declare_parameter<int>("solver_fail_grace", 5);
    stall_speed_ = declare_parameter<double>("stall_speed", 0.02);
    stall_timeout_ = declare_parameter<double>("stall_timeout", 3.0);
    const double rate = declare_parameter<double>("control_rate", 50.0);

    if (planning_frame_.empty() || base_frame_.empty())
      throw std::runtime_error(
          "planning_frame and base_frame must not be empty");
    if (!std::isfinite(pose_timeout_) || pose_timeout_ <= 0.0)
      throw std::runtime_error("pose_timeout must be finite and > 0");
    if (scan_config_.downsample < 1 || scan_timeout_ <= 0.0 ||
        pointcloud_timeout_ <= 0.0)
      throw std::runtime_error(
          "scan_downsample must be >= 1 and obstacle timeouts must be > 0");

    if (config_file.empty())
      throw std::runtime_error("parameter 'config_file' is required");
    planner_ = std::make_unique<neupan::NeuPANPlanner>(
        neupan::NeuPANPlanner::fromYaml(config_file, dune_checkpoint));

    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("neupan_cmd_vel", 10);
    plan_pub_ = create_publisher<nav_msgs::msg::Path>("neupan_plan", 10);
    ref_state_pub_ =
        create_publisher<nav_msgs::msg::Path>("neupan_ref_state", 10);
    ref_path_pub_ =
        create_publisher<nav_msgs::msg::Path>("neupan_initial_path", 10);
    dune_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "dune_point_markers", 10);
    nrmp_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "nrmp_point_markers", 10);
    robot_marker_pub_ =
        create_publisher<visualization_msgs::msg::Marker>("robot_marker", 10);
    // A decision layer may drive its chassis output off this, so publish every cycle.
    arrive_pub_ = create_publisher<std_msgs::msg::Bool>("neupan_arrive", 10);
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "neupan_diagnostics", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    if (obstacle_source_ != ObstacleSource::PointCloud) {
      scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
          "scan", rclcpp::SensorDataQoS(),
          [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
            scanCallback(*msg);
          });
    }
    if (obstacle_source_ != ObstacleSource::Scan) {
      pointcloud_sub_ =
          create_subscription<sensor_msgs::msg::PointCloud2>(
              "obstacles", rclcpp::SensorDataQoS(),
              [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
                pointCloudCallback(*msg);
              });
    }
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "initial_path", 10, [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
          pathCallback(*msg);
        });
    waypoints_sub_ = create_subscription<nav_msgs::msg::Path>(
        "neupan_waypoints", 10,
        [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
          waypointsCallback(*msg);
        });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "neupan_goal", 10,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
          goalCallback(*msg);
        });

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate), [this] { run(); });

    RCLCPP_INFO(
        get_logger(),
        "native NeuPAN node ready (config: %s, planning frame: %s, "
        "obstacle source: %s)",
        config_file.c_str(), planning_frame_.c_str(),
        obstacleSourceName(obstacle_source_));
  }

 private:
  std::optional<neupan::Vec3> lookupPose(const std::string& target,
                                      const std::string& source,
                                      double max_age = 0.0) {
    if (target == source) return neupan::Vec3::Zero();
    try {
      const auto tfs = tf_buffer_->lookupTransform(target, source,
                                                   tf2::TimePointZero);
      // Latest TF remains cached after its publisher stops. Only the robot
      // pose needs this age bound; reference-frame transforms may be static.
      if (max_age > 0.0) {
        const double age = (now() - rclcpp::Time(tfs.header.stamp)).seconds();
        if (age > max_age || age < -0.05) {
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "robot pose TF %s <- %s has invalid age %.3f s (timeout %.3f s)",
              target.c_str(), source.c_str(), age, max_age);
          return std::nullopt;
        }
      }
      return neupan::Vec3(tfs.transform.translation.x,
                          tfs.transform.translation.y,
                          quatToYaw(tfs.transform.rotation));
    } catch (const tf2::TransformException& ex) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "waiting for tf %s -> %s: %s", target.c_str(),
                           source.c_str(), ex.what());
      return std::nullopt;
    }
  }

  std::optional<neupan::Vec3> lookupPoseAt(const std::string& target,
                                           const std::string& source,
                                           const rclcpp::Time& stamp) {
    if (target == source) return neupan::Vec3::Zero();
    try {
      const auto tfs = tf_buffer_->lookupTransform(target, source, stamp);
      return neupan::Vec3(tfs.transform.translation.x,
                          tfs.transform.translation.y,
                          quatToYaw(tfs.transform.rotation));
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "waiting for timestamped tf %s -> %s: %s",
                           target.c_str(), source.c_str(), ex.what());
      return std::nullopt;
    }
  }

  bool requireFrame(const std::string& frame, const char* input_name) {
    if (!frame.empty()) return true;
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "rejecting %s: header.frame_id must not be empty",
        input_name);
    return false;
  }

  static double normalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  static neupan::Vec3 transformPose(
      const geometry_msgs::msg::Pose& pose,
      const neupan::Vec3& target_from_source) {
    const double c = std::cos(target_from_source(2));
    const double s = std::sin(target_from_source(2));
    return neupan::Vec3(
        target_from_source(0) + c * pose.position.x - s * pose.position.y,
        target_from_source(1) + s * pose.position.x + c * pose.position.y,
        normalizeAngle(target_from_source(2) + quatToYaw(pose.orientation)));
  }

  // Publishing nothing is not a stop: consumers hold the last command.
  void halt(const char* why) {
    vel_pub_->publish(geometry_msgs::msg::Twist());
    publishArrive(false);
    publishStatus(diagnostic_msgs::msg::DiagnosticStatus::WARN, why, {});
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s", why);
  }

  void run() {
    const auto state = lookupPose(planning_frame_, base_frame_, pose_timeout_);
    if (!state) {
      halt("no fresh robot pose (TF planning_frame <- base_frame), holding still");
      return;
    }
    robot_state_ = *state;
    have_state_ = true;

    if (!syncReferenceInput()) {
      halt("no transform from reference input to planning_frame");
      return;
    }

    std::string obstacle_error;
    const TimedObservation* active = selectObstacleInput(obstacle_error);
    if (!active) {
      halt(obstacle_error.c_str());
      return;
    }

    auto& ipath = planner_->ipath();
    if (ipath.hasConfiguredWaypoints() && !ipath.hasPath())
      ipath.setIpathWithState(robot_state_);

    if (!ipath.hasPath()) {
      halt("waiting for neupan initial path");
      return;
    }

    publishInitialPath();

    const neupan::Mat2X* active_points = &active->observation.points;
    if (compensate_obstacle_latency_ && active->max_speed > 0.0 &&
        active_points->cols() > 0) {
      const double age =
          std::max(0.0, (now() - active->observed_at).seconds());
      compensated_obstacle_points_ =
          *active_points + age * active->observation.velocities;
      active_points = &compensated_obstacle_points_;
    }

    if (active_points->cols() == 0)
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "no obstacle points, only path tracking will be performed");

    neupan::NeuPANPlanner::Info info;
    const neupan::Vec2 action = planner_->forward(
        robot_state_, *active_points, active->observation.velocities, info);

    if (info.arrive)
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "arrive at the target");
    if (info.stop)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                           "neupan stop, min distance %.3f below threshold %.3f",
                           info.min_distance, planner_->collisionThreshold());

    geometry_msgs::msg::Twist twist;
    if (!info.stop && !info.arrive && !solverGaveUp(info)) {
      twist.linear.x = action(0);
      twist.angular.z = action(1);
    }
    vel_pub_->publish(twist);
    publishArrive(info.arrive);
    updateStall(twist, info.arrive);
    publishDiagnostics(info, twist);

    if (info.opt_s.cols() > 0) plan_pub_->publish(matToPath(info.opt_s));
    if (info.ref_s.cols() > 0) ref_state_pub_->publish(matToPath(info.ref_s));
    publishPointMarkers(info.dune_points, dune_markers_pub_, 160, 32, 240);
    publishPointMarkers(info.nrmp_points, nrmp_markers_pub_, 255, 128, 0);
    publishRobotMarker();
  }

  // A run of unsolved cycles means driving blind on a nominal with no avoidance.
  bool solverGaveUp(const neupan::NeuPANPlanner::Info& info) {
    if (info.solved) {
      consecutive_unsolved_ = 0;
      return false;
    }
    ++consecutive_unsolved_;
    if (consecutive_unsolved_ <= solver_fail_grace_) return false;

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                         "%d consecutive unsolved cycles (osqp status %d), stopping",
                         consecutive_unsolved_, info.solver_status);
    return true;
  }

  struct TimedObservation {
    neupan_ros::ObstacleObservation observation;
    rclcpp::Time received_at{0, 0, RCL_ROS_TIME};
    rclcpp::Time observed_at{0, 0, RCL_ROS_TIME};
    double max_speed = 0.0;
    bool seen = false;
  };

  bool fresh(const TimedObservation& input, double timeout) const {
    if (!input.seen) return false;
    const rclcpp::Time current = now();
    const double receive_age = (current - input.received_at).seconds();
    const double observation_age = (current - input.observed_at).seconds();
    return receive_age >= 0.0 && receive_age <= timeout &&
           observation_age >= -0.05 && observation_age <= timeout;
  }

  const TimedObservation* selectObstacleInput(std::string& error) {
    const bool cloud_fresh = fresh(pointcloud_observation_, pointcloud_timeout_);
    const bool scan_fresh = fresh(scan_observation_, scan_timeout_);

    const bool use_cloud =
        obstacle_source_ == ObstacleSource::PointCloud ||
        (obstacle_source_ == ObstacleSource::Auto && cloud_fresh);
    if (use_cloud) {
      if (!cloud_fresh) {
        error = pointcloud_observation_.seen
                    ? "obstacle point cloud is stale, holding still"
                    : "waiting for obstacle point cloud";
        return nullptr;
      }
      active_obstacle_source_ = "pointcloud";
      active_obstacle_format_ =
          neupan_ros::toString(pointcloud_observation_.observation.format);
      updateActiveObstacleDiagnostics(pointcloud_observation_);
      return &pointcloud_observation_;
    } else {
      if (!scan_fresh) {
        error = scan_observation_.seen
                    ? "scan is stale, obstacles unknown, holding still"
                    : "waiting for obstacle scan";
        return nullptr;
      }
      active_obstacle_source_ = "scan";
      active_obstacle_format_ =
          neupan_ros::toString(scan_observation_.observation.format);
      updateActiveObstacleDiagnostics(scan_observation_);
      return &scan_observation_;
    }
  }

  void updateActiveObstacleDiagnostics(
      const TimedObservation& input) {
    active_obstacle_count_ =
        static_cast<std::size_t>(input.observation.points.cols());
    active_max_obstacle_speed_ = input.max_speed;
  }

  void publishArrive(bool arrived) {
    std_msgs::msg::Bool msg;
    msg.data = arrived;
    arrive_pub_->publish(msg);
  }

  // Idling at the clearance in front of a blocked path sets neither stop nor arrive.
  void updateStall(const geometry_msgs::msg::Twist& cmd, bool arrived) {
    const bool moving = std::abs(cmd.linear.x) > stall_speed_ ||
                        std::abs(cmd.angular.z) > stall_speed_;
    // First planned cycle starts the clock; the zero epoch would flag instantly.
    const bool first = last_progress_time_.nanoseconds() == 0;
    if (moving || arrived || first) {
      last_progress_time_ = now();
      stalled_ = false;
      return;
    }
    if ((now() - last_progress_time_).seconds() > stall_timeout_) {
      stalled_ = true;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "stalled for %.1f s without arriving; the path may "
                           "be blocked and need a global replan",
                           stall_timeout_);
    }
  }

  void publishStatus(
      uint8_t level, const std::string& message,
      const std::vector<std::pair<std::string, std::string>>& values) {
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = "neupan";
    st.hardware_id = base_frame_;
    st.level = level;
    st.message = message;
    for (const auto& [k, v] : values) {
      diagnostic_msgs::msg::KeyValue e;
      e.key = k;
      e.value = v;
      st.values.push_back(e);
    }
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = now();
    arr.status.push_back(st);
    diag_pub_->publish(arr);
  }

  void publishDiagnostics(const neupan::NeuPANPlanner::Info& info,
                          const geometry_msgs::msg::Twist& cmd) {
    using diagnostic_msgs::msg::DiagnosticStatus;
    uint8_t level = DiagnosticStatus::OK;
    std::string message = "tracking";
    if (info.arrive) {  // arrival skips the solve, so check it first
      message = "arrived";
    } else if (stalled_ || consecutive_unsolved_ > solver_fail_grace_) {
      level = DiagnosticStatus::ERROR;
      message = stalled_ ? "stalled" : "solver failing";
    } else if (info.stop || !info.solved) {
      level = DiagnosticStatus::WARN;
      message = info.stop ? "stopped at collision threshold" : "unsolved";
    }

    publishStatus(
        level, message,
        {{"solved", info.solved ? "true" : "false"},
         {"solver_status", std::to_string(info.solver_status)},
         {"consecutive_unsolved", std::to_string(consecutive_unsolved_)},
         {"min_distance", std::to_string(info.min_distance)},
         {"obstacle_source", active_obstacle_source_},
         {"obstacle_format", active_obstacle_format_},
         {"obstacle_points", std::to_string(active_obstacle_count_)},
         {"max_obstacle_speed", std::to_string(active_max_obstacle_speed_)},
         {"planning_frame", planning_frame_},
         {"reference_frame", referenceFrame()},
         {"stalled", stalled_ ? "true" : "false"},
         {"cmd_v", std::to_string(cmd.linear.x)},
         {"cmd_w", std::to_string(cmd.angular.z)}});
  }

  void scanCallback(const sensor_msgs::msg::LaserScan& msg) {
    if (!requireFrame(msg.header.frame_id, "LaserScan")) return;
    const rclcpp::Time observation_time =
        msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0
            ? now()
            : rclcpp::Time(msg.header.stamp);
    auto observation = neupan_ros::preprocessLaserScan(msg, scan_config_);
    if (msg.header.frame_id != planning_frame_) {
      const auto planning_from_lidar = lookupPoseAt(
          planning_frame_, msg.header.frame_id, observation_time);
      if (!planning_from_lidar) return;
      neupan_ros::transformObservation(observation, *planning_from_lidar);
    }
    scan_observation_.observation = std::move(observation);
    scan_observation_.received_at = now();
    scan_observation_.observed_at = observation_time;
    scan_observation_.max_speed = 0.0;
    scan_observation_.seen = true;
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2& msg) {
    if (!requireFrame(msg.header.frame_id, "PointCloud2")) return;
    const rclcpp::Time observation_time =
        msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0
            ? now()
            : rclcpp::Time(msg.header.stamp);
    neupan_ros::ObstacleObservation observation;
    try {
      observation = neupan_ros::preprocessPointCloud(msg);
    } catch (const std::exception& ex) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "invalid obstacle PointCloud2: %s", ex.what());
      return;
    }

    if (msg.header.frame_id != planning_frame_) {
      const auto planning_from_sensor = lookupPoseAt(
          planning_frame_, msg.header.frame_id, observation_time);
      if (!planning_from_sensor) return;
      neupan_ros::transformObservation(observation, *planning_from_sensor);
    }
    if (observation.discarded_points > 0) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "discarded %zu non-finite or directionless PointCloud2 points",
          observation.discarded_points);
    }
    pointcloud_observation_.max_speed =
        observation.velocities.cols() == 0
            ? 0.0
            : observation.velocities.colwise().norm().maxCoeff();
    pointcloud_observation_.observation = std::move(observation);
    pointcloud_observation_.received_at = now();
    pointcloud_observation_.observed_at = observation_time;
    pointcloud_observation_.seen = true;
  }

  // Path poses -> [x, y, theta, gear]; theta from the points gradient unless
  // include_initial_path_direction is set (as upstream).
  std::vector<neupan::InitialPath::PathPoint> pathMsgToPoints(
      const nav_msgs::msg::Path& msg,
      const neupan::Vec3& target_from_source) const {
    std::vector<neupan::InitialPath::PathPoint> out;
    const size_t n = msg.poses.size();
    out.reserve(n);
    std::vector<neupan::Vec3> poses;
    poses.reserve(n);
    for (const auto& stamped_pose : msg.poses)
      poses.push_back(transformPose(stamped_pose.pose, target_from_source));
    for (size_t i = 0; i < n; ++i) {
      double theta;
      if (include_initial_path_direction_) {
        theta = poses[i](2);
      } else if (i + 1 < n) {
        theta = std::atan2(poses[i + 1](1) - poses[i](1),
                           poses[i + 1](0) - poses[i](0));
      } else {
        theta = out.empty() ? poses[i](2) : out.back()(2);
      }
      out.emplace_back(poses[i](0), poses[i](1), theta, 1.0);
    }
    return out;
  }

  void pathCallback(const nav_msgs::msg::Path& msg) {
    if (!requireFrame(msg.header.frame_id, "initial_path")) return;
    if (msg.poses.size() < 2) {
      RCLCPP_WARN(get_logger(), "ignoring initial path with < 2 poses");
      return;
    }
    reference_path_ = msg;
    reference_source_ = ReferenceSource::Path;
  }

  void waypointsCallback(const nav_msgs::msg::Path& msg) {
    if (!requireFrame(msg.header.frame_id, "neupan_waypoints")) return;
    if (msg.poses.empty()) return;
    reference_path_ = msg;
    reference_source_ = ReferenceSource::Waypoints;
  }

  static bool samePoints(
      const std::vector<neupan::InitialPath::PathPoint>& a,
      const std::vector<neupan::InitialPath::PathPoint>& b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i)
      if ((a[i] - b[i]).cwiseAbs().maxCoeff() > 1e-6) return false;
    return true;
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped& msg) {
    if (!requireFrame(msg.header.frame_id, "neupan_goal")) return;
    reference_goal_ = msg;
    reference_source_ = ReferenceSource::Goal;
  }

  std::string referenceFrame() const {
    if (reference_source_ == ReferenceSource::Goal)
      return reference_goal_.header.frame_id;
    if (reference_source_ == ReferenceSource::Path ||
        reference_source_ == ReferenceSource::Waypoints)
      return reference_path_.header.frame_id;
    return "none";
  }

  bool syncReferenceInput() {
    if (reference_source_ == ReferenceSource::None) return true;

    const std::string& source_frame =
        reference_source_ == ReferenceSource::Goal
            ? reference_goal_.header.frame_id
            : reference_path_.header.frame_id;
    const auto planning_from_source =
        lookupPose(planning_frame_, source_frame);
    if (!planning_from_source) return false;

    if (reference_source_ == ReferenceSource::Goal) {
      const neupan::Vec3 goal =
          transformPose(reference_goal_.pose, *planning_from_source);
      const bool unchanged =
          applied_reference_source_ == ReferenceSource::Goal &&
          (goal.head<2>() - last_reference_goal_.head<2>()).norm() <= 1e-6 &&
          std::abs(normalizeAngle(goal(2) - last_reference_goal_(2))) <= 1e-6;
      if (unchanged) return true;

      RCLCPP_INFO(get_logger(),
                  "set neupan goal in %s: [%.2f, %.2f, %.2f]",
                  planning_frame_.c_str(), goal(0), goal(1), goal(2));
      planner_->updateInitialPathFromGoal(robot_state_, goal);
      planner_->reset();
      last_reference_goal_ = goal;
      applied_reference_source_ = ReferenceSource::Goal;
      publishArrive(false);
      return true;
    }

    const auto points =
        pathMsgToPoints(reference_path_, *planning_from_source);
    if (applied_reference_source_ == reference_source_ &&
        samePoints(points, last_reference_points_))
      return true;

    if (reference_source_ == ReferenceSource::Path) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "initial path update in %s (%zu poses, source frame %s)",
          planning_frame_.c_str(), points.size(), source_frame.c_str());
      // Replacing preserves progress when a global-to-local transform changes.
      if (planner_->ipath().hasPath()) {
        planner_->replaceInitialPath(points, robot_state_);
      } else {
        planner_->setInitialPath(points);
        planner_->reset();
      }
    } else {
      std::vector<neupan::Vec3> waypoints{robot_state_};
      waypoints.reserve(points.size() + 1);
      for (const auto& point : points)
        waypoints.emplace_back(point.head<3>());
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "waypoint update in %s (%zu poses, source frame %s)",
          planning_frame_.c_str(), points.size(), source_frame.c_str());
      planner_->setWaypoints(waypoints);
      planner_->reset();
    }

    last_reference_points_ = points;
    applied_reference_source_ = reference_source_;
    publishArrive(false);
    return true;
  }

  nav_msgs::msg::Path matToPath(const neupan::Mat3X& states) {
    nav_msgs::msg::Path path;
    path.header.frame_id = planning_frame_;
    path.header.stamp = now();
    path.poses.reserve(static_cast<std::size_t>(states.cols()));
    for (Eigen::Index i = 0; i < states.cols(); ++i) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = planning_frame_;
      ps.pose.position.x = states(0, i);
      ps.pose.position.y = states(1, i);
      ps.pose.orientation = yawToQuat(states(2, i));
      path.poses.push_back(ps);
    }
    return path;
  }

  void publishInitialPath() {
    const auto& path = planner_->ipath().initialPath();
    nav_msgs::msg::Path msg;
    msg.header.frame_id = planning_frame_;
    msg.header.stamp = now();
    msg.poses.reserve(path.size());
    for (const auto& p : path) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = planning_frame_;
      ps.pose.position.x = p(0);
      ps.pose.position.y = p(1);
      ps.pose.orientation = yawToQuat(p(2));
      msg.poses.push_back(ps);
    }
    ref_path_pub_->publish(msg);
  }

  void publishPointMarkers(
      const neupan::Mat2X& points,
      const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr&
          pub,
      int r, int g, int b) {
    if (points.cols() == 0) return;
    visualization_msgs::msg::MarkerArray arr;
    arr.markers.reserve(static_cast<std::size_t>(points.cols()));
    for (Eigen::Index i = 0; i < points.cols(); ++i) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = planning_frame_;
      m.header.stamp = now();
      m.id = static_cast<int>(i);
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.scale.x = m.scale.y = m.scale.z = marker_size_;
      m.color.a = 1.0;
      m.color.r = r / 255.0f;
      m.color.g = g / 255.0f;
      m.color.b = b / 255.0f;
      m.pose.position.x = points(0, i);
      m.pose.position.y = points(1, i);
      m.pose.position.z = 0.3;
      m.pose.orientation.w = 1.0;
      arr.markers.push_back(m);
    }
    pub->publish(arr);
  }

  void publishRobotMarker() {
    const auto& cfg = planner_->config();
    visualization_msgs::msg::Marker m;
    m.header.frame_id = planning_frame_;
    m.header.stamp = now();
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.color.a = 1.0;
    m.color.g = 1.0;
    m.scale.x = cfg.length;
    m.scale.y = cfg.width;
    m.scale.z = marker_z_;
    m.pose.position.x = robot_state_(0);
    m.pose.position.y = robot_state_(1);
    m.pose.orientation = yawToQuat(robot_state_(2));
    robot_marker_pub_->publish(m);
  }

  std::unique_ptr<neupan::NeuPANPlanner> planner_;

  std::string planning_frame_, base_frame_;
  double pose_timeout_ = 0.5;
  std::string active_obstacle_source_ = "none";
  std::string active_obstacle_format_ = "none";
  ObstacleSource obstacle_source_ = ObstacleSource::Auto;
  double marker_size_, marker_z_;
  neupan_ros::LaserScanPreprocessorConfig scan_config_;
  bool include_initial_path_direction_;
  double scan_timeout_, pointcloud_timeout_, stall_speed_, stall_timeout_;
  bool compensate_obstacle_latency_ = true;
  int solver_fail_grace_;

  int consecutive_unsolved_ = 0;
  bool stalled_ = false;
  rclcpp::Time last_progress_time_{0, 0, RCL_ROS_TIME};
  ReferenceSource reference_source_ = ReferenceSource::None;
  ReferenceSource applied_reference_source_ = ReferenceSource::None;
  nav_msgs::msg::Path reference_path_;
  geometry_msgs::msg::PoseStamped reference_goal_;
  std::vector<neupan::InitialPath::PathPoint> last_reference_points_;
  neupan::Vec3 last_reference_goal_ = neupan::Vec3::Zero();

  neupan::Vec3 robot_state_ = neupan::Vec3::Zero();
  bool have_state_ = false;
  TimedObservation scan_observation_;
  TimedObservation pointcloud_observation_;
  neupan::Mat2X compensated_obstacle_points_ = neupan::Mat2X(2, 0);
  std::size_t active_obstacle_count_ = 0;
  double active_max_obstacle_speed_ = 0.0;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_, ref_state_pub_,
      ref_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      dune_markers_pub_, nrmp_markers_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arrive_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      robot_marker_pub_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      pointcloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_,
      waypoints_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace neupan_ros

RCLCPP_COMPONENTS_REGISTER_NODE(neupan_ros::NeuPANNode)
