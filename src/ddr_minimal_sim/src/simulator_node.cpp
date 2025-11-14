#include "ddr_minimal_sim/simulator.hpp"
#include <cmath>

namespace ddr_minimal_sim {

MinimalSimulator::MinimalSimulator()
  : Node("minimal_simulator"),
    sim_time_(0, 0, RCL_ROS_TIME),
    rng_(std::random_device{}()),
    noise_dist_(0.0, 1.0)
{
  declareParameters();
  loadParameters();
  validateParameters();

  current_pose_ << initial_pose_.x, initial_pose_.y, initial_pose_.yaw;
  current_velocity_.setZero();
  desired_velocity_.setZero();
  current_accel_.setZero();

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&MinimalSimulator::cmdVelCallback, this, std::placeholders::_1));

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  vis_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/vehicle_markers", 10);
  clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Use single timer to ensure update and publish are synchronized
  // Use the higher frequency to maintain accuracy
  double combined_frequency = std::max(simulation_.frequency, simulation_.publish_rate);
  double timer_period = 1.0 / combined_frequency;

  // Calculate decimation ratios
  update_decimation_ = static_cast<int>(std::round(combined_frequency / simulation_.frequency));
  publish_decimation_ = static_cast<int>(std::round(combined_frequency / simulation_.publish_rate));
  tick_counter_ = 0;

  combined_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(timer_period),
    std::bind(&MinimalSimulator::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "Minimal Simulator initialized");
  RCLCPP_INFO(this->get_logger(), "  Combined timer frequency: %.1f Hz (update every %d ticks, publish every %d ticks)",
              combined_frequency, update_decimation_, publish_decimation_);
  RCLCPP_INFO(this->get_logger(), "  Vehicle: %.3f x %.3f x %.3f m",
              vehicle_.length, vehicle_.width, vehicle_.height);
  RCLCPP_INFO(this->get_logger(), "  Wheel base: %.3f m, Tread: %.3f m",
              vehicle_.wheel_base, vehicle_.tread);
}

void MinimalSimulator::declareParameters() {
  // Vehicle physical dimensions
  this->declare_parameter("vehicle.length", 0.556);
  this->declare_parameter("vehicle.width", 0.363);
  this->declare_parameter("vehicle.height", 0.2);
  this->declare_parameter("vehicle.wheel_base", 0.380);
  this->declare_parameter("vehicle.tread", 0.360);

  // Dynamics constraints
  this->declare_parameter("dynamics.max_linear_velocity", 1.5);
  this->declare_parameter("dynamics.max_angular_velocity", 3.0);
  this->declare_parameter("dynamics.max_linear_acceleration", 2.0);
  this->declare_parameter("dynamics.max_angular_acceleration", 4.0);

  // Simulation settings
  this->declare_parameter("update_frequency", 100.0);
  this->declare_parameter("publish_rate", 50.0);
  this->declare_parameter("enable_noise", false);
  this->declare_parameter("noise_std_v", 0.01);
  this->declare_parameter("noise_std_omega", 0.02);

  // Initial pose
  this->declare_parameter("initial_pose.x", 0.0);
  this->declare_parameter("initial_pose.y", 0.0);
  this->declare_parameter("initial_pose.yaw", 0.0);

  // Visualization
  this->declare_parameter("visualization.show_trajectory", true);
  this->declare_parameter("visualization.trajectory_length", 100);
}

void MinimalSimulator::loadParameters() {
  // Load vehicle parameters
  vehicle_.length = this->get_parameter("vehicle.length").as_double();
  vehicle_.width = this->get_parameter("vehicle.width").as_double();
  vehicle_.height = this->get_parameter("vehicle.height").as_double();
  vehicle_.wheel_base = this->get_parameter("vehicle.wheel_base").as_double();
  vehicle_.tread = this->get_parameter("vehicle.tread").as_double();

  // Load dynamics parameters
  dynamics_.max_linear_velocity = this->get_parameter("dynamics.max_linear_velocity").as_double();
  dynamics_.max_angular_velocity = this->get_parameter("dynamics.max_angular_velocity").as_double();
  dynamics_.max_linear_acceleration = this->get_parameter("dynamics.max_linear_acceleration").as_double();
  dynamics_.max_angular_acceleration = this->get_parameter("dynamics.max_angular_acceleration").as_double();

  // Load simulation parameters
  simulation_.frequency = this->get_parameter("update_frequency").as_double();
  simulation_.publish_rate = this->get_parameter("publish_rate").as_double();
  simulation_.enable_noise = this->get_parameter("enable_noise").as_bool();
  simulation_.noise_std_v = this->get_parameter("noise_std_v").as_double();
  simulation_.noise_std_omega = this->get_parameter("noise_std_omega").as_double();

  // Load initial pose
  initial_pose_.x = this->get_parameter("initial_pose.x").as_double();
  initial_pose_.y = this->get_parameter("initial_pose.y").as_double();
  initial_pose_.yaw = this->get_parameter("initial_pose.yaw").as_double();

  // Load visualization parameters
  visualization_.show_trajectory = this->get_parameter("visualization.show_trajectory").as_bool();
  visualization_.trajectory_length = this->get_parameter("visualization.trajectory_length").as_int();
}

void MinimalSimulator::validateParameters() {
  // Validate vehicle dimensions
  if (vehicle_.length <= 0.0 || vehicle_.width <= 0.0 || vehicle_.height <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Vehicle dimensions must be positive!");
    throw std::runtime_error("Invalid vehicle dimensions");
  }
  if (vehicle_.wheel_base <= 0.0 || vehicle_.tread <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Wheel parameters must be positive!");
    throw std::runtime_error("Invalid wheel parameters");
  }

  // Validate dynamics constraints (防止负值和零值)
  if (dynamics_.max_linear_velocity <= 0.0 || dynamics_.max_angular_velocity <= 0.0) {
    RCLCPP_ERROR(this->get_logger(),
                 "Max velocities must be positive! (linear=%.3f, angular=%.3f)",
                 dynamics_.max_linear_velocity, dynamics_.max_angular_velocity);
    throw std::runtime_error("Invalid dynamics: max velocities must be positive");
  }
  if (dynamics_.max_linear_acceleration <= 0.0 || dynamics_.max_angular_acceleration <= 0.0) {
    RCLCPP_ERROR(this->get_logger(),
                 "Max accelerations must be positive! (linear=%.3f, angular=%.3f)",
                 dynamics_.max_linear_acceleration, dynamics_.max_angular_acceleration);
    throw std::runtime_error("Invalid dynamics: max accelerations must be positive");
  }

  // Validate simulation frequencies (防止除零)
  if (simulation_.frequency <= 0.0 || simulation_.frequency > 1000.0) {
    RCLCPP_ERROR(this->get_logger(),
                 "Update frequency must be in range (0, 1000] Hz, got %.3f Hz",
                 simulation_.frequency);
    throw std::runtime_error("Invalid update_frequency parameter");
  }
  if (simulation_.publish_rate <= 0.0 || simulation_.publish_rate > 1000.0) {
    RCLCPP_ERROR(this->get_logger(),
                 "Publish rate must be in range (0, 1000] Hz, got %.3f Hz",
                 simulation_.publish_rate);
    throw std::runtime_error("Invalid publish_rate parameter");
  }

  // Validate noise parameters (防止负值)
  if (simulation_.noise_std_v < 0.0 || simulation_.noise_std_omega < 0.0) {
    RCLCPP_WARN(this->get_logger(),
                "Noise standard deviations should be non-negative, using absolute values");
    simulation_.noise_std_v = std::abs(simulation_.noise_std_v);
    simulation_.noise_std_omega = std::abs(simulation_.noise_std_omega);
  }
}

void MinimalSimulator::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  desired_velocity_(0) = std::clamp(msg->linear.x,
                                    -dynamics_.max_linear_velocity,
                                    dynamics_.max_linear_velocity);
  desired_velocity_(1) = std::clamp(msg->angular.z,
                                    -dynamics_.max_angular_velocity,
                                    dynamics_.max_angular_velocity);
}

void MinimalSimulator::timerCallback() {
  tick_counter_++;

  // Calculate timer period and advance simulation time
  double combined_frequency = std::max(simulation_.frequency, simulation_.publish_rate);
  double dt_tick = 1.0 / combined_frequency;
  sim_time_ = sim_time_ + rclcpp::Duration::from_seconds(dt_tick);

  // Publish clock for simulation time synchronization
  rosgraph_msgs::msg::Clock clock_msg;
  clock_msg.clock = sim_time_;
  clock_pub_->publish(clock_msg);

  // Update physics state (with decimation if needed)
  if (tick_counter_ % update_decimation_ == 0) {
    double dt = 1.0 / simulation_.frequency;
    updateState(dt);
  }

  // Publish state (with decimation if needed)
  if (tick_counter_ % publish_decimation_ == 0) {
    // Use simulation time for all messages
    rclcpp::Time current_time = sim_time_;

    // Create state snapshot under mutex protection
    Eigen::Vector3d pose_snapshot;
    Eigen::Vector2d velocity_snapshot;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      pose_snapshot = current_pose_;
      velocity_snapshot = current_velocity_;
    }

    publishOdometry(current_time, pose_snapshot, velocity_snapshot);
    publishVisualization(current_time, pose_snapshot);
    publishTF(current_time, pose_snapshot);
  }
}

void MinimalSimulator::updateState(double dt) {
  std::lock_guard<std::mutex> lock(state_mutex_);

  applyAccelerationLimits(dt);

  if (simulation_.enable_noise) {
    current_velocity_(0) = addNoise(current_velocity_(0), simulation_.noise_std_v);
    current_velocity_(1) = addNoise(current_velocity_(1), simulation_.noise_std_omega);
  }

  propagateState(dt);
}

void MinimalSimulator::applyAccelerationLimits(double dt) {
  Eigen::Vector2d desired_accel;
  desired_accel(0) = (desired_velocity_(0) - current_velocity_(0)) / dt;
  desired_accel(1) = (desired_velocity_(1) - current_velocity_(1)) / dt;

  if (std::abs(desired_accel(0)) > dynamics_.max_linear_acceleration) {
    current_accel_(0) = std::copysign(dynamics_.max_linear_acceleration, desired_accel(0));
  } else {
    current_accel_(0) = desired_accel(0);
  }

  if (std::abs(desired_accel(1)) > dynamics_.max_angular_acceleration) {
    current_accel_(1) = std::copysign(dynamics_.max_angular_acceleration, desired_accel(1));
  } else {
    current_accel_(1) = desired_accel(1);
  }

  current_velocity_(0) += current_accel_(0) * dt;
  current_velocity_(1) += current_accel_(1) * dt;

  current_velocity_(0) = std::clamp(current_velocity_(0),
                                    -dynamics_.max_linear_velocity,
                                    dynamics_.max_linear_velocity);
  current_velocity_(1) = std::clamp(current_velocity_(1),
                                    -dynamics_.max_angular_velocity,
                                    dynamics_.max_angular_velocity);
}

void MinimalSimulator::propagateState(double dt) {
  double v = current_velocity_(0);
  double omega = current_velocity_(1);
  double theta = current_pose_(2);

  current_pose_(0) += v * std::cos(theta) * dt;
  current_pose_(1) += v * std::sin(theta) * dt;
  current_pose_(2) += omega * dt;
  current_pose_(2) = normalizeAngle(current_pose_(2));

  if (visualization_.show_trajectory) {
    trajectory_.push_back(current_pose_);
    if (trajectory_.size() > static_cast<size_t>(visualization_.trajectory_length)) {
      trajectory_.pop_front();  // O(1) operation with deque
    }
  }
}

void MinimalSimulator::publishOdometry(
    const rclcpp::Time& stamp,
    const Eigen::Vector3d& pose,
    const Eigen::Vector2d& velocity) {
  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.stamp = stamp;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";

  odom_msg.pose.pose.position.x = pose(0);
  odom_msg.pose.pose.position.y = pose(1);
  odom_msg.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, pose(2));
  odom_msg.pose.pose.orientation = tf2::toMsg(q);

  odom_msg.twist.twist.linear.x = velocity(0);
  odom_msg.twist.twist.angular.z = velocity(1);

  odom_pub_->publish(odom_msg);
}

void MinimalSimulator::publishVisualization(
    const rclcpp::Time& stamp,
    const Eigen::Vector3d& pose) {
  auto marker_array = visualization_msgs::msg::MarkerArray();

  // Use emplace_back with move semantics to avoid copies
  marker_array.markers.emplace_back(createVehicleMarker(stamp, pose));

  if (visualization_.show_trajectory && !trajectory_.empty()) {
    marker_array.markers.emplace_back(createTrajectoryMarker(stamp));
  }

  vis_pub_->publish(marker_array);
}

void MinimalSimulator::publishTF(
    const rclcpp::Time& stamp,
    const Eigen::Vector3d& pose) {
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_link";

  transform.transform.translation.x = pose(0);
  transform.transform.translation.y = pose(1);
  transform.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, pose(2));
  transform.transform.rotation = tf2::toMsg(q);

  tf_broadcaster_->sendTransform(transform);
}

visualization_msgs::msg::Marker MinimalSimulator::createVehicleMarker(
    const rclcpp::Time& stamp,
    const Eigen::Vector3d& pose) {
  auto marker = visualization_msgs::msg::Marker();
  marker.header.stamp = stamp;
  marker.header.frame_id = "odom";
  marker.ns = "vehicle";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = pose(0);
  marker.pose.position.y = pose(1);
  marker.pose.position.z = vehicle_.height / 2.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, pose(2));
  marker.pose.orientation = tf2::toMsg(q);

  marker.scale.x = vehicle_.length;
  marker.scale.y = vehicle_.width;
  marker.scale.z = vehicle_.height;

  marker.color.r = 0.0;
  marker.color.g = 0.5;
  marker.color.b = 1.0;
  marker.color.a = 0.8;

  marker.lifetime = rclcpp::Duration::from_seconds(0.2);

  return marker;
}

visualization_msgs::msg::Marker MinimalSimulator::createTrajectoryMarker(
    const rclcpp::Time& stamp) {
  auto marker = visualization_msgs::msg::Marker();
  marker.header.stamp = stamp;
  marker.header.frame_id = "odom";
  marker.ns = "trajectory";
  marker.id = 1;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.scale.x = 0.05;

  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 0.8;

  // Pre-allocate points to avoid reallocations
  std::lock_guard<std::mutex> lock(state_mutex_);
  marker.points.reserve(trajectory_.size());

  for (const auto& pose : trajectory_) {
    geometry_msgs::msg::Point p;
    p.x = pose(0);
    p.y = pose(1);
    p.z = 0.05;
    marker.points.push_back(p);
  }

  marker.lifetime = rclcpp::Duration::from_seconds(0.0);

  return marker;
}

double MinimalSimulator::normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double MinimalSimulator::addNoise(double value, double std_dev) {
  return value + noise_dist_(rng_) * std_dev;
}

}  // namespace ddr_minimal_sim

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ddr_minimal_sim::MinimalSimulator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
