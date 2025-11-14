#include "ddr_minimal_sim/laser_simulator.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <random>
#include <chrono>

namespace ddr_minimal_sim {

LaserSimulator::LaserSimulator()
  : Node("laser_simulator"),
    map_received_(false),
    rng_(std::random_device{}()),
    noise_dist_(0.0, 1.0)
{
  declareParameters();
  loadParameters();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/environment_grid", 10,
    std::bind(&LaserSimulator::mapCallback, this, std::placeholders::_1));

  scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

  scan_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(laser_params_.scan_time),
    std::bind(&LaserSimulator::scanCallback, this));

  RCLCPP_INFO(this->get_logger(), "Laser Simulator initialized");
  RCLCPP_INFO(this->get_logger(), "  Angle range: [%.2f, %.2f] rad",
              laser_params_.angle_min, laser_params_.angle_max);
  RCLCPP_INFO(this->get_logger(), "  Range: [%.2f, %.2f] m",
              laser_params_.range_min, laser_params_.range_max);
  RCLCPP_INFO(this->get_logger(), "  Angular resolution: %.4f rad (%.2f deg)",
              laser_params_.angle_increment,
              laser_params_.angle_increment * 180.0 / M_PI);
}

void LaserSimulator::declareParameters() {
  this->declare_parameter("laser.angle_min", -M_PI);
  this->declare_parameter("laser.angle_max", M_PI);
  this->declare_parameter("laser.angle_increment", M_PI / 180.0);  // 1 degree
  this->declare_parameter("laser.range_min", 0.1);
  this->declare_parameter("laser.range_max", 10.0);
  this->declare_parameter("laser.scan_time", 0.1);  // 10 Hz
  this->declare_parameter("laser.frame_id", std::string("base_link"));

  this->declare_parameter("noise.enable_noise", false);
  this->declare_parameter("noise.noise_std_dev", 0.01);
}

void LaserSimulator::loadParameters() {
  laser_params_.angle_min = this->get_parameter("laser.angle_min").as_double();
  laser_params_.angle_max = this->get_parameter("laser.angle_max").as_double();
  laser_params_.angle_increment = this->get_parameter("laser.angle_increment").as_double();
  laser_params_.range_min = this->get_parameter("laser.range_min").as_double();
  laser_params_.range_max = this->get_parameter("laser.range_max").as_double();
  laser_params_.scan_time = this->get_parameter("laser.scan_time").as_double();
  laser_params_.frame_id = this->get_parameter("laser.frame_id").as_string();

  noise_params_.enable_noise = this->get_parameter("noise.enable_noise").as_bool();
  noise_params_.noise_std_dev = this->get_parameter("noise.noise_std_dev").as_double();

  // Validate laser parameters (防止除零和无效配置)
  if (laser_params_.angle_increment <= 0.0) {
    RCLCPP_ERROR(this->get_logger(),
                 "Laser angle_increment must be positive, got %.6f rad",
                 laser_params_.angle_increment);
    throw std::runtime_error("Invalid laser.angle_increment parameter");
  }
  if (laser_params_.angle_max <= laser_params_.angle_min) {
    RCLCPP_ERROR(this->get_logger(),
                 "Laser angle_max (%.3f) must be greater than angle_min (%.3f)",
                 laser_params_.angle_max, laser_params_.angle_min);
    throw std::runtime_error("Invalid laser angle range");
  }
  if (laser_params_.range_min < 0.0 || laser_params_.range_max <= laser_params_.range_min) {
    RCLCPP_ERROR(this->get_logger(),
                 "Laser range invalid: range_max (%.3f) must be > range_min (%.3f) >= 0",
                 laser_params_.range_max, laser_params_.range_min);
    throw std::runtime_error("Invalid laser range parameters");
  }
  if (laser_params_.scan_time <= 0.0) {
    RCLCPP_ERROR(this->get_logger(),
                 "Laser scan_time must be positive, got %.6f s",
                 laser_params_.scan_time);
    throw std::runtime_error("Invalid laser.scan_time parameter");
  }

  // Validate noise parameters
  if (noise_params_.noise_std_dev < 0.0) {
    RCLCPP_WARN(this->get_logger(),
                "Noise std_dev should be non-negative, using absolute value");
    noise_params_.noise_std_dev = std::abs(noise_params_.noise_std_dev);
  }

  // Check if number of laser rays is reasonable
  int num_rays = static_cast<int>(
    (laser_params_.angle_max - laser_params_.angle_min) / laser_params_.angle_increment) + 1;
  if (num_rays > 10000) {
    RCLCPP_WARN(this->get_logger(),
                "Large number of laser rays (%d) may cause performance issues",
                num_rays);
  }
}

void LaserSimulator::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  environment_map_ = msg;
  if (!map_received_) {
    map_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Received environment map: %dx%d @ %.3f m/cell",
                msg->info.width, msg->info.height, msg->info.resolution);
  }
}

void LaserSimulator::scanCallback() {
  auto start_time = std::chrono::high_resolution_clock::now();

  if (!map_received_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Waiting for environment map...");
    return;
  }

  // Get current time for consistent timestamps
  rclcpp::Time scan_time = this->now();

  // Get robot pose in map frame using scan time
  // Use a small timeout to wait for TF if needed
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      "odom", laser_params_.frame_id, scan_time,
      rclcpp::Duration::from_seconds(0.1));  // 100ms timeout
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Could not transform at time %.3f: %s",
                         scan_time.seconds(), ex.what());
    return;
  }

  double robot_x = transform.transform.translation.x;
  double robot_y = transform.transform.translation.y;

  // Extract yaw from quaternion
  tf2::Quaternion q(
    transform.transform.rotation.x,
    transform.transform.rotation.y,
    transform.transform.rotation.z,
    transform.transform.rotation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Create laser scan message with consistent timestamp
  auto scan = sensor_msgs::msg::LaserScan();
  scan.header.stamp = scan_time;
  scan.header.frame_id = laser_params_.frame_id;

  scan.angle_min = laser_params_.angle_min;
  scan.angle_max = laser_params_.angle_max;
  scan.angle_increment = laser_params_.angle_increment;
  scan.time_increment = 0.0;
  scan.scan_time = laser_params_.scan_time;
  scan.range_min = laser_params_.range_min;
  scan.range_max = laser_params_.range_max;

  int num_readings = static_cast<int>(
    (laser_params_.angle_max - laser_params_.angle_min) / laser_params_.angle_increment) + 1;
  scan.ranges.resize(num_readings);
  scan.intensities.resize(num_readings);

  // Perform ray casting for each angle
  for (int i = 0; i < num_readings; ++i) {
    double angle = laser_params_.angle_min + i * laser_params_.angle_increment;
    double global_angle = yaw + angle;

    double range = rayCast(robot_x, robot_y, global_angle);

    if (noise_params_.enable_noise && range < laser_params_.range_max) {
      range = addNoise(range);
    }

    scan.ranges[i] = range;
    scan.intensities[i] = (range < laser_params_.range_max) ? 1.0 : 0.0;
  }

  scan_pub_->publish(scan);

  // Performance measurement
  auto end_time = std::chrono::high_resolution_clock::now();
  double elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

  perf_stats_.scan_count++;
  perf_stats_.total_time_ms += elapsed_ms;
  perf_stats_.max_time_ms = std::max(perf_stats_.max_time_ms, elapsed_ms);
  perf_stats_.min_time_ms = std::min(perf_stats_.min_time_ms, elapsed_ms);

  // Log performance stats every 100 scans
  // if (perf_stats_.scan_count % 100 == 0) {
    // double avg_ms = perf_stats_.total_time_ms / perf_stats_.scan_count;
  //   RCLCPP_INFO(this->get_logger(),
  //               "[Laser Perf] Scans: %zu | Avg: %.2f ms | Min: %.2f ms | Max: %.2f ms | "
  //               "Rays/scan: %d",
  //               perf_stats_.scan_count, avg_ms, perf_stats_.min_time_ms,
  //               perf_stats_.max_time_ms, num_readings);
  // }
}

double LaserSimulator::rayCast(double x, double y, double angle) {
  // Calculate end point of ray
  double dx = laser_params_.range_max * std::cos(angle);
  double dy = laser_params_.range_max * std::sin(angle);
  double end_x = x + dx;
  double end_y = y + dy;

  // Convert start point to grid coordinates
  int x0 = static_cast<int>((x - environment_map_->info.origin.position.x) /
                            environment_map_->info.resolution);
  int y0 = static_cast<int>((y - environment_map_->info.origin.position.y) /
                            environment_map_->info.resolution);

  // Convert end point to grid coordinates
  int x1 = static_cast<int>((end_x - environment_map_->info.origin.position.x) /
                            environment_map_->info.resolution);
  int y1 = static_cast<int>((end_y - environment_map_->info.origin.position.y) /
                            environment_map_->info.resolution);

  // Bresenham's line algorithm for efficient grid traversal
  int dx_grid = std::abs(x1 - x0);
  int dy_grid = std::abs(y1 - y0);
  int x_step = (x0 < x1) ? 1 : -1;
  int y_step = (y0 < y1) ? 1 : -1;
  int err = dx_grid - dy_grid;

  int grid_x = x0;
  int grid_y = y0;

  while (true) {
    // Check if current grid cell is occupied
    if (isOccupied(grid_x, grid_y)) {
      // Calculate actual distance to hit point
      double hit_x = environment_map_->info.origin.position.x +
                     (grid_x + 0.5) * environment_map_->info.resolution;
      double hit_y = environment_map_->info.origin.position.y +
                     (grid_y + 0.5) * environment_map_->info.resolution;
      double dx_hit = hit_x - x;
      double dy_hit = hit_y - y;
      double distance = std::sqrt(dx_hit * dx_hit + dy_hit * dy_hit);

      // Clamp to range
      return std::min(distance, laser_params_.range_max);
    }

    // Reached end point
    if (grid_x == x1 && grid_y == y1) {
      break;
    }

    // Bresenham step
    int err2 = 2 * err;
    if (err2 > -dy_grid) {
      err -= dy_grid;
      grid_x += x_step;
    }
    if (err2 < dx_grid) {
      err += dx_grid;
      grid_y += y_step;
    }
  }

  return laser_params_.range_max;
}

bool LaserSimulator::isOccupied(int grid_x, int grid_y) const {
  if (grid_x < 0 || grid_x >= static_cast<int>(environment_map_->info.width) ||
      grid_y < 0 || grid_y >= static_cast<int>(environment_map_->info.height)) {
    return true;  // Out of bounds = occupied
  }

  int index = grid_y * environment_map_->info.width + grid_x;
  return environment_map_->data[index] > 50;  // Occupied if > 50%
}

double LaserSimulator::addNoise(double value) {
  return value + noise_dist_(rng_) * noise_params_.noise_std_dev;
}

}  // namespace ddr_minimal_sim

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ddr_minimal_sim::LaserSimulator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
