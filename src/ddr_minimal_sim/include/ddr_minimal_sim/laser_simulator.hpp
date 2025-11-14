#ifndef DDR_MINIMAL_SIM_LASER_SIMULATOR_HPP_
#define DDR_MINIMAL_SIM_LASER_SIMULATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <memory>
#include <vector>
#include <random>
#include <limits>

namespace ddr_minimal_sim {

class LaserSimulator : public rclcpp::Node {
public:
  LaserSimulator();
  ~LaserSimulator() = default;

private:
  // Parameters - Laser Specifications
  struct LaserParams {
    double angle_min;        // Start angle (rad)
    double angle_max;        // End angle (rad)
    double angle_increment;  // Angular resolution (rad)
    double range_min;        // Minimum range (m)
    double range_max;        // Maximum range (m)
    double scan_time;        // Time between scans (s)
    std::string frame_id;    // Frame ID for laser
  } laser_params_;

  // Parameters - Noise
  struct NoiseParams {
    bool enable_noise;
    double noise_std_dev;    // Range noise standard deviation (m)
  } noise_params_;

  // Environment map
  nav_msgs::msg::OccupancyGrid::SharedPtr environment_map_;
  bool map_received_;

  // ROS2 Communication
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::TimerBase::SharedPtr scan_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Random number generation for noise
  std::mt19937 rng_;
  std::normal_distribution<double> noise_dist_;

  // Performance diagnostics
  struct {
    size_t scan_count = 0;
    double total_time_ms = 0.0;
    double max_time_ms = 0.0;
    double min_time_ms = std::numeric_limits<double>::max();
  } perf_stats_;

  // Member Functions
  void declareParameters();
  void loadParameters();
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void scanCallback();

  // Ray tracing
  double rayCast(double x, double y, double angle);
  bool isOccupied(int grid_x, int grid_y) const;
  double addNoise(double value);
};

}  // namespace ddr_minimal_sim

#endif  // DDR_MINIMAL_SIM_LASER_SIMULATOR_HPP_
