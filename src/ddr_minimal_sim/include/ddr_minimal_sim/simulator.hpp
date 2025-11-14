#ifndef DDR_MINIMAL_SIM_SIMULATOR_HPP_
#define DDR_MINIMAL_SIM_SIMULATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <random>
#include <mutex>
#include <chrono>

namespace ddr_minimal_sim {

class MinimalSimulator : public rclcpp::Node {
public:
  MinimalSimulator();
  ~MinimalSimulator() = default;

private:
  // Parameters - Vehicle Physical Properties
  struct VehicleParams {
    double length;          // Vehicle length (m)
    double width;           // Vehicle width (m)
    double height;          // Vehicle height (m)
    double wheel_base;      // Distance between front and rear axles (m)
    double tread;           // Distance between left and right wheels (m)
  } vehicle_;

  // Parameters - Dynamics Constraints
  struct DynamicsParams {
    double max_linear_velocity;       // Max linear velocity (m/s)
    double max_angular_velocity;      // Max angular velocity (rad/s)
    double max_linear_acceleration;   // Max linear acceleration (m/s^2)
    double max_angular_acceleration;  // Max angular acceleration (rad/s^2)
  } dynamics_;

  // Parameters - Simulation Settings
  struct SimulationParams {
    double frequency;          // Physics update frequency (Hz)
    double publish_rate;       // State publish rate (Hz)
    bool enable_noise;         // Enable velocity noise
    double noise_std_v;        // Linear velocity noise std dev
    double noise_std_omega;    // Angular velocity noise std dev
  } simulation_;

  // Parameters - Initial State
  struct InitialPose {
    double x;
    double y;
    double yaw;
  } initial_pose_;

  // Parameters - Visualization
  struct VisualizationParams {
    bool show_trajectory;
    int trajectory_length;
  } visualization_;

  // State Variables
  Eigen::Vector3d current_pose_;       // [x, y, yaw]
  Eigen::Vector2d current_velocity_;   // [v, omega]
  Eigen::Vector2d desired_velocity_;   // [v_desired, omega_desired]
  Eigen::Vector2d current_accel_;      // [a, alpha]

  // Trajectory history for visualization
  std::deque<Eigen::Vector3d> trajectory_;  // Changed to deque for O(1) pop_front()

  // ROS2 Communication
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_pub_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;

  rclcpp::TimerBase::SharedPtr combined_timer_;
  int tick_counter_;
  int update_decimation_;
  int publish_decimation_;

  // Simulation time management
  rclcpp::Time sim_time_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Random number generator for noise
  std::mt19937 rng_;
  std::normal_distribution<double> noise_dist_;

  // Mutex for thread safety
  mutable std::mutex state_mutex_;

  // Member Functions - Parameter Management
  void declareParameters();
  void loadParameters();
  void validateParameters();

  // Member Functions - Control Callbacks
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // Member Functions - Physics and State
  void timerCallback();  // Combined update and publish
  void updateState(double dt);
  void propagateState(double dt);
  void applyAccelerationLimits(double dt);

  // Member Functions - Publishing
  void publishOdometry(const rclcpp::Time& stamp,
                       const Eigen::Vector3d& pose,
                       const Eigen::Vector2d& velocity);
  void publishVisualization(const rclcpp::Time& stamp,
                            const Eigen::Vector3d& pose);
  void publishTF(const rclcpp::Time& stamp,
                 const Eigen::Vector3d& pose);

  // Member Functions - Visualization
  visualization_msgs::msg::Marker createVehicleMarker(const rclcpp::Time& stamp,
                                                       const Eigen::Vector3d& pose);
  visualization_msgs::msg::Marker createTrajectoryMarker(const rclcpp::Time& stamp);

  // Utility functions
  double normalizeAngle(double angle);
  double addNoise(double value, double std_dev);
};

}  // namespace ddr_minimal_sim

#endif  // DDR_MINIMAL_SIM_SIMULATOR_HPP_
