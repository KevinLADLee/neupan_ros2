#ifndef DDR_MINIMAL_SIM_ENVIRONMENT_HPP_
#define DDR_MINIMAL_SIM_ENVIRONMENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <vector>
#include <memory>
#include <string>

namespace ddr_minimal_sim {

struct Obstacle {
  enum Type {
    CIRCLE,
    RECTANGLE,
    POLYGON
  };

  Type type;
  double x;           // Center x position
  double y;           // Center y position
  double radius;      // For circle
  double length;      // For rectangle
  double width;       // For rectangle
  double yaw;         // For rectangle rotation
  std::vector<geometry_msgs::msg::Point> points;  // For polygon
};

class EnvironmentNode : public rclcpp::Node {
public:
  EnvironmentNode();
  ~EnvironmentNode() = default;

  // Query if a point is in collision
  bool isCollision(double x, double y) const;

  // Get all obstacles
  const std::vector<Obstacle>& getObstacles() const { return obstacles_; }

private:
  // Obstacle generation mode
  enum class ObstacleGenerationMode {
    STATIC,    // Hard-coded default obstacles
    FILE,      // Load from YAML file
    SCENARIO   // Use predefined test scenario
  };

  // Parameters
  struct MapParams {
    double width;        // Map width (m)
    double height;       // Map height (m)
    double resolution;   // Grid resolution (m)
    ObstacleGenerationMode generation_mode;  // How to generate obstacles
    std::string scenario_name;               // Scenario name (if mode == SCENARIO)
    std::string map_file;                    // Map file path (if mode == FILE)
  } map_params_;

  struct VisualizationParams {
    double obstacle_height;  // Obstacle height for visualization (m)
    double marker_alpha;     // Obstacle transparency (0-1)
  } visualization_;

  // Obstacles
  std::vector<Obstacle> obstacles_;

  // Cached occupancy grid (built once at startup)
  nav_msgs::msg::OccupancyGrid cached_grid_;

  // ROS2 Communication
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Member Functions
  void declareParameters();
  void loadParameters();
  void loadObstacles();
  void buildOccupancyGrid();  // Build and cache the occupancy grid
  void publishCallback();
  void publishMarkers();
  void publishOccupancyGrid();

  // Collision checking helpers
  bool isPointInCircle(double px, double py, const Obstacle& obs) const;
  bool isPointInRectangle(double px, double py, const Obstacle& obs) const;
  bool isPointInPolygon(double px, double py, const Obstacle& obs) const;

  // Visualization helpers
  visualization_msgs::msg::Marker createObstacleMarker(const Obstacle& obs, int id);

  // DDR-opt map loading
  void loadFromYAML(const std::string& map_file);
};

}  // namespace ddr_minimal_sim

#endif  // DDR_MINIMAL_SIM_ENVIRONMENT_HPP_
