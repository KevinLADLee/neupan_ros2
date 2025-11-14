#include "ddr_minimal_sim/environment.hpp"
#include "ddr_minimal_sim/scenario_library.hpp"
#include <cmath>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace ddr_minimal_sim {

EnvironmentNode::EnvironmentNode()
  : Node("environment_node")
{
  declareParameters();
  loadParameters();
  loadObstacles();

  // Build occupancy grid once (static obstacles)
  buildOccupancyGrid();

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/environment_markers", rclcpp::QoS(10).transient_local());
  grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/environment_grid", 10);

  // Publish static markers once at startup (obstacles don't change)
  publishMarkers();
  RCLCPP_INFO(this->get_logger(), "Published %zu static obstacle markers", obstacles_.size());

  // Timer only publishes dynamic data (occupancy grid)
  publish_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&EnvironmentNode::publishOccupancyGrid, this));

  RCLCPP_INFO(this->get_logger(), "Environment node initialized with %zu obstacles",
              obstacles_.size());
  RCLCPP_INFO(this->get_logger(), "Map size: %.2f x %.2f m, resolution: %.3f m",
              map_params_.width, map_params_.height, map_params_.resolution);
}

void EnvironmentNode::declareParameters() {
  this->declare_parameter("map.width", 20.0);
  this->declare_parameter("map.height", 20.0);
  this->declare_parameter("map.resolution", 0.05);
  this->declare_parameter("map.generation_mode", std::string("static"));  // Options: static, file, scenario
  this->declare_parameter("map.scenario", std::string("empty"));          // Scenario name if mode=scenario
  this->declare_parameter("map.map_file", std::string(""));               // Map file if mode=file
  this->declare_parameter("visualization.obstacle_height", 2.0);
  this->declare_parameter("visualization.marker_alpha", 0.8);
}

void EnvironmentNode::loadParameters() {
  map_params_.width = this->get_parameter("map.width").as_double();
  map_params_.height = this->get_parameter("map.height").as_double();
  map_params_.resolution = this->get_parameter("map.resolution").as_double();

  // Load generation mode and related parameters
  std::string mode_str = this->get_parameter("map.generation_mode").as_string();
  if (mode_str == "static") {
    map_params_.generation_mode = ObstacleGenerationMode::STATIC;
  } else if (mode_str == "file") {
    map_params_.generation_mode = ObstacleGenerationMode::FILE;
  } else if (mode_str == "scenario") {
    map_params_.generation_mode = ObstacleGenerationMode::SCENARIO;
  } else {
    RCLCPP_WARN(this->get_logger(),
                "Unknown generation_mode '%s', defaulting to 'static'",
                mode_str.c_str());
    map_params_.generation_mode = ObstacleGenerationMode::STATIC;
  }

  map_params_.scenario_name = this->get_parameter("map.scenario").as_string();
  map_params_.map_file = this->get_parameter("map.map_file").as_string();

  visualization_.obstacle_height = this->get_parameter("visualization.obstacle_height").as_double();
  visualization_.marker_alpha = this->get_parameter("visualization.marker_alpha").as_double();

  // Validate map parameters (防止除零崩溃)
  if (map_params_.resolution <= 0.0) {
    RCLCPP_ERROR(this->get_logger(),
                 "Map resolution must be positive, got %.6f m",
                 map_params_.resolution);
    throw std::runtime_error("Invalid map.resolution parameter");
  }
  if (map_params_.width <= 0.0 || map_params_.height <= 0.0) {
    RCLCPP_ERROR(this->get_logger(),
                 "Map dimensions must be positive! (width=%.3f, height=%.3f)",
                 map_params_.width, map_params_.height);
    throw std::runtime_error("Invalid map dimensions");
  }
  if (map_params_.resolution > map_params_.width || map_params_.resolution > map_params_.height) {
    RCLCPP_WARN(this->get_logger(),
                "Map resolution (%.3f m) is larger than map dimensions (%.3fx%.3f m)",
                map_params_.resolution, map_params_.width, map_params_.height);
  }
}

void EnvironmentNode::loadObstacles() {
  switch (map_params_.generation_mode) {
    case ObstacleGenerationMode::FILE:
      // Load obstacles from YAML file
      if (map_params_.map_file.empty()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Generation mode is 'file' but map.map_file parameter is empty!");
        throw std::runtime_error("Missing map.map_file parameter");
      }
      loadFromYAML(map_params_.map_file);
      RCLCPP_INFO(this->get_logger(),
                  "Loaded obstacles from file: %s",
                  map_params_.map_file.c_str());
      break;

    case ObstacleGenerationMode::SCENARIO:
      // Generate obstacles from predefined scenario
      try {
        TestScenario scenario = stringToScenario(map_params_.scenario_name);
        obstacles_ = generateScenario(scenario, map_params_.width, map_params_.height);
        RCLCPP_INFO(this->get_logger(),
                    "Generated scenario '%s' with %zu obstacles: %s",
                    map_params_.scenario_name.c_str(),
                    obstacles_.size(),
                    getScenarioDescription(scenario).c_str());

        // Log recommended waypoints for this scenario
        ScenarioWaypoints waypoints = getRecommendedWaypoints(scenario);
        RCLCPP_INFO(this->get_logger(),
                    "Recommended waypoints: start=(%.1f, %.1f), goal=(%.1f, %.1f)",
                    waypoints.start_x, waypoints.start_y,
                    waypoints.goal_x, waypoints.goal_y);
      } catch (const std::invalid_argument& e) {
        RCLCPP_ERROR(this->get_logger(),
                     "Invalid scenario name '%s': %s",
                     map_params_.scenario_name.c_str(), e.what());
        throw;
      }
      break;

    case ObstacleGenerationMode::STATIC:
    default:
      // Default: Add boundary walls and some obstacles
      RCLCPP_INFO(this->get_logger(), "Using static obstacle configuration");
      Obstacle wall;
      wall.type = Obstacle::RECTANGLE;

      // Bottom wall
      wall.x = 0.0; wall.y = -10.0;
      wall.length = 20.0; wall.width = 0.2; wall.yaw = 0.0;
      obstacles_.push_back(wall);

      // Top wall
      wall.x = 0.0; wall.y = 10.0;
      wall.length = 20.0; wall.width = 0.2; wall.yaw = 0.0;
      obstacles_.push_back(wall);

      // Left wall
      wall.x = -10.0; wall.y = 0.0;
      wall.length = 0.2; wall.width = 20.0; wall.yaw = 0.0;
      obstacles_.push_back(wall);

      // Right wall
      wall.x = 10.0; wall.y = 0.0;
      wall.length = 0.2; wall.width = 20.0; wall.yaw = 0.0;
      obstacles_.push_back(wall);

      // Add some circular obstacles
      Obstacle circle;
      circle.type = Obstacle::CIRCLE;

      circle.x = 3.0; circle.y = 3.0; circle.radius = 0.8;
      obstacles_.push_back(circle);

      circle.x = -4.0; circle.y = 2.0; circle.radius = 1.0;
      obstacles_.push_back(circle);

      circle.x = 2.0; circle.y = -3.0; circle.radius = 0.6;
      obstacles_.push_back(circle);

      // Add some rectangular obstacles
      Obstacle rect;
      rect.type = Obstacle::RECTANGLE;

      rect.x = -2.0; rect.y = -2.0;
      rect.length = 2.0; rect.width = 1.0; rect.yaw = 0.0;
      obstacles_.push_back(rect);

      rect.x = 5.0; rect.y = -4.0;
      rect.length = 1.5; rect.width = 0.8; rect.yaw = M_PI / 4;
      obstacles_.push_back(rect);
      break;
  }
}

void EnvironmentNode::publishMarkers() {
  auto marker_array = visualization_msgs::msg::MarkerArray();

  // Pre-allocate space for obstacle markers to avoid reallocations
  marker_array.markers.reserve(obstacles_.size());

  // Add all obstacle markers
  for (size_t i = 0; i < obstacles_.size(); ++i) {
    marker_array.markers.emplace_back(createObstacleMarker(obstacles_[i], i));
  }

  marker_pub_->publish(marker_array);
}

void EnvironmentNode::buildOccupancyGrid() {
  RCLCPP_INFO(this->get_logger(), "Building occupancy grid...");

  cached_grid_.header.frame_id = "odom";
  cached_grid_.info.resolution = map_params_.resolution;
  cached_grid_.info.width = static_cast<int>(map_params_.width / map_params_.resolution);
  cached_grid_.info.height = static_cast<int>(map_params_.height / map_params_.resolution);
  cached_grid_.info.origin.position.x = -map_params_.width / 2.0;
  cached_grid_.info.origin.position.y = -map_params_.height / 2.0;
  cached_grid_.info.origin.position.z = 0.0;
  cached_grid_.info.origin.orientation.w = 1.0;

  cached_grid_.data.resize(cached_grid_.info.width * cached_grid_.info.height, 0);

  // Fill occupancy grid - this is expensive but only done once
  int occupied_cells = 0;
  for (unsigned int i = 0; i < cached_grid_.info.height; ++i) {
    for (unsigned int j = 0; j < cached_grid_.info.width; ++j) {
      double x = cached_grid_.info.origin.position.x + (j + 0.5) * cached_grid_.info.resolution;
      double y = cached_grid_.info.origin.position.y + (i + 0.5) * cached_grid_.info.resolution;

      if (isCollision(x, y)) {
        cached_grid_.data[i * cached_grid_.info.width + j] = 100;  // Occupied
        occupied_cells++;
      }
    }
  }

  RCLCPP_INFO(this->get_logger(), "Occupancy grid built: %ux%u cells, %d occupied (%.1f%%)",
              cached_grid_.info.width, cached_grid_.info.height, occupied_cells,
              100.0 * occupied_cells / (cached_grid_.info.width * cached_grid_.info.height));
}

void EnvironmentNode::publishOccupancyGrid() {
  // Simply publish the cached grid with updated timestamp
  cached_grid_.header.stamp = this->now();
  grid_pub_->publish(cached_grid_);
}

bool EnvironmentNode::isCollision(double x, double y) const {
  for (const auto& obs : obstacles_) {
    switch (obs.type) {
      case Obstacle::CIRCLE:
        if (isPointInCircle(x, y, obs)) return true;
        break;
      case Obstacle::RECTANGLE:
        if (isPointInRectangle(x, y, obs)) return true;
        break;
      case Obstacle::POLYGON:
        if (isPointInPolygon(x, y, obs)) return true;
        break;
    }
  }
  return false;
}

bool EnvironmentNode::isPointInCircle(double px, double py, const Obstacle& obs) const {
  double dx = px - obs.x;
  double dy = py - obs.y;
  return (dx * dx + dy * dy) <= (obs.radius * obs.radius);
}

bool EnvironmentNode::isPointInRectangle(double px, double py, const Obstacle& obs) const {
  // Transform point to obstacle's local frame
  double dx = px - obs.x;
  double dy = py - obs.y;

  double cos_yaw = std::cos(-obs.yaw);
  double sin_yaw = std::sin(-obs.yaw);

  double local_x = dx * cos_yaw - dy * sin_yaw;
  double local_y = dx * sin_yaw + dy * cos_yaw;

  return (std::abs(local_x) <= obs.length / 2.0) &&
         (std::abs(local_y) <= obs.width / 2.0);
}

bool EnvironmentNode::isPointInPolygon(double px, double py, const Obstacle& obs) const {
  if (obs.points.size() < 3) return false;

  bool inside = false;
  for (size_t i = 0, j = obs.points.size() - 1; i < obs.points.size(); j = i++) {
    double xi = obs.points[i].x, yi = obs.points[i].y;
    double xj = obs.points[j].x, yj = obs.points[j].y;

    bool intersect = ((yi > py) != (yj > py)) &&
                     (px < (xj - xi) * (py - yi) / (yj - yi) + xi);
    if (intersect) inside = !inside;
  }
  return inside;
}

visualization_msgs::msg::Marker EnvironmentNode::createObstacleMarker(
    const Obstacle& obs, int id) {
  auto marker = visualization_msgs::msg::Marker();
  marker.header.stamp = this->now();
  marker.header.frame_id = "odom";
  marker.ns = "obstacles";
  marker.id = id;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime = rclcpp::Duration::from_seconds(0);

  marker.color.r = 0.8;
  marker.color.g = 0.2;
  marker.color.b = 0.2;
  marker.color.a = visualization_.marker_alpha;

  switch (obs.type) {
    case Obstacle::CIRCLE:
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.pose.position.x = obs.x;
      marker.pose.position.y = obs.y;
      marker.pose.position.z = visualization_.obstacle_height / 2.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = obs.radius * 2.0;
      marker.scale.y = obs.radius * 2.0;
      marker.scale.z = visualization_.obstacle_height;
      break;

    case Obstacle::RECTANGLE:
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.pose.position.x = obs.x;
      marker.pose.position.y = obs.y;
      marker.pose.position.z = visualization_.obstacle_height / 2.0;
      marker.pose.orientation.z = std::sin(obs.yaw / 2.0);
      marker.pose.orientation.w = std::cos(obs.yaw / 2.0);
      marker.scale.x = obs.length;
      marker.scale.y = obs.width;
      marker.scale.z = visualization_.obstacle_height;
      break;

    case Obstacle::POLYGON:
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.1;
      for (const auto& p : obs.points) {
        marker.points.push_back(p);
      }
      if (!obs.points.empty()) {
        marker.points.push_back(obs.points[0]);  // Close the polygon
      }
      break;
  }

  return marker;
}

void EnvironmentNode::loadFromYAML(const std::string& map_file) {
  RCLCPP_INFO(this->get_logger(), "Loading map from: %s", map_file.c_str());

  // Resolve file path
  std::string full_path = map_file;
  if (map_file[0] != '/') {
    try {
      std::string package_dir = ament_index_cpp::get_package_share_directory("ddr_minimal_sim");
      full_path = package_dir + "/config/maps/" + map_file;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to find package directory: %s", e.what());
      return;
    }
  }

  // Load YAML file
  YAML::Node config;
  try {
    config = YAML::LoadFile(full_path);
  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file: %s", e.what());
    return;
  }

  // Parse world section (optional)
  if (config["world"]) {
    if (config["world"]["width"]) {
      map_params_.width = config["world"]["width"].as<double>();
    }
    if (config["world"]["height"]) {
      map_params_.height = config["world"]["height"].as<double>();
    }
    RCLCPP_INFO(this->get_logger(), "Map size from YAML: %.2f x %.2f m",
                map_params_.width, map_params_.height);
  }

  // Parse obstacles section
  if (!config["obstacles"]) {
    RCLCPP_WARN(this->get_logger(), "No obstacles section found in YAML");
    return;
  }

  int obstacle_count = 0;
  for (const auto& obs_node : config["obstacles"]) {
    Obstacle obs;

    // Parse shape
    if (!obs_node["shape"]) {
      RCLCPP_WARN(this->get_logger(), "Obstacle missing 'shape' field, skipping");
      continue;
    }

    auto shape = obs_node["shape"];
    std::string type = shape["type"].as<std::string>();

    if (type == "rectangle") {
      obs.type = Obstacle::RECTANGLE;
      obs.length = shape["length"].as<double>();
      obs.width = shape["width"].as<double>();
      obs.radius = 0.0;
    }
    else if (type == "circle" || type == "cylinder") {
      obs.type = Obstacle::CIRCLE;
      obs.radius = shape["radius"].as<double>();
      obs.length = 0.0;
      obs.width = 0.0;
    }
    else if (type == "polygon") {
      obs.type = Obstacle::POLYGON;
      obs.radius = 0.0;
      obs.length = 0.0;
      obs.width = 0.0;

      // Parse points array
      if (shape["points"]) {
        for (const auto& pt : shape["points"]) {
          geometry_msgs::msg::Point p;
          p.x = pt[0].as<double>();
          p.y = pt[1].as<double>();
          p.z = 0.0;
          obs.points.push_back(p);
        }
      }
    }
    else {
      RCLCPP_WARN(this->get_logger(), "Unknown obstacle type: %s", type.c_str());
      continue;
    }

    // Parse pose
    if (!obs_node["pose"]) {
      RCLCPP_WARN(this->get_logger(), "Obstacle missing 'pose' field, skipping");
      continue;
    }

    auto pose = obs_node["pose"];
    obs.x = pose["x"].as<double>();
    obs.y = pose["y"].as<double>();
    obs.yaw = pose["yaw"] ? pose["yaw"].as<double>() : 0.0;

    obstacles_.push_back(obs);
    obstacle_count++;
  }

  RCLCPP_INFO(this->get_logger(), "Loaded %d obstacles from YAML file", obstacle_count);
}

}  // namespace ddr_minimal_sim

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ddr_minimal_sim::EnvironmentNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
