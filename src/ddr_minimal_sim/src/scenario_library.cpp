#include "ddr_minimal_sim/scenario_library.hpp"
#include <stdexcept>
#include <cmath>

namespace ddr_minimal_sim {

// ===== Enum Conversion Functions =====

std::string scenarioToString(TestScenario scenario) {
  switch (scenario) {
    case TestScenario::EMPTY: return "empty";
    case TestScenario::SPARSE_5: return "sparse_5";
    case TestScenario::SPARSE_10: return "sparse_10";
    case TestScenario::CORRIDOR: return "corridor";
    case TestScenario::U_TRAP: return "u_trap";
    case TestScenario::NARROW_PASSAGE: return "narrow_passage";
    case TestScenario::MAZE_SIMPLE: return "maze_simple";
    default: return "unknown";
  }
}

TestScenario stringToScenario(const std::string& name) {
  if (name == "empty") return TestScenario::EMPTY;
  if (name == "sparse_5") return TestScenario::SPARSE_5;
  if (name == "sparse_10") return TestScenario::SPARSE_10;
  if (name == "corridor") return TestScenario::CORRIDOR;
  if (name == "u_trap") return TestScenario::U_TRAP;
  if (name == "narrow_passage") return TestScenario::NARROW_PASSAGE;
  if (name == "maze_simple") return TestScenario::MAZE_SIMPLE;
  throw std::invalid_argument("Unknown scenario name: " + name);
}

// ===== Main Generation Function =====

std::vector<Obstacle> generateScenario(
  TestScenario scenario,
  double map_width,
  double map_height
) {
  switch (scenario) {
    case TestScenario::EMPTY:
      return scenarios::generateEmpty(map_width, map_height);
    case TestScenario::SPARSE_5:
      return scenarios::generateSparse5(map_width, map_height);
    case TestScenario::SPARSE_10:
      return scenarios::generateSparse10(map_width, map_height);
    case TestScenario::CORRIDOR:
      return scenarios::generateCorridor(map_width, map_height);
    case TestScenario::U_TRAP:
      return scenarios::generateUTrap(map_width, map_height);
    case TestScenario::NARROW_PASSAGE:
      return scenarios::generateNarrowPassage(map_width, map_height);
    case TestScenario::MAZE_SIMPLE:
      return scenarios::generateMazeSimple(map_width, map_height);
    default:
      throw std::runtime_error("Unhandled scenario type");
  }
}

// ===== Description Function =====

std::string getScenarioDescription(TestScenario scenario) {
  switch (scenario) {
    case TestScenario::EMPTY:
      return "Empty environment with only boundary walls";
    case TestScenario::SPARSE_5:
      return "Sparse obstacles (5 well-spaced obstacles)";
    case TestScenario::SPARSE_10:
      return "Medium density obstacles (10 varied obstacles)";
    case TestScenario::CORRIDOR:
      return "Narrow corridor requiring careful navigation";
    case TestScenario::U_TRAP:
      return "U-shaped trap to test local minima escape";
    case TestScenario::NARROW_PASSAGE:
      return "Very narrow passage requiring precise control";
    case TestScenario::MAZE_SIMPLE:
      return "Simple maze with walls and obstacles";
    default:
      return "Unknown scenario";
  }
}

// ===== Recommended Waypoints =====

ScenarioWaypoints getRecommendedWaypoints(TestScenario scenario) {
  // Coordinate system: center at origin, map is 10x10m -> (-5,-5) to (5,5)
  // Default waypoints (bottom-left to top-right)
  ScenarioWaypoints waypoints{-4.0, -4.0, 4.0, 4.0};

  switch (scenario) {
    case TestScenario::EMPTY:
      waypoints = {-4.0, -4.0, 4.0, 4.0};
      break;
    case TestScenario::SPARSE_5:
    case TestScenario::SPARSE_10:
      waypoints = {-4.0, -4.0, 4.0, 4.0};
      break;
    case TestScenario::CORRIDOR:
      waypoints = {-4.5, 0.0, 4.5, 0.0};  // Left to right through corridor at y=0
      break;
    case TestScenario::U_TRAP:
      waypoints = {0.0, -3.0, 0.0, 3.0};  // Navigate out of U-trap vertically
      break;
    case TestScenario::NARROW_PASSAGE:
      waypoints = {-4.0, 0.0, 4.0, 0.0};  // Through narrow passage horizontally
      break;
    case TestScenario::MAZE_SIMPLE:
      waypoints = {-4.0, -4.0, 4.0, 4.0};  // Navigate through maze
      break;
  }

  return waypoints;
}

// ===== Scenario Generation Functions =====

namespace scenarios {

// ----- Empty Environment -----

std::vector<Obstacle> generateEmpty(double map_width, double map_height) {
  std::vector<Obstacle> obstacles;
  addBoundaryWalls(obstacles, map_width, map_height);
  return obstacles;
}

// ----- Sparse 5 Obstacles -----

std::vector<Obstacle> generateSparse5(double map_width, double map_height) {
  std::vector<Obstacle> obstacles;
  obstacles.reserve(9);  // 4 walls + 5 obstacles

  addBoundaryWalls(obstacles, map_width, map_height);

  // Add 5 well-spaced circular obstacles (radius 0.25-0.30m)
  // Coordinate system: center at origin, map is 10x10m -> (-5,-5) to (5,5)
  obstacles.emplace_back(makeCircle(-2.5, -2.5, 0.30));
  obstacles.emplace_back(makeCircle(2.5, -2.5, 0.25));
  obstacles.emplace_back(makeCircle(0.0, 0.0, 0.30));
  obstacles.emplace_back(makeCircle(-2.5, 2.5, 0.25));
  obstacles.emplace_back(makeCircle(2.5, 2.5, 0.30));

  return obstacles;
}

// ----- Sparse 10 Obstacles -----

std::vector<Obstacle> generateSparse10(double map_width, double map_height) {
  std::vector<Obstacle> obstacles;
  obstacles.reserve(14);  // 4 walls + 10 obstacles

  addBoundaryWalls(obstacles, map_width, map_height);

  // Add 10 varied obstacles (mix of circles and small walls)
  // Coordinate system: center at origin, map is 10x10m -> (-5,-5) to (5,5)

  // Circles: radius 0.20-0.30m
  obstacles.emplace_back(makeCircle(-4.0, -3.5, 0.25));
  obstacles.emplace_back(makeCircle(-1.5, -4.0, 0.20));
  obstacles.emplace_back(makeCircle(1.5, -3.5, 0.30));
  obstacles.emplace_back(makeCircle(4.0, -4.0, 0.25));

  // Small walls: length 1.0-1.5m, width 0.2m
  obstacles.emplace_back(makeWall(-2.5, 0.0, 1.2, 0.2, 0.0));
  obstacles.emplace_back(makeWall(2.0, 1.0, 1.5, 0.2, M_PI / 4));

  obstacles.emplace_back(makeCircle(-3.5, 3.0, 0.30));
  obstacles.emplace_back(makeCircle(0.0, 3.5, 0.25));
  obstacles.emplace_back(makeCircle(2.5, 4.0, 0.25));
  obstacles.emplace_back(makeCircle(4.0, 2.5, 0.20));

  return obstacles;
}

// ----- Corridor Scenario -----

std::vector<Obstacle> generateCorridor(double map_width, double map_height) {
  std::vector<Obstacle> obstacles;
  obstacles.reserve(10);  // 4 boundary walls + 2 corridor walls + 4 obstacle walls

  addBoundaryWalls(obstacles, map_width, map_height);

  // Similar to NeuPAN corridor design:
  // 2 long parallel horizontal walls creating a 2m wide corridor
  // 4 short angled walls inside the corridor as obstacles
  // Coordinate system: center at origin, map is 10x10m -> (-5,-5) to (5,5)

  double wall_thickness = 0.2;
  double corridor_half_width = 1.0;  // Corridor width = 2m (5.5x vehicle width)

  // Top long wall: y = +1.0, spanning entire map width
  obstacles.emplace_back(makeWall(0.0, corridor_half_width, 10.0, wall_thickness, 0.0));

  // Bottom long wall: y = -1.0, spanning entire map width
  obstacles.emplace_back(makeWall(0.0, -corridor_half_width, 10.0, wall_thickness, 0.0));

  // Short obstacle walls inside corridor (between y=-1 and y=+1)
  // Wall 1: vertical, left side
  obstacles.emplace_back(makeWall(-3.0, 0.0, 1.0, wall_thickness, M_PI / 2.0));

  // Wall 2: vertical, center-left
  obstacles.emplace_back(makeWall(-0.5, 0.3, 1.0, wall_thickness, M_PI / 2.0));

  // Wall 3: angled ~60°, center-right
  obstacles.emplace_back(makeWall(2.0, -0.2, 1.2, wall_thickness, 1.0));

  // Wall 4: angled ~240°, right side
  obstacles.emplace_back(makeWall(4.0, 0.4, 1.0, wall_thickness, 4.2));

  return obstacles;
}

// ----- U-Trap Scenario -----

std::vector<Obstacle> generateUTrap(double map_width, double map_height) {
  std::vector<Obstacle> obstacles;
  obstacles.reserve(7);  // 4 walls + 3 U-trap walls

  addBoundaryWalls(obstacles, map_width, map_height);

  // Create U-shaped trap (opening toward bottom)
  // Trap width: 2.0m (~5.5x vehicle width) for escape challenge
  double trap_x = map_width / 2.0;
  double trap_y = map_height / 2.0;
  double trap_width = 2.0;   // Reduced from 6.0m
  double trap_height = 4.0;  // Reduced from 8.0m
  double wall_thickness = 0.2;  // Reduced from 0.3m

  // Left wall of U
  obstacles.emplace_back(makeWall(
    trap_x - trap_width / 2.0,
    trap_y,
    trap_height,
    wall_thickness,
    M_PI / 2.0
  ));

  // Right wall of U
  obstacles.emplace_back(makeWall(
    trap_x + trap_width / 2.0,
    trap_y,
    trap_height,
    wall_thickness,
    M_PI / 2.0
  ));

  // Top wall of U
  obstacles.emplace_back(makeWall(
    trap_x,
    trap_y + trap_height / 2.0,
    trap_width,
    wall_thickness,
    0.0
  ));

  return obstacles;
}

// ----- Narrow Passage Scenario -----

std::vector<Obstacle> generateNarrowPassage(double map_width, double map_height) {
  std::vector<Obstacle> obstacles;
  obstacles.reserve(6);  // 4 walls + 2 passage walls

  addBoundaryWalls(obstacles, map_width, map_height);

  // Create extremely narrow passage (0.55m wide, ~1.5x vehicle width)
  // This requires very precise control for the 0.363m wide limo
  double center_x = map_width / 2.0;
  double center_y = map_height / 2.0;
  double passage_width = 0.55;  // Reduced from 1.0m to 0.55m
  double wall_length = 6.0;     // Reduced from 8.0m to 6.0m
  double wall_thickness = 0.2;  // Reduced from 0.3m to 0.2m

  // Top wall
  obstacles.emplace_back(makeWall(
    center_x,
    center_y + passage_width / 2.0,
    wall_length,
    wall_thickness,
    0.0
  ));

  // Bottom wall
  obstacles.emplace_back(makeWall(
    center_x,
    center_y - passage_width / 2.0,
    wall_length,
    wall_thickness,
    0.0
  ));

  return obstacles;
}

// ----- Simple Maze Scenario -----

std::vector<Obstacle> generateMazeSimple(double map_width, double map_height) {
  std::vector<Obstacle> obstacles;
  obstacles.reserve(15);  // 4 walls + ~11 maze obstacles

  addBoundaryWalls(obstacles, map_width, map_height);

  // Create a simple maze structure with appropriately scaled obstacles
  double wall_thickness = 0.2;  // Reduced from 0.3m

  // Vertical walls creating chambers (length 2-4m)
  obstacles.emplace_back(makeWall(7.0, 5.0, 4.0, wall_thickness, M_PI / 2.0));
  obstacles.emplace_back(makeWall(13.0, 15.0, 4.0, wall_thickness, M_PI / 2.0));

  // Horizontal walls (length 2-3m)
  obstacles.emplace_back(makeWall(10.0, 7.0, 3.0, wall_thickness, 0.0));
  obstacles.emplace_back(makeWall(10.0, 13.0, 3.0, wall_thickness, 0.0));

  // Add some circular obstacles in chambers (radius 0.25-0.35m)
  obstacles.emplace_back(makeCircle(4.0, 4.0, 0.30));
  obstacles.emplace_back(makeCircle(10.0, 10.0, 0.35));
  obstacles.emplace_back(makeCircle(16.0, 6.0, 0.25));
  obstacles.emplace_back(makeCircle(4.0, 16.0, 0.30));
  obstacles.emplace_back(makeCircle(16.0, 16.0, 0.25));

  // Additional walls for complexity (length 2-3m)
  obstacles.emplace_back(makeWall(4.0, 10.0, 2.5, wall_thickness, M_PI / 2.0));
  obstacles.emplace_back(makeWall(16.0, 10.0, 2.5, wall_thickness, M_PI / 2.0));

  return obstacles;
}

// ===== Helper Functions =====

Obstacle makeWall(double x, double y, double length, double width, double yaw) {
  Obstacle wall;
  wall.type = Obstacle::RECTANGLE;
  wall.x = x;
  wall.y = y;
  wall.length = length;
  wall.width = width;
  wall.yaw = yaw;
  wall.radius = 0.0;  // Not used for rectangles
  return wall;
}

Obstacle makeCircle(double x, double y, double radius) {
  Obstacle circle;
  circle.type = Obstacle::CIRCLE;
  circle.x = x;
  circle.y = y;
  circle.radius = radius;
  circle.length = 0.0;  // Not used for circles
  circle.width = 0.0;   // Not used for circles
  circle.yaw = 0.0;     // Not used for circles
  return circle;
}

void addBoundaryWalls(
  std::vector<Obstacle>& obstacles,
  double map_width,
  double map_height,
  double wall_thickness
) {
  double half_width = map_width / 2.0;
  double half_height = map_height / 2.0;

  // Coordinate system: center at origin (-width/2, -height/2) to (+width/2, +height/2)

  // Bottom wall (y = -half_height)
  obstacles.emplace_back(makeWall(
    0.0, -half_height,
    map_width + 2 * wall_thickness,
    wall_thickness,
    0.0
  ));

  // Top wall (y = +half_height)
  obstacles.emplace_back(makeWall(
    0.0, half_height,
    map_width + 2 * wall_thickness,
    wall_thickness,
    0.0
  ));

  // Left wall (x = -half_width)
  obstacles.emplace_back(makeWall(
    -half_width, 0.0,
    map_height,
    wall_thickness,
    M_PI / 2.0
  ));

  // Right wall (x = +half_width)
  obstacles.emplace_back(makeWall(
    half_width, 0.0,
    map_height,
    wall_thickness,
    M_PI / 2.0
  ));
}

}  // namespace scenarios

}  // namespace ddr_minimal_sim
