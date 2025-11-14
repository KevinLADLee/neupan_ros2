#ifndef DDR_MINIMAL_SIM_SCENARIO_LIBRARY_HPP_
#define DDR_MINIMAL_SIM_SCENARIO_LIBRARY_HPP_

#include "ddr_minimal_sim/environment.hpp"
#include <string>
#include <vector>

namespace ddr_minimal_sim {

/**
 * @brief Predefined test scenarios for path planning algorithm testing
 *
 * Each scenario is designed to test specific aspects of navigation:
 * - EMPTY: Basic sanity check with no obstacles
 * - SPARSE_5: Simple obstacle avoidance (5 obstacles)
 * - SPARSE_10: Medium complexity (10 obstacles)
 * - CORRIDOR: Narrow passage navigation
 * - U_TRAP: Local minima escape testing
 * - NARROW_PASSAGE: Precision control testing
 * - MAZE_SIMPLE: Complex path planning
 */
enum class TestScenario {
  EMPTY,
  SPARSE_5,
  SPARSE_10,
  CORRIDOR,
  U_TRAP,
  NARROW_PASSAGE,
  MAZE_SIMPLE
};

/**
 * @brief Convert scenario enum to string name
 */
std::string scenarioToString(TestScenario scenario);

/**
 * @brief Convert string name to scenario enum
 * @throws std::invalid_argument if scenario name is unknown
 */
TestScenario stringToScenario(const std::string& name);

/**
 * @brief Generate obstacles for a given test scenario
 *
 * @param scenario The scenario to generate
 * @param map_width Map width in meters (default: 20.0)
 * @param map_height Map height in meters (default: 20.0)
 * @return Vector of obstacles representing the scenario
 */
std::vector<Obstacle> generateScenario(
  TestScenario scenario,
  double map_width = 20.0,
  double map_height = 20.0
);

/**
 * @brief Get description of a test scenario
 */
std::string getScenarioDescription(TestScenario scenario);

/**
 * @brief Get recommended start and goal points for a scenario
 */
struct ScenarioWaypoints {
  double start_x;
  double start_y;
  double goal_x;
  double goal_y;
};

ScenarioWaypoints getRecommendedWaypoints(TestScenario scenario);

// ===== Scenario Generation Functions =====

namespace scenarios {

/**
 * @brief Empty environment (only boundary walls)
 */
std::vector<Obstacle> generateEmpty(double map_width, double map_height);

/**
 * @brief Sparse obstacles (5 obstacles)
 */
std::vector<Obstacle> generateSparse5(double map_width, double map_height);

/**
 * @brief Sparse obstacles (10 obstacles)
 */
std::vector<Obstacle> generateSparse10(double map_width, double map_height);

/**
 * @brief Corridor scenario with narrow passages
 */
std::vector<Obstacle> generateCorridor(double map_width, double map_height);

/**
 * @brief U-trap scenario for local minima testing
 */
std::vector<Obstacle> generateUTrap(double map_width, double map_height);

/**
 * @brief Narrow passage requiring precise control
 */
std::vector<Obstacle> generateNarrowPassage(double map_width, double map_height);

/**
 * @brief Simple maze scenario
 */
std::vector<Obstacle> generateMazeSimple(double map_width, double map_height);

// ===== Helper Functions =====

/**
 * @brief Create a rectangular wall obstacle
 */
Obstacle makeWall(double x, double y, double length, double width, double yaw);

/**
 * @brief Create a circular obstacle
 */
Obstacle makeCircle(double x, double y, double radius);

/**
 * @brief Add boundary walls to obstacle list
 */
void addBoundaryWalls(
  std::vector<Obstacle>& obstacles,
  double map_width,
  double map_height,
  double wall_thickness = 0.1
);

}  // namespace scenarios

}  // namespace ddr_minimal_sim

#endif  // DDR_MINIMAL_SIM_SCENARIO_LIBRARY_HPP_
