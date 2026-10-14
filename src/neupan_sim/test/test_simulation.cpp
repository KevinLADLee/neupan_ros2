#include <cmath>
#include <algorithm>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "neupan_sim/simulation.hpp"

namespace {

using neupan_sim::CircleObstacle;
using neupan_sim::Pose2;
using neupan_sim::Result;
using neupan_sim::SegmentObstacle;
using neupan_sim::Simulation;
using neupan_sim::SimulationConfig;
using neupan_sim::Vec2;

TEST(PolygonFootprint, ClearanceUsesEdgesInteriorAndPose) {
  SimulationConfig config;
  config.min_x = config.min_y = -10;
  config.max_x = config.max_y = 10;
  config.robot_vertices = {{0, 0}, {2, 0}, {0, 2}};
  // Outside triangle, inside its bounding box: must not report collision.
  Simulation outside(config, {}, {8, 8}, {{{1.5, 1.5}, 0.1, {}}}, {});
  EXPECT_NEAR(outside.clearance(), std::sqrt(0.5) - 0.1, 1e-10);
  EXPECT_EQ(outside.result(), Result::Running);
  Simulation inside(config, {}, {8, 8}, {{{0.25, 0.25}, 0.1, {}}}, {});
  EXPECT_NEAR(inside.clearance(), -0.35, 1e-10);
  EXPECT_EQ(inside.result(), Result::Collision);
  // Rotate by pi/2 and translate (3, 1); the clearance must be invariant.
  Simulation rotated(config, {3, 1, std::acos(-1.0) / 2}, {8, 8},
                     {{{1.5, 2.5}, 0.1, {}}}, {});
  EXPECT_NEAR(rotated.clearance(), outside.clearance(), 1e-10);
  std::reverse(config.robot_vertices.begin(), config.robot_vertices.end());
  Simulation reversed(config, {}, {8, 8}, {{{1.5, 1.5}, 0.1, {}}}, {});
  EXPECT_NEAR(reversed.clearance(), outside.clearance(), 1e-10);
  Simulation segment(config, {}, {8, 8}, {}, {{{-1, 0.5}, {2, 0.5}}});
  EXPECT_EQ(segment.result(), Result::Collision);
  Simulation contained(config, {}, {8, 8}, {}, {{{.2, .2}, {.4, .4}}});
  EXPECT_EQ(contained.result(), Result::Collision);
  config.max_x = 1;
  Simulation boundary(config, {}, {8, 8}, {}, {});
  EXPECT_EQ(boundary.result(), Result::Collision);
}

TEST(PolygonFootprint, RejectsInvalidGeometry) {
  for (const std::vector<Vec2> vertices : {
           std::vector<Vec2>{{0, 0}, {1, 0}},
           {{0, 0}, {1, 0}, {2, 0}},
           {{0, 0}, {1, 0}, {0, 0}},
           {{0, 0}, {2, 0}, {1, .5}, {2, 1}, {0, 1}},
           {{0, 0}, {std::numeric_limits<double>::quiet_NaN(), 0}, {0, 1}}}) {
    SimulationConfig config;
    config.robot_vertices = vertices;
    EXPECT_THROW(Simulation(config, {}, {5, 0}, {}, {}), std::invalid_argument);
  }
}

SimulationConfig openWorld() {
  SimulationConfig config;
  config.min_x = -10.0;
  config.max_x = 10.0;
  config.min_y = -10.0;
  config.max_y = 10.0;
  config.robot_length = 0.5;
  config.robot_width = 0.4;
  config.max_linear_speed = 5.0;
  config.max_angular_speed = 5.0;
  config.max_linear_acceleration = 10.0;
  config.max_angular_acceleration = 10.0;
  config.command_timeout = 1.0;
  config.goal_tolerance = 0.05;
  config.simulation_timeout = 20.0;
  config.integration_substeps = 1;
  return config;
}

TEST(Simulation, RaycastUsesExactCircleGeometryAndVelocity) {
  const CircleObstacle moving{{3.0, 0.0}, 0.5, {0.0, 0.6}};
  Simulation simulation(openWorld(), {}, {9.0, 9.0}, {moving}, {});

  const auto hit = simulation.raycast({}, 0.0, 0.05, 8.0);

  ASSERT_TRUE(hit.hit);
  EXPECT_NEAR(hit.range, 2.5, 1e-12);
  EXPECT_NEAR(hit.point.x, 2.5, 1e-12);
  EXPECT_NEAR(hit.velocity.y, 0.6, 1e-12);
  EXPECT_TRUE(hit.dynamic);
}

TEST(Simulation, RaycastSupportsSegmentsAndReportsNoReturn) {
  const SegmentObstacle wall{{2.0, -1.0}, {2.0, 1.0}};
  Simulation simulation(openWorld(), {}, {9.0, 9.0}, {}, {wall});

  EXPECT_NEAR(simulation.raycast({}, 0.0, 0.05, 8.0).range, 2.0, 1e-12);
  const auto miss = simulation.raycast({}, M_PI_2, 0.05, 8.0);
  EXPECT_FALSE(miss.hit);
  EXPECT_TRUE(std::isinf(miss.range));
}

TEST(Simulation, IntegratesAnExactAcceleratedUnicycleArc) {
  auto config = openWorld();
  config.max_linear_acceleration = 100.0;
  config.max_angular_acceleration = 100.0;
  Simulation simulation(config, {}, {9.0, 9.0}, {}, {});
  simulation.setCommand({1.0, 1.0});

  simulation.step(1.0);

  EXPECT_NEAR(simulation.pose().x, std::sin(1.0), 1e-12);
  EXPECT_NEAR(simulation.pose().y, 1.0 - std::cos(1.0), 1e-12);
  EXPECT_NEAR(simulation.pose().yaw, 1.0, 1e-12);
  EXPECT_NEAR(simulation.pathLength(), 1.0, 1e-12);
}

TEST(Simulation, StaleCommandDeceleratesUnderAccelerationLimit) {
  auto config = openWorld();
  config.command_timeout = 0.15;
  config.max_linear_acceleration = 2.0;
  Simulation simulation(config, {}, {9.0, 9.0}, {}, {});
  simulation.setCommand({1.0, 0.0});

  simulation.step(0.1);
  EXPECT_NEAR(simulation.velocity().linear, 0.2, 1e-12);
  simulation.step(0.1);
  EXPECT_NEAR(simulation.velocity().linear, 0.0, 1e-12);
}

TEST(Simulation, DetectsRectangleCollisionWithoutGridSampling) {
  const CircleObstacle obstacle{{0.3, 0.0}, 0.1, {}};
  Simulation simulation(openWorld(), {}, {9.0, 9.0}, {obstacle}, {});

  EXPECT_EQ(simulation.result(), Result::Collision);
  EXPECT_LT(simulation.minimumClearance(), 0.0);
}

TEST(Simulation, ReflectsMovingObstacleAtContinuousWorldBoundary) {
  auto config = openWorld();
  const CircleObstacle moving{{9.4, 0.0}, 0.5, {2.0, 0.0}};
  Simulation simulation(config, {}, {9.0, 9.0}, {moving}, {});

  simulation.step(0.2);

  EXPECT_NEAR(simulation.circles().front().center.x, 9.2, 1e-12);
  EXPECT_NEAR(simulation.circles().front().velocity.x, -2.0, 1e-12);
}

TEST(Simulation, LatchesGoalAndStopsMotion) {
  auto config = openWorld();
  config.goal_tolerance = 0.11;
  config.max_linear_acceleration = 100.0;
  Simulation simulation(config, {}, {0.1, 0.0}, {}, {});
  simulation.setCommand({1.0, 0.0});

  simulation.step(0.1);

  EXPECT_EQ(simulation.result(), Result::GoalReached);
  EXPECT_DOUBLE_EQ(simulation.velocity().linear, 0.0);
}

}  // namespace
