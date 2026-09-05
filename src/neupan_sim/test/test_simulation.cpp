#include <cmath>
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
