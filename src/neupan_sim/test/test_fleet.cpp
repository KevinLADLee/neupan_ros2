#include <cmath>
#include <gtest/gtest.h>
#include "neupan_sim/simulation.hpp"
using namespace neupan_sim;

SimulationConfig fleetWorld() {
  SimulationConfig c;
  c.min_x = c.min_y = -10; c.max_x = c.max_y = 10;
  c.command_timeout = 10;
  c.max_linear_acceleration = c.max_angular_acceleration = 100;
  c.integration_substeps = 10;
  return c;
}

TEST(Fleet, ExactPolygonSensingAndNoSelfHits) {
  Simulation a(fleetWorld(), {}, {8,8}, {}, {});
  auto cfg = fleetWorld();
  cfg.robot_vertices = {{0,0},{2,0},{0,2}};
  Simulation b(cfg, {2,-1,0}, {8,8}, {}, {});
  Simulation::synchronizeRobots({&a,&b});
  EXPECT_EQ(a.peerCount(), 1u);
  EXPECT_NEAR(a.raycast({}, 0, .05, 8).range, 2, 1e-12);
  EXPECT_NEAR(b.raycast({2.1,-.9,0}, 0, .05, 8).range, 7.9, 1e-12);
  // A smaller robot in the triangular peer's bounding box but outside its body.
  Simulation outside(fleetWorld(), {3.7,.7,0}, {8,8}, {}, {});
  Simulation::synchronizeRobots({&outside,&b});
  EXPECT_GT(outside.clearance(), 0);
  EXPECT_EQ(outside.result(), Result::Running);
}

TEST(Fleet, RayVelocityIncludesRigidRotationAndNearestOcclusion) {
  Simulation a(fleetWorld(), {}, {8,8}, {}, {});
  Simulation b(fleetWorld(), {2,0,0}, {8,8}, {}, {});
  b.setCommand({.5,1});
  Simulation::stepTogether({&a,&b}, .01);
  const auto hit = a.raycast(a.pose(), 0, .05, 8);
  const auto v = b.velocity(); const auto p = b.pose();
  EXPECT_TRUE(hit.dynamic);
  EXPECT_NEAR(hit.velocity.x, v.linear*std::cos(p.yaw)-v.angular*(hit.point.y-p.y), 1e-10);
  EXPECT_NEAR(hit.velocity.y, v.linear*std::sin(p.yaw)+v.angular*(hit.point.x-p.x), 1e-10);
  Simulation occluded(fleetWorld(), {}, {8,8}, {{{1,0},.2,{}}}, {});
  Simulation::synchronizeRobots({&occluded,&b});
  EXPECT_NEAR(occluded.raycast({},0,.05,8).range,.8,1e-12);
  EXPECT_FALSE(occluded.raycast({},0,.05,8).dynamic);
}

TEST(Fleet, SimultaneousContactAndOrderIndependence) {
  Simulation a(fleetWorld(), {-1,0,0}, {8,0}, {}, {});
  Simulation b(fleetWorld(), {1,0,M_PI}, {-8,0}, {}, {});
  Simulation c(fleetWorld(), {-1,0,0}, {8,0}, {}, {});
  Simulation d(fleetWorld(), {1,0,M_PI}, {-8,0}, {}, {});
  for (auto* r : {&a,&b,&c,&d}) r->setCommand({1,0});
  for (int i=0;i<100;++i) {
    Simulation::stepTogether({&a,&b},.01);
    Simulation::stepTogether({&d,&c},.01);
  }
  EXPECT_EQ(a.result(),Result::Collision); EXPECT_EQ(b.result(),Result::Collision);
  EXPECT_DOUBLE_EQ(a.pose().x,c.pose().x); EXPECT_DOUBLE_EQ(b.pose().x,d.pose().x);
  EXPECT_DOUBLE_EQ(a.elapsedTime(),b.elapsedTime());
  EXPECT_DOUBLE_EQ(a.velocity().linear,0);
}

TEST(Fleet, ParkedRobotRemainsVisibleAndCollisionCanOverrideArrival) {
  Simulation parked(fleetWorld(), {}, {0,0}, {}, {});
  Simulation moving(fleetWorld(), {-1,0,0}, {8,0}, {}, {});
  Simulation::synchronizeRobots({&parked,&moving});
  EXPECT_EQ(parked.result(),Result::GoalReached);
  EXPECT_NEAR(moving.raycast(moving.pose(),0,.05,8).range,.75,1e-12);
  moving.setCommand({1,0});
  for (int i=0;i<100;++i) Simulation::stepTogether({&parked,&moving},.01);
  EXPECT_EQ(parked.result(),Result::Collision); EXPECT_EQ(moving.result(),Result::Collision);
  EXPECT_DOUBLE_EQ(parked.pose().x,0);
}

TEST(Fleet, InitialContainmentAndInvalidGroups) {
  auto large=fleetWorld(); large.robot_length=large.robot_width=2;
  Simulation a(large, {}, {8,8}, {}, {}), b(fleetWorld(), {}, {8,8}, {}, {});
  Simulation::synchronizeRobots({&a,&b});
  EXPECT_EQ(a.result(),Result::Collision); EXPECT_EQ(b.result(),Result::Collision);
  EXPECT_THROW(Simulation::stepTogether({&a,&a},.01),std::invalid_argument);
  EXPECT_THROW(Simulation::stepTogether({nullptr},.01),std::invalid_argument);
  EXPECT_THROW(Simulation::stepTogether({&a},0),std::invalid_argument);
}

TEST(Fleet, EnvironmentKeepsMovingAfterFirstRobotParks) {
  const std::vector<CircleObstacle> circles={{{3,3},.2,{.2,0}}};
  Simulation a(fleetWorld(), {}, {0,0}, circles, {});
  Simulation b(fleetWorld(), {-3,0,0}, {8,8}, circles, {});
  Simulation::stepTogether({&a,&b},1);
  EXPECT_NEAR(a.circles()[0].center.x,3.2,1e-12);
  EXPECT_DOUBLE_EQ(a.circles()[0].center.x,b.circles()[0].center.x);
}
