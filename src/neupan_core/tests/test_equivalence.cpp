/*
 * Equation-level regression tests against the original NeuPAN definitions.
 */

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include "neupan/initial_path.hpp"
#include "neupan/neupan_planner.hpp"
#include "neupan/nrmp.hpp"
#include "neupan/obstacle_prediction.hpp"
#include "neupan/pan.hpp"
#include "neupan/robot.hpp"

using namespace neupan;

namespace {

Robot testRobot(int horizon = 3) {
  return Robot::diffRectangle(horizon, 0.1, Vec2(2.0, 1.0),
                              Vec2(1.0, 2.0), 1.0, 0.6);
}

Vec3 nonlinearDiffStep(const Robot& robot, const Vec3& s, const Vec2& u) {
  return s + robot.dt *
                 Vec3(u(0) * std::cos(s(2)), u(0) * std::sin(s(2)), u(1));
}

}  // namespace

TEST(Equivalence, DiffLinearizationIsOriginalFirstOrderModel) {
  const Robot robot = testRobot();
  const Vec3 s0(1.2, -0.7, 0.63);
  const Vec2 u0(0.8, -0.25);

  Mat33 A;
  Mat32 B;
  Vec3 C;
  robot.linearize(s0, u0, A, B, C);

  // The affine model must interpolate the nonlinear Euler step at the
  // nominal point, as the Python A/B/C formula does.
  EXPECT_LT((A * s0 + B * u0 + C - nonlinearDiffStep(robot, s0, u0)).norm(),
            1e-14);

  // Its local error must be second order in a simultaneous state/control
  // perturbation. Halving epsilon should reduce the error by about four.
  const Vec3 ds(0.2, -0.1, 0.3);
  const Vec2 du(-0.4, 0.2);
  const auto error = [&](double eps) {
    const Vec3 s = s0 + eps * ds;
    const Vec2 u = u0 + eps * du;
    return (A * s + B * u + C - nonlinearDiffStep(robot, s, u)).norm();
  };
  EXPECT_GT(error(1e-3), 0.0);
  EXPECT_NEAR(error(5e-4) / error(1e-3), 0.25, 2e-3);
}

TEST(Equivalence, FaFbMatchesOriginalDualAffineFormAndPadding) {
  const Robot robot = testRobot(2);
  NRMPParams params;
  params.max_num = 3;
  NRMP nrmp(robot, params);

  std::vector<Mat> mu(3, Mat::Zero(robot.G.rows(), 2));
  std::vector<Mat2X> lam(3, Mat2X::Zero(2, 2));
  std::vector<Mat2X> points(3, Mat2X::Zero(2, 2));
  for (int t = 0; t <= robot.T; ++t) {
    mu[t].col(0) << 0.2, 0.3, 0.1, 0.4;
    mu[t].col(1) << 0.1, 0.5, 0.2, 0.2;
    lam[t] << 0.7, -0.2, -0.4, 0.6;
    points[t] << 1.0 + t, -0.5, 0.2, 0.8 + t;
  }

  std::vector<Mat> fa;
  std::vector<Vec> fb;
  nrmp.buildFaFb(mu, lam, points, fa, fb);

  for (int t = 0; t < robot.T; ++t) {
    const int stage = t + 1;
    for (int k = 0; k < 2; ++k) {
      EXPECT_LT((fa[t].row(k) - lam[stage].col(k).transpose()).norm(), 1e-14);
      const double expected = lam[stage].col(k).dot(points[stage].col(k)) +
                              mu[stage].col(k).dot(robot.h);
      EXPECT_NEAR(fb[t](k), expected, 1e-14);
    }
    // Upstream repeats the closest constraint when fewer than max_num points
    // are available.
    EXPECT_LT((fa[t].row(2) - fa[t].row(0)).norm(), 1e-14);
    EXPECT_DOUBLE_EQ(fb[t](2), fb[t](0));
  }
}

TEST(Equivalence, DynamicObstaclePredictionMatchesOriginalPanFormula) {
  Mat2X points(2, 6);
  Mat2X velocities(2, 6);
  for (int i = 0; i < 6; ++i) {
    points.col(i) << 10.0 + i, -2.0 * i;
    velocities.col(i) << 0.1 * i, 1.0 + i;
  }

  const auto prediction =
      predictObstaclePoints(points, velocities, 3, 0.2, 3);
  ASSERT_EQ(prediction.size(), 4U);

  // np.linspace(0, 5, 3).astype(int) -> [0, 2, 5]. Position and velocity
  // must use the same indices, then p_t = p_0 + t*dt*v.
  const std::array<int, 3> indices{0, 2, 5};
  for (int t = 0; t <= 3; ++t)
    for (int k = 0; k < 3; ++k) {
      const Vec2 expected = points.col(indices[k]) +
                            t * 0.2 * velocities.col(indices[k]);
      EXPECT_LT((prediction[t].col(k) - expected).norm(), 1e-14);
    }

  Mat2X bad_velocities(2, 5);
  EXPECT_THROW(predictObstaclePoints(points, bad_velocities, 3, 0.2, 3),
               std::invalid_argument);
}

TEST(Equivalence, NoObstacleProblemOmitsDistanceVariables) {
  const Robot robot = testRobot();
  NRMPParams params;
  params.max_num = 0;
  NRMP nrmp(robot, params);

  Mat3X nom_s = Mat3X::Zero(3, robot.T + 1);
  Mat2X nom_u = Mat2X::Zero(2, robot.T);
  Mat3X ref_s = Mat3X::Zero(3, robot.T + 1);
  for (int t = 0; t <= robot.T; ++t) ref_s(0, t) = 0.1 * t;
  const Vec ref_us = Vec::Constant(robot.T, 1.0);

  const NRMP::Result result = nrmp.solve(nom_s, nom_u, ref_s, ref_us, {}, {});
  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.d.size(), 0);
  EXPECT_EQ(result.s.cols(), robot.T + 1);
  EXPECT_EQ(result.u.cols(), robot.T);

  for (int t = 0; t < robot.T; ++t) {
    Mat33 A;
    Mat32 B;
    Vec3 C;
    robot.linearize(nom_s.col(t), nom_u.col(t), A, B, C);
    EXPECT_LT((result.s.col(t + 1) -
               (A * result.s.col(t) + B * result.u.col(t) + C))
                  .norm(),
              2e-5);
  }
}

TEST(Equivalence, PanNavigationOnlyNeedsNoDuneModel) {
  const Robot robot = testRobot();
  NRMPParams params;
  params.max_num = 8;
  PAN::Options options;
  options.dune_max_num = 0;
  options.iter_num = 2;

  PAN pan(robot, "", params, options);
  EXPECT_EQ(pan.nrmp().params().max_num, 0);

  Mat3X nom_s = Mat3X::Zero(3, robot.T + 1);
  Mat2X nom_u = Mat2X::Zero(2, robot.T);
  Mat3X ref_s = Mat3X::Zero(3, robot.T + 1);
  const Vec ref_us = Vec::Constant(robot.T, 0.5);
  const PAN::Output out =
      pan.forward(nom_s, nom_u, ref_s, ref_us, Mat2X(2, 0));
  EXPECT_TRUE(out.solved);
  EXPECT_EQ(out.opt_d.size(), 0);
  EXPECT_TRUE(std::isinf(out.min_distance));
}

TEST(Equivalence, NavigationOnlyYamlNeedsNoCheckpoint) {
  NeuPANPlanner planner = NeuPANPlanner::fromYaml(
      std::string(NEUPAN_TEST_DATA_DIR) + "/no_obstacles.yaml");
  EXPECT_EQ(planner.pan().nrmp().params().max_num, 0);
}

TEST(Equivalence, ExternalPathPreservesSamplesGearAndAverageInterval) {
  const Robot robot = testRobot();
  InitialPath::Options options;
  InitialPath path(robot, 1.0, options);

  std::vector<InitialPath::PathPoint> input{
      {0.0, 0.0, 0.1, 1.0},
      {1.0, 0.0, 0.2, 1.0},
      {4.0, 0.0, 3.0, -1.0},
  };
  path.setInitialPath(input);

  ASSERT_EQ(path.initialPath().size(), input.size());
  for (size_t i = 0; i < input.size(); ++i)
    EXPECT_LT((path.initialPath()[i] - input[i]).norm(), 1e-14);
  EXPECT_DOUBLE_EQ(path.pathInterval(), 2.0);
}
