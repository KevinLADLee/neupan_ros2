#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <limits>
#include <unistd.h>

#include "neupan/neupan_planner.hpp"

using namespace neupan;

namespace {
struct YamlFile {
  char path[64] = "/tmp/neupan-polygon-XXXXXX";
  explicit YamlFile(const std::string& robot) {
    const int fd = mkstemp(path);
    if (fd < 0) throw std::runtime_error("temporary YAML creation failed");
    close(fd);
    std::ofstream(path) << "robot:\n  kinematics: diff\n" << robot
                        << "\npan:\n  dune_max_num: 0\n";
  }
  ~YamlFile() { std::remove(path); }
};

Mat2X polygon(std::initializer_list<Vec2> points) {
  Mat2X v(2, points.size());
  Eigen::Index i = 0;
  for (const auto& p : points) v.col(i++) = p;
  return v;
}
}  // namespace

TEST(RobotGeometry, UpstreamTrapezoidWindingAndCoefficients) {
  // Exact vertices from upstream example/polygon_robot/diff/planner.yaml.
  Mat2X cw = polygon({{-0.8, -1}, {-1.8, 1}, {1.8, 1}, {0.8, -1}});
  Mat2X ccw = polygon({{-0.8, -1}, {0.8, -1}, {1.8, 1}, {-1.8, 1}});
  Mat g, reverse_g;
  Vec h, reverse_h;
  genInequalFromVertex(cw, g, h);
  genInequalFromVertex(ccw, reverse_g, reverse_h);
  EXPECT_LT((g - reverse_g).norm(), 1e-14);
  EXPECT_LT((h - reverse_h).norm(), 1e-14);
  Mat expected(4, 2);
  expected << 0, -1.6, 2, -1, 0, 3.6, -2, -1;
  Vec expected_h(4);
  expected_h << 1.6, 2.6, 3.6, 2.6;
  EXPECT_LT((g - expected).norm(), 1e-14);
  EXPECT_LT((h - expected_h).norm(), 1e-14);
  EXPECT_TRUE((g * Vec2(1.4, 0.8) - h).maxCoeff() <= 0);
  EXPECT_GT((g * Vec2(1.4, -0.8) - h).maxCoeff(), 0);
}

TEST(RobotGeometry, YamlVerticesOverrideDimensionsAndReachPlanner) {
  YamlFile file("  length: 99\n  width: 99\n  wheelbase: 99\n"
                "  vertices: [[0, 0], [2, 0], [0, 1]]\n");
  auto planner = NeuPANPlanner::fromYaml(file.path);
  ASSERT_EQ(planner.robot().vertices.cols(), 3);
  EXPECT_NEAR(planner.robot().h(1), 2, 1e-14);
  planner.setWaypoints({Vec3(0, 0, 0), Vec3(3, 0, 0)});
  NeuPANPlanner::Info info;
  const auto command = planner.forward(Vec3::Zero(), Mat2X(2, 0), info);
  EXPECT_TRUE(info.solved);
  EXPECT_TRUE(command.allFinite());
}

TEST(RobotGeometry, RectangleAndNullVerticesKeepAxleOffset) {
  YamlFile file("  length: 2\n  width: 1\n  wheelbase: 0.4\n  vertices: null\n");
  auto planner = NeuPANPlanner::fromYaml(file.path);
  const auto& v = planner.robot().vertices;
  ASSERT_EQ(v.cols(), 4);
  EXPECT_NEAR(v.row(0).minCoeff(), -0.8, 1e-14);
  EXPECT_NEAR(v.row(0).maxCoeff(), 1.2, 1e-14);
}

TEST(RobotGeometry, RejectMalformedConcaveAndDegenerateYaml) {
  for (const std::string vertices : {
           "[]", "[0, 1, 2]", "[[0, 0], [1, 0], [0, 1, 2]]",
           "[[0, 0], [1, 0], [2, 0]]", "[[0, 0], [1, 0], [0, 0]]",
           "[[0, 0], [1, 1], [0, 1], [1, 0]]",
           "[[0, 0], [2, 0], [1, 0.5], [2, 1], [0, 1]]",
           "[[0, 0], [.nan, 0], [0, 1]]"}) {
    YamlFile file("  vertices: " + vertices + "\n");
    EXPECT_THROW(NeuPANPlanner::fromYaml(file.path), std::exception) << vertices;
  }
  // A pentagram passes a local turn-sign sweep, but is not an ordered convex boundary.
  Mat2X star(2, 5);
  for (int i = 0; i < 5; ++i) {
    const double angle = 4 * std::acos(-1.0) * i / 5;
    star.col(i) << std::cos(angle), std::sin(angle);
  }
  Mat g;
  Vec h;
  EXPECT_THROW(genInequalFromVertex(star, g, h), std::invalid_argument);
}

TEST(RobotGeometry, PolygonCannotUseMismatchedRectangleCheckpoint) {
  NeuPANPlanner::Config c;
  c.vertices = polygon({{0, 0}, {1, 0}, {0, 1}});
  c.dune_checkpoint = std::string(NEUPAN_MODEL_DIR) + "/diff_default.bin";
  EXPECT_THROW(NeuPANPlanner{c}, std::runtime_error);
  // Same edge count is insufficient: footprint metadata must also match.
  c.vertices = polygon({{-0.8, -1}, {-1.8, 1}, {1.8, 1}, {0.8, -1}});
  c.dune_checkpoint = std::string(NEUPAN_MODEL_DIR) + "/diff_scout_mini_612x580.bin";
  EXPECT_THROW(NeuPANPlanner{c}, std::runtime_error);
}
