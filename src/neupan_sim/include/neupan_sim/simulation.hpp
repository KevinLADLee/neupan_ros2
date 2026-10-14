#pragma once

#include <limits>
#include <string>
#include <vector>

namespace neupan_sim {

struct Vec2 {
  double x = 0.0;
  double y = 0.0;
};

struct Pose2 {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct Twist2 {
  double linear = 0.0;
  double angular = 0.0;
};

struct CircleObstacle {
  Vec2 center;
  double radius = 0.0;
  Vec2 velocity;
};

struct SegmentObstacle {
  Vec2 first;
  Vec2 second;
};

struct RayHit {
  bool hit = false;
  double range = std::numeric_limits<double>::infinity();
  Vec2 point;
  Vec2 velocity;
  bool dynamic = false;
  bool peer = false;
};

enum class Result { Running, GoalReached, Collision, TimedOut };

const char* resultName(Result result);

struct SimulationConfig {
  double min_x = -2.0;
  double max_x = 8.0;
  double min_y = -4.0;
  double max_y = 4.0;
  double robot_length = 0.5;
  double robot_width = 0.5;
  std::vector<Vec2> robot_vertices;  // body-frame convex polygon; overrides size
  double max_linear_speed = 2.0;
  double max_angular_speed = 1.5;
  double max_linear_acceleration = 2.0;
  double max_angular_acceleration = 2.0;
  double command_timeout = 0.25;
  double goal_tolerance = 0.12;
  double simulation_timeout = 30.0;
  int integration_substeps = 2;
};

class Simulation {
 public:
  Simulation(SimulationConfig config, Pose2 initial_pose, Vec2 goal,
             std::vector<CircleObstacle> circles,
             std::vector<SegmentObstacle> segments);

  void setCommand(Twist2 command);
  void step(double dt);
  // All members must describe the same environment. Integrate first, then
  // snapshot peers and evaluate contacts, so list order cannot bias collisions.
  static void synchronizeRobots(const std::vector<Simulation*>& robots);
  static void stepTogether(const std::vector<Simulation*>& robots, double dt);
  std::vector<Vec2> worldVertices() const;
  std::size_t peerCount() const { return peers_.size(); }

  RayHit raycast(const Pose2& sensor_pose, double local_angle,
                 double range_min, double range_max) const;

  const SimulationConfig& config() const { return config_; }
  const Pose2& pose() const { return pose_; }
  const Twist2& velocity() const { return velocity_; }
  const Vec2& goal() const { return goal_; }
  const std::vector<CircleObstacle>& circles() const { return circles_; }
  const std::vector<SegmentObstacle>& segments() const { return segments_; }
  Result result() const { return result_; }
  double elapsedTime() const { return elapsed_time_; }
  double commandAge() const { return command_age_; }
  double pathLength() const { return path_length_; }
  double goalDistance() const;
  double clearance() const;
  double minimumClearance() const { return minimum_clearance_; }

 private:
  void validate() const;
  void integrateRobot(double dt);
  void integrateObstacles(double dt);
  bool collides() const;
  void evaluateResult();
  struct Peer {
    Pose2 pose;
    Twist2 velocity;
    std::vector<Vec2> vertices;
  };
  std::vector<Peer> peers_;

  SimulationConfig config_;
  Pose2 pose_;
  Vec2 goal_;
  std::vector<CircleObstacle> circles_;
  std::vector<SegmentObstacle> segments_;
  Twist2 command_;
  Twist2 velocity_;
  Result result_ = Result::Running;
  double elapsed_time_ = 0.0;
  double command_age_ = std::numeric_limits<double>::infinity();
  double path_length_ = 0.0;
  double minimum_clearance_ = std::numeric_limits<double>::infinity();
};

Pose2 composePose(const Pose2& parent, const Pose2& child);

}  // namespace neupan_sim
