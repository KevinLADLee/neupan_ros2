#include "neupan_sim/simulation.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace neupan_sim {
namespace {

constexpr double kEpsilon = 1e-10;

double cross(const Vec2& a, const Vec2& b) {
  return a.x * b.y - a.y * b.x;
}

Vec2 subtract(const Vec2& a, const Vec2& b) {
  return {a.x - b.x, a.y - b.y};
}

double norm(const Vec2& value) {
  return std::hypot(value.x, value.y);
}

double normalizeAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

double moveToward(double current, double target, double maximum_delta) {
  return current +
         std::clamp(target - current, -maximum_delta, maximum_delta);
}

Vec2 worldToBody(const Pose2& pose, const Vec2& point) {
  const double dx = point.x - pose.x;
  const double dy = point.y - pose.y;
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {c * dx + s * dy, -s * dx + c * dy};
}

Vec2 bodyToWorld(const Pose2& pose, const Vec2& point) {
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {pose.x + c * point.x - s * point.y,
          pose.y + s * point.x + c * point.y};
}

double pointSegmentDistance(const Vec2& point, const SegmentObstacle& segment) {
  const Vec2 edge = subtract(segment.second, segment.first);
  const double length_squared = edge.x * edge.x + edge.y * edge.y;
  if (length_squared <= kEpsilon) return norm(subtract(point, segment.first));
  const Vec2 offset = subtract(point, segment.first);
  const double ratio = std::clamp(
      (offset.x * edge.x + offset.y * edge.y) / length_squared, 0.0, 1.0);
  const Vec2 nearest{segment.first.x + ratio * edge.x,
                     segment.first.y + ratio * edge.y};
  return norm(subtract(point, nearest));
}

int orientation(const Vec2& a, const Vec2& b, const Vec2& c) {
  const double value = cross(subtract(b, a), subtract(c, a));
  if (std::abs(value) <= kEpsilon) return 0;
  return value > 0.0 ? 1 : -1;
}

bool onSegment(const Vec2& a, const Vec2& b, const Vec2& point) {
  return point.x >= std::min(a.x, b.x) - kEpsilon &&
         point.x <= std::max(a.x, b.x) + kEpsilon &&
         point.y >= std::min(a.y, b.y) - kEpsilon &&
         point.y <= std::max(a.y, b.y) + kEpsilon;
}

bool segmentsIntersect(const SegmentObstacle& a, const SegmentObstacle& b) {
  const int o1 = orientation(a.first, a.second, b.first);
  const int o2 = orientation(a.first, a.second, b.second);
  const int o3 = orientation(b.first, b.second, a.first);
  const int o4 = orientation(b.first, b.second, a.second);
  if (o1 != o2 && o3 != o4) return true;
  if (o1 == 0 && onSegment(a.first, a.second, b.first)) return true;
  if (o2 == 0 && onSegment(a.first, a.second, b.second)) return true;
  if (o3 == 0 && onSegment(b.first, b.second, a.first)) return true;
  return o4 == 0 && onSegment(b.first, b.second, a.second);
}

double segmentDistance(const SegmentObstacle& a, const SegmentObstacle& b) {
  if (segmentsIntersect(a, b)) return 0.0;
  return std::min({pointSegmentDistance(a.first, b),
                   pointSegmentDistance(a.second, b),
                   pointSegmentDistance(b.first, a),
                   pointSegmentDistance(b.second, a)});
}

std::vector<Vec2> robotCorners(const Pose2& pose, double length, double width) {
  const double hx = length * 0.5;
  const double hy = width * 0.5;
  return {bodyToWorld(pose, {-hx, -hy}), bodyToWorld(pose, {hx, -hy}),
          bodyToWorld(pose, {hx, hy}), bodyToWorld(pose, {-hx, hy})};
}

std::vector<SegmentObstacle> robotEdges(const std::vector<Vec2>& corners) {
  return {{corners[0], corners[1]}, {corners[1], corners[2]},
          {corners[2], corners[3]}, {corners[3], corners[0]}};
}

bool pointInsideRobot(const Pose2& pose, double length, double width,
                      const Vec2& point) {
  const Vec2 local = worldToBody(pose, point);
  return std::abs(local.x) <= length * 0.5 + kEpsilon &&
         std::abs(local.y) <= width * 0.5 + kEpsilon;
}

double orientedBoxCircleClearance(const Pose2& pose, double length,
                                  double width,
                                  const CircleObstacle& circle) {
  const Vec2 local = worldToBody(pose, circle.center);
  const double dx = std::max(std::abs(local.x) - length * 0.5, 0.0);
  const double dy = std::max(std::abs(local.y) - width * 0.5, 0.0);
  const bool center_inside = std::abs(local.x) <= length * 0.5 &&
                             std::abs(local.y) <= width * 0.5;
  if (center_inside) {
    const double inside = std::min(length * 0.5 - std::abs(local.x),
                                   width * 0.5 - std::abs(local.y));
    return -inside - circle.radius;
  }
  return std::hypot(dx, dy) - circle.radius;
}

double rayCircle(const Vec2& origin, const Vec2& direction,
                 const CircleObstacle& circle) {
  const Vec2 offset = subtract(origin, circle.center);
  const double b = offset.x * direction.x + offset.y * direction.y;
  const double c = offset.x * offset.x + offset.y * offset.y -
                   circle.radius * circle.radius;
  const double discriminant = b * b - c;
  if (discriminant < 0.0) return std::numeric_limits<double>::infinity();
  const double root = std::sqrt(discriminant);
  const double near = -b - root;
  if (near >= 0.0) return near;
  const double far = -b + root;
  return far >= 0.0 ? far : std::numeric_limits<double>::infinity();
}

double raySegment(const Vec2& origin, const Vec2& direction,
                  const SegmentObstacle& segment) {
  const Vec2 edge = subtract(segment.second, segment.first);
  const double denominator = cross(direction, edge);
  if (std::abs(denominator) <= kEpsilon)
    return std::numeric_limits<double>::infinity();
  const Vec2 offset = subtract(segment.first, origin);
  const double distance = cross(offset, edge) / denominator;
  const double ratio = cross(offset, direction) / denominator;
  return distance >= 0.0 && ratio >= 0.0 && ratio <= 1.0
             ? distance
             : std::numeric_limits<double>::infinity();
}

double reflectCoordinate(double value, double radius, double minimum,
                         double maximum, double& velocity) {
  const double low = minimum + radius;
  const double high = maximum - radius;
  while (value < low || value > high) {
    if (value < low) {
      value = 2.0 * low - value;
      velocity = std::abs(velocity);
    } else {
      value = 2.0 * high - value;
      velocity = -std::abs(velocity);
    }
  }
  return value;
}

}  // namespace

const char* resultName(Result result) {
  switch (result) {
    case Result::Running:
      return "running";
    case Result::GoalReached:
      return "goal_reached";
    case Result::Collision:
      return "collision";
    case Result::TimedOut:
      return "timed_out";
  }
  return "unknown";
}

Pose2 composePose(const Pose2& parent, const Pose2& child) {
  const double c = std::cos(parent.yaw);
  const double s = std::sin(parent.yaw);
  return {parent.x + c * child.x - s * child.y,
          parent.y + s * child.x + c * child.y,
          normalizeAngle(parent.yaw + child.yaw)};
}

Simulation::Simulation(SimulationConfig config, Pose2 initial_pose, Vec2 goal,
                       std::vector<CircleObstacle> circles,
                       std::vector<SegmentObstacle> segments)
    : config_(std::move(config)),
      pose_(initial_pose),
      goal_(goal),
      circles_(std::move(circles)),
      segments_(std::move(segments)) {
  validate();
  minimum_clearance_ = clearance();
  if (collides()) {
    result_ = Result::Collision;
  } else if (goalDistance() <= config_.goal_tolerance) {
    result_ = Result::GoalReached;
  }
}

void Simulation::validate() const {
  if (config_.min_x >= config_.max_x || config_.min_y >= config_.max_y ||
      config_.robot_length <= 0.0 || config_.robot_width <= 0.0 ||
      config_.max_linear_speed <= 0.0 || config_.max_angular_speed <= 0.0 ||
      config_.max_linear_acceleration <= 0.0 ||
      config_.max_angular_acceleration <= 0.0 ||
      config_.command_timeout <= 0.0 || config_.goal_tolerance <= 0.0 ||
      config_.simulation_timeout <= 0.0 || config_.integration_substeps < 1) {
    throw std::invalid_argument("invalid simulation configuration");
  }
  for (const auto& circle : circles_) {
    if (circle.radius <= 0.0 ||
        2.0 * circle.radius > config_.max_x - config_.min_x ||
        2.0 * circle.radius > config_.max_y - config_.min_y ||
        circle.center.x - circle.radius < config_.min_x ||
        circle.center.x + circle.radius > config_.max_x ||
        circle.center.y - circle.radius < config_.min_y ||
        circle.center.y + circle.radius > config_.max_y) {
      throw std::invalid_argument("invalid circle obstacle");
    }
  }
  for (const auto& segment : segments_) {
    if (norm(subtract(segment.second, segment.first)) <= kEpsilon)
      throw std::invalid_argument("zero-length segment obstacle");
  }
}

void Simulation::setCommand(Twist2 command) {
  command_.linear = std::clamp(command.linear, -config_.max_linear_speed,
                               config_.max_linear_speed);
  command_.angular = std::clamp(command.angular, -config_.max_angular_speed,
                                config_.max_angular_speed);
  command_age_ = 0.0;
}

void Simulation::step(double dt) {
  if (!(dt > 0.0) || !std::isfinite(dt))
    throw std::invalid_argument("simulation step must be finite and positive");
  if (result_ != Result::Running) return;

  const double substep = dt / config_.integration_substeps;
  for (int i = 0; i < config_.integration_substeps; ++i) {
    elapsed_time_ += substep;
    command_age_ += substep;
    integrateObstacles(substep);
    integrateRobot(substep);
    minimum_clearance_ = std::min(minimum_clearance_, clearance());
    if (collides()) {
      result_ = Result::Collision;
      velocity_ = {};
      return;
    }
    if (goalDistance() <= config_.goal_tolerance) {
      result_ = Result::GoalReached;
      velocity_ = {};
      return;
    }
    if (elapsed_time_ >= config_.simulation_timeout) {
      result_ = Result::TimedOut;
      velocity_ = {};
      return;
    }
  }
}

void Simulation::integrateRobot(double dt) {
  const Twist2 target = command_age_ <= config_.command_timeout
                            ? command_
                            : Twist2{};
  velocity_.linear = moveToward(velocity_.linear, target.linear,
                                config_.max_linear_acceleration * dt);
  velocity_.angular = moveToward(velocity_.angular, target.angular,
                                 config_.max_angular_acceleration * dt);

  const double rotation = velocity_.angular * dt;
  if (std::abs(velocity_.angular) <= kEpsilon) {
    pose_.x += velocity_.linear * std::cos(pose_.yaw) * dt;
    pose_.y += velocity_.linear * std::sin(pose_.yaw) * dt;
  } else {
    const double radius = velocity_.linear / velocity_.angular;
    pose_.x += radius * (std::sin(pose_.yaw + rotation) - std::sin(pose_.yaw));
    pose_.y -= radius * (std::cos(pose_.yaw + rotation) - std::cos(pose_.yaw));
  }
  pose_.yaw = normalizeAngle(pose_.yaw + rotation);
  path_length_ += std::abs(velocity_.linear) * dt;
}

void Simulation::integrateObstacles(double dt) {
  for (auto& circle : circles_) {
    circle.center.x += circle.velocity.x * dt;
    circle.center.y += circle.velocity.y * dt;
    circle.center.x = reflectCoordinate(circle.center.x, circle.radius,
                                        config_.min_x, config_.max_x,
                                        circle.velocity.x);
    circle.center.y = reflectCoordinate(circle.center.y, circle.radius,
                                        config_.min_y, config_.max_y,
                                        circle.velocity.y);
  }
}

double Simulation::goalDistance() const {
  return std::hypot(pose_.x - goal_.x, pose_.y - goal_.y);
}

double Simulation::clearance() const {
  double best = std::numeric_limits<double>::infinity();
  for (const auto& circle : circles_) {
    best = std::min(best, orientedBoxCircleClearance(
                              pose_, config_.robot_length, config_.robot_width,
                              circle));
  }

  const auto corners =
      robotCorners(pose_, config_.robot_length, config_.robot_width);
  const auto edges = robotEdges(corners);
  for (const auto& segment : segments_) {
    if (pointInsideRobot(pose_, config_.robot_length, config_.robot_width,
                         segment.first) ||
        pointInsideRobot(pose_, config_.robot_length, config_.robot_width,
                         segment.second)) {
      return 0.0;
    }
    for (const auto& edge : edges)
      best = std::min(best, segmentDistance(edge, segment));
  }
  for (const auto& corner : corners) {
    best = std::min({best, corner.x - config_.min_x,
                     config_.max_x - corner.x, corner.y - config_.min_y,
                     config_.max_y - corner.y});
  }
  return best;
}

bool Simulation::collides() const { return clearance() <= 0.0; }

RayHit Simulation::raycast(const Pose2& sensor_pose, double local_angle,
                           double range_min, double range_max) const {
  if (range_min < 0.0 || range_max <= range_min)
    throw std::invalid_argument("invalid ray range");
  const double angle = sensor_pose.yaw + local_angle;
  const Vec2 origin{sensor_pose.x, sensor_pose.y};
  const Vec2 direction{std::cos(angle), std::sin(angle)};
  RayHit best;

  auto consider = [&](double distance, Vec2 velocity, bool dynamic) {
    if (distance < best.range && distance <= range_max) {
      best.hit = true;
      best.range = std::max(distance, range_min);
      best.point = {origin.x + best.range * direction.x,
                    origin.y + best.range * direction.y};
      best.velocity = velocity;
      best.dynamic = dynamic;
    }
  };

  for (const auto& circle : circles_) {
    consider(rayCircle(origin, direction, circle), circle.velocity,
             std::hypot(circle.velocity.x, circle.velocity.y) > kEpsilon);
  }
  for (const auto& segment : segments_)
    consider(raySegment(origin, direction, segment), {}, false);

  const SegmentObstacle boundaries[] = {
      {{config_.min_x, config_.min_y}, {config_.max_x, config_.min_y}},
      {{config_.max_x, config_.min_y}, {config_.max_x, config_.max_y}},
      {{config_.max_x, config_.max_y}, {config_.min_x, config_.max_y}},
      {{config_.min_x, config_.max_y}, {config_.min_x, config_.min_y}}};
  for (const auto& boundary : boundaries)
    consider(raySegment(origin, direction, boundary), {}, false);
  return best;
}

}  // namespace neupan_sim
