/*
 * neupan_ros: a minimal reference global planner for NeuPAN demos.
 *
 * Reads a latched nav_msgs/OccupancyGrid (map), inflates obstacles by the
 * robot radius, runs 8-connected A* from the current TF pose to an RViz
 * "2D Nav Goal" (goal_pose), simplifies the result and publishes it as a
 * nav_msgs/Path on initial_path -- the reference line NeuPAN tracks.
 *
 * NeuPAN does local obstacle avoidance from the live scan, so this planner
 * only needs to be topologically valid: a light robot-radius inflation, no
 * costmap machinery. Bring your own global planner for production.
 *
 * This program is free software under the GNU General Public License v3+.
 */

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {

constexpr double kSqrt2 = 1.41421356237;

}  // namespace

class AStarGlobalNode : public rclcpp::Node {
 public:
  AStarGlobalNode() : Node("astar_global_node") {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    // Footprint circumradius plus room for localization drift.
    robot_radius_ = declare_parameter<double>("robot_radius", 0.45);
    allow_unknown_ = declare_parameter<bool>("allow_unknown", false);
    // Douglas-Peucker tolerance (m): larger -> fewer, coarser waypoints.
    simplify_tol_ = declare_parameter<double>("simplify_tolerance", 0.15);
    // Decision layers republish the same goal every tick; without this each one replans.
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.3);

    if (base_frame_.empty())
      throw std::runtime_error("base_frame must not be empty");

    path_pub_ = create_publisher<nav_msgs::msg::Path>("initial_path", 10);

    rclcpp::QoS map_qos(1);
    map_qos.transient_local().reliable();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", map_qos,
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
          onMap(*msg);
        });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
          onGoal(*msg);
        });

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(get_logger(), "astar_global_node ready (robot_radius %.2f m)",
                robot_radius_);
  }

 private:
  // Inflate obstacles/unknowns by robot_radius via multi-source BFS (Chebyshev
  // dilation), so A* only walks cells the robot body can occupy.
  void onMap(const nav_msgs::msg::OccupancyGrid& map) {
    if (map.header.frame_id.empty()) {
      RCLCPP_ERROR(get_logger(), "rejecting map: header.frame_id is empty");
      return;
    }
    if (map.info.resolution <= 0.0 || map.info.width == 0 ||
        map.info.height == 0 ||
        map.data.size() !=
            static_cast<size_t>(map.info.width) * map.info.height) {
      RCLCPP_ERROR(get_logger(), "rejecting malformed occupancy grid");
      return;
    }
    if (!map_frame_.empty() && map_frame_ != map.header.frame_id) {
      RCLCPP_INFO(get_logger(), "map frame changed from '%s' to '%s'",
                  map_frame_.c_str(), map.header.frame_id.c_str());
      have_path_ = false;
    }
    map_frame_ = map.header.frame_id;
    map_ = map;
    w_ = map.info.width;
    h_ = map.info.height;
    res_ = map.info.resolution;
    ox_ = map.info.origin.position.x;
    oy_ = map.info.origin.position.y;
    const double origin_yaw = tf2YawFromQuat(map.info.origin.orientation);
    origin_cos_ = std::cos(origin_yaw);
    origin_sin_ = std::sin(origin_yaw);

    const int radius_cells = std::max(0, static_cast<int>(
                                             std::round(robot_radius_ / res_)));
    blocked_.assign(static_cast<size_t>(w_) * h_, 0);
    std::vector<int> dist(static_cast<size_t>(w_) * h_, -1);
    std::queue<int> q;
    for (size_t i = 0; i < map.data.size(); ++i) {
      const int8_t v = map.data[i];
      const bool obstacle = v >= 50 || (v < 0 && !allow_unknown_);
      if (obstacle) {
        blocked_[i] = 1;
        dist[i] = 0;
        q.push(static_cast<int>(i));
      }
    }
    while (!q.empty()) {
      const int cur = q.front();
      q.pop();
      if (dist[cur] >= radius_cells) continue;
      const int cx = cur % w_, cy = cur / w_;
      for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) continue;
          const int nx = cx + dx, ny = cy + dy;
          if (nx < 0 || ny < 0 || nx >= w_ || ny >= h_) continue;
          const int ni = ny * w_ + nx;
          if (dist[ni] != -1) continue;
          dist[ni] = dist[cur] + 1;
          blocked_[ni] = 1;
          q.push(ni);
        }
    }
    have_map_ = true;
    RCLCPP_INFO(get_logger(), "map received %dx%d @ %.3f m, inflated %d cells",
                w_, h_, res_, radius_cells);
  }

  bool worldToCell(double wx, double wy, int& cx, int& cy) const {
    const double dx = wx - ox_;
    const double dy = wy - oy_;
    const double mx = origin_cos_ * dx + origin_sin_ * dy;
    const double my = -origin_sin_ * dx + origin_cos_ * dy;
    cx = static_cast<int>(std::floor(mx / res_));
    cy = static_cast<int>(std::floor(my / res_));
    return cx >= 0 && cy >= 0 && cx < w_ && cy < h_;
  }

  std::array<double, 2> cellToWorld(int cx, int cy) const {
    const double mx = (cx + 0.5) * res_;
    const double my = (cy + 0.5) * res_;
    return {ox_ + origin_cos_ * mx - origin_sin_ * my,
            oy_ + origin_sin_ * mx + origin_cos_ * my};
  }

  void onGoal(const geometry_msgs::msg::PoseStamped& goal) {
    if (goal.header.frame_id.empty()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "rejecting goal_pose: header.frame_id is empty");
      return;
    }
    if (!have_map_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "no map yet, ignoring goal");
      return;
    }
    const auto goal_in_map = transformGoalToMap(goal);
    if (!goal_in_map) return;
    if (have_path_ &&
        std::hypot(goal_in_map->pose.position.x - last_goal_x_,
                   goal_in_map->pose.position.y - last_goal_y_) <
            goal_tolerance_)
      return;
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(map_frame_, base_frame_,
                                       tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(get_logger(), "no tf %s <- %s: %s", map_frame_.c_str(),
                  base_frame_.c_str(), ex.what());
      return;
    }

    int sx, sy, gx, gy;
    if (!worldToCell(tf.transform.translation.x, tf.transform.translation.y,
                     sx, sy) ||
        !worldToCell(goal_in_map->pose.position.x,
                     goal_in_map->pose.position.y, gx, gy)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "start or goal outside the map");
      return;
    }
    // Nudge endpoints out of inflated cells (robot/goal hugging a wall).
    snapToFree(sx, sy);
    const int gx_asked = gx, gy_asked = gy;
    snapToFree(gx, gy);
    const bool goal_snapped = gx != gx_asked || gy != gy_asked;

    const std::vector<int> cells = astar(sy * w_ + sx, gy * w_ + gx);
    if (cells.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "A* found no path to goal, keeping the previous one");
      return;
    }

    std::vector<std::array<double, 2>> pts;
    pts.reserve(cells.size());
    for (int c : cells)
      pts.push_back(cellToWorld(c % w_, c / w_));
    pts = simplify(pts, simplify_tol_);

    // Cell centres stop short of the goal; restore it only if it was not snapped.
    if (!goal_snapped && !pts.empty())
      pts.back() = {goal_in_map->pose.position.x,
                    goal_in_map->pose.position.y};

    publishPath(pts, *goal_in_map);
    last_goal_x_ = goal_in_map->pose.position.x;
    last_goal_y_ = goal_in_map->pose.position.y;
    have_path_ = true;
    RCLCPP_INFO(get_logger(), "published global path: %zu cells -> %zu waypoints",
                cells.size(), pts.size());
  }

  std::optional<geometry_msgs::msg::PoseStamped> transformGoalToMap(
      const geometry_msgs::msg::PoseStamped& goal) {
    if (goal.header.frame_id == map_frame_) return goal;

    try {
      const auto tf = tf_buffer_->lookupTransform(
          map_frame_, goal.header.frame_id, tf2::TimePointZero);
      const double yaw = tf2YawFromQuat(tf.transform.rotation);
      const double c = std::cos(yaw), s = std::sin(yaw);
      geometry_msgs::msg::PoseStamped transformed = goal;
      transformed.header.frame_id = map_frame_;
      transformed.header.stamp = now();
      transformed.pose.position.x =
          tf.transform.translation.x + c * goal.pose.position.x -
          s * goal.pose.position.y;
      transformed.pose.position.y =
          tf.transform.translation.y + s * goal.pose.position.x +
          c * goal.pose.position.y;
      const double goal_yaw = yaw + tf2YawFromQuat(goal.pose.orientation);
      transformed.pose.orientation.x = 0.0;
      transformed.pose.orientation.y = 0.0;
      transformed.pose.orientation.z = std::sin(goal_yaw / 2.0);
      transformed.pose.orientation.w = std::cos(goal_yaw / 2.0);
      return transformed;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "no tf %s <- %s for goal_pose: %s", map_frame_.c_str(),
          goal.header.frame_id.c_str(), ex.what());
      return std::nullopt;
    }
  }

  void snapToFree(int& cx, int& cy) const {
    if (!blocked_[cy * w_ + cx]) return;
    for (int r = 1; r < std::max(w_, h_); ++r) {
      for (int dy = -r; dy <= r; ++dy)
        for (int dx = -r; dx <= r; ++dx) {
          if (std::abs(dx) != r && std::abs(dy) != r) continue;  // ring only
          const int nx = cx + dx, ny = cy + dy;
          if (nx < 0 || ny < 0 || nx >= w_ || ny >= h_) continue;
          if (!blocked_[ny * w_ + nx]) {
            cx = nx;
            cy = ny;
            return;
          }
        }
    }
  }

  std::vector<int> astar(int start, int goal) const {
    const size_t n = static_cast<size_t>(w_) * h_;
    std::vector<float> g(n, std::numeric_limits<float>::infinity());
    std::vector<int> came(n, -1);
    using PQ = std::pair<float, int>;  // (f, idx)
    std::priority_queue<PQ, std::vector<PQ>, std::greater<PQ>> open;

    const int gxc = goal % w_, gyc = goal / w_;
    auto heur = [&](int i) {
      const double dx = std::abs(i % w_ - gxc), dy = std::abs(i / w_ - gyc);
      return static_cast<float>((dx + dy) + (kSqrt2 - 2.0) * std::min(dx, dy));
    };

    g[start] = 0.0f;
    open.push({heur(start), start});
    while (!open.empty()) {
      const auto [f, cur] = open.top();
      open.pop();
      if (cur == goal) break;
      if (f > g[cur] + heur(cur) + 1e-3f) continue;  // stale entry
      const int cx = cur % w_, cy = cur / w_;
      for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) continue;
          const int nx = cx + dx, ny = cy + dy;
          if (nx < 0 || ny < 0 || nx >= w_ || ny >= h_) continue;
          const int ni = ny * w_ + nx;
          if (blocked_[ni]) continue;
          // no diagonal corner cutting
          if (dx != 0 && dy != 0 &&
              (blocked_[cy * w_ + nx] || blocked_[ny * w_ + cx]))
            continue;
          const float step = (dx != 0 && dy != 0) ? kSqrt2 : 1.0f;
          const float ng = g[cur] + step;
          if (ng < g[ni]) {
            g[ni] = ng;
            came[ni] = cur;
            open.push({ng + heur(ni), ni});
          }
        }
    }

    std::vector<int> path;
    if (came[goal] == -1 && start != goal) return path;
    for (int c = goal; c != -1; c = came[c]) path.push_back(c);
    std::reverse(path.begin(), path.end());
    return path;
  }

  // Iterative Douglas-Peucker on the polyline.
  static std::vector<std::array<double, 2>> simplify(
      const std::vector<std::array<double, 2>>& pts, double tol) {
    if (pts.size() < 3) return pts;
    const int last = static_cast<int>(pts.size()) - 1;
    std::vector<char> keep(pts.size(), 0);
    keep[0] = 1;
    keep[last] = 1;
    std::vector<std::pair<int, int>> stack{{0, last}};
    while (!stack.empty()) {
      const auto [a, b] = stack.back();
      stack.pop_back();
      double dmax = 0.0;
      int idx = -1;
      const double ax = pts[a][0], ay = pts[a][1];
      const double bx = pts[b][0], by = pts[b][1];
      const double len = std::hypot(bx - ax, by - ay);
      for (int i = a + 1; i < b; ++i) {
        double d;
        if (len < 1e-9) {
          d = std::hypot(pts[i][0] - ax, pts[i][1] - ay);
        } else {
          d = std::abs((bx - ax) * (ay - pts[i][1]) -
                       (ax - pts[i][0]) * (by - ay)) /
              len;
        }
        if (d > dmax) {
          dmax = d;
          idx = i;
        }
      }
      if (dmax > tol && idx != -1) {
        keep[idx] = 1;
        stack.push_back({a, idx});
        stack.push_back({idx, b});
      }
    }
    std::vector<std::array<double, 2>> out;
    for (size_t i = 0; i < pts.size(); ++i)
      if (keep[i]) out.push_back(pts[i]);
    return out;
  }

  void publishPath(const std::vector<std::array<double, 2>>& pts,
                   const geometry_msgs::msg::PoseStamped& goal) {
    nav_msgs::msg::Path path;
    path.header.frame_id = map_frame_;
    path.header.stamp = now();
    for (size_t i = 0; i < pts.size(); ++i) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = pts[i][0];
      ps.pose.position.y = pts[i][1];
      double yaw;
      if (i + 1 < pts.size())
        yaw = std::atan2(pts[i + 1][1] - pts[i][1], pts[i + 1][0] - pts[i][0]);
      else
        yaw = tf2YawFromQuat(goal.pose.orientation);
      ps.pose.orientation.z = std::sin(yaw / 2.0);
      ps.pose.orientation.w = std::cos(yaw / 2.0);
      path.poses.push_back(ps);
    }
    path_pub_->publish(path);
  }

  static double tf2YawFromQuat(const geometry_msgs::msg::Quaternion& q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.z * q.z + q.y * q.y));
  }

  std::string map_frame_, base_frame_;
  double robot_radius_, simplify_tol_, goal_tolerance_;
  bool allow_unknown_ = false;

  double last_goal_x_ = 0.0, last_goal_y_ = 0.0;
  bool have_path_ = false;

  nav_msgs::msg::OccupancyGrid map_;
  std::vector<uint8_t> blocked_;
  int w_ = 0, h_ = 0;
  double res_ = 0.0, ox_ = 0.0, oy_ = 0.0;
  double origin_cos_ = 1.0, origin_sin_ = 0.0;
  bool have_map_ = false;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AStarGlobalNode>());
  rclcpp::shutdown();
  return 0;
}
