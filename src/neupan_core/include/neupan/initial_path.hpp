/*
 * neupan_cpp: C++ port of the NeuPAN planner.
 *
 * Ported from NeuPAN (https://github.com/hanruihua/NeuPAN),
 * Copyright (c) 2025 Ruihua Han <hanrh@connect.hku.hk>.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version. See <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <vector>

#include "neupan/robot.hpp"
#include "neupan/types.hpp"

namespace neupan {

double wrapToPi(double rad);

// Port of neupan/blocks/initial_path.py restricted to curve_style == "line"
// (each path point is [x, y, theta, gear], gear always 1 for line paths).
class InitialPath {
 public:
  struct Options {
    std::vector<Vec3> waypoints;
    bool loop = false;
    double interval = -1.0;  // < 0: default dt * ref_speed
    double arrive_threshold = 0.1;
    double close_threshold = 0.1;
    int ind_range = 10;
    int arrive_index_threshold = 1;
  };

  // x/y/yaw use the same world frame and SI units as NeuPANPlanner::forward.
  using PathPoint = Eigen::Vector4d;  // x [m], y [m], yaw [rad], gear

  InitialPath(const Robot& robot, double ref_speed, const Options& opts);

  // neupan.forward step 1: returns true when arrived at the path end.
  // Generates the initial path from the current state on first call.
  bool checkArrive(const Vec3& state);

  struct NomRef {
    Mat3X nom_s;   // (3, T+1)
    Mat2X nom_u;   // (2, T) == cur_vel_array
    Mat3X ref_s;   // (3, T+1)
    Vec ref_us;    // (T,)
  };
  NomRef generateNomRefState(const Vec3& state, const Mat2X& cur_vel_array,
                             double ref_speed) const;

  void setIpathWithState(const Vec3& state);
  void setIpathWithWaypoints(const std::vector<Vec3>& waypoints);
  void updateInitialPathFromGoal(const Vec3& start, const Vec3& goal);
  // Port of set_initial_path: preserves points and gear, and derives interval_
  // from the average adjacent XY spacing.
  void setInitialPath(std::vector<PathPoint> path);
  // setInitialPath, but re-projects the state onto the new path to keep progress.
  void replaceInitialPath(std::vector<PathPoint> path, const Vec3& state);
  void reset();

  bool hasPath() const { return !curve_list_.empty(); }
  bool hasConfiguredWaypoints() const { return !opts_.waypoints.empty(); }
  const std::vector<PathPoint>& initialPath() const { return initial_path_; }
  double pathInterval() const { return interval_; }

 private:
  std::vector<PathPoint> generateLineCurve(
      const std::vector<Vec3>& waypoints) const;
  void setPath(std::vector<PathPoint> path);
  void splitPathWithGear();
  double closestPoint(const Vec3& state, double threshold, int ind_range);
  bool checkCurveArrive(const Vec3& state) const;
  // Returns intersection of circle (ref xy, radius=length) with the curve,
  // advancing ref_index; mirrors find_interaction_point.
  Vec3 findInteractionPoint(const Vec3& ref_state, int& ref_index,
                            double length) const;
  Vec3 motionPredict(const Vec3& state, const Vec2& vel) const;

  const std::vector<PathPoint>& curCurve() const {
    return curve_list_[curve_index_];
  }

  Robot robot_;
  int T_;
  double dt_;
  double ref_speed_;
  Options opts_;
  double interval_ = 0.0;

  std::vector<PathPoint> initial_path_;
  std::vector<std::vector<PathPoint>> curve_list_;
  int curve_index_ = 0;

 public:
  // exposed for the planner / visualization (mutated by checkArrive)
  int point_index_ = 0;
  bool arrive_flag_ = false;
};

}  // namespace neupan
