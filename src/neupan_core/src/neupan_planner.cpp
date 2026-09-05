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

#include "neupan/neupan_planner.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>

namespace neupan {

namespace {

Robot makeRobot(const NeuPANPlanner::Config& c) {
  return Robot::diffRectangle(c.receding, c.step_time, c.max_speed,
                              c.max_acce, c.length, c.width, c.wheelbase);
}

// cfg_ is the first member, so this runs before the robot and the blocks.
const NeuPANPlanner::Config& validated(const NeuPANPlanner::Config& c) {
  NeuPANPlanner::validate(c);
  return c;
}

}  // namespace

void NeuPANPlanner::validate(const Config& c) {
  const auto fail = [](const std::string& what) {
    throw std::runtime_error("neupan config: " + what);
  };
  const NRMPParams& a = c.adjust;

  if (c.receding < 1 || c.step_time <= 0.0)
    fail("receding must be >= 1 and step_time > 0");

  if (a.max_num < 0 || c.pan.dune_max_num < 0)
    fail("nrmp_max_num and dune_max_num must be >= 0");

  if (c.pan.iter_num < 1) fail("iter_num must be >= 1");

  if (a.d_min < 0.0)
    fail("d_min is negative (" + std::to_string(a.d_min) +
         "); a negative clearance lets the plan penetrate obstacles");

  if (a.d_max < a.d_min)
    fail("d_max (" + std::to_string(a.d_max) + ") is below d_min (" +
         std::to_string(a.d_min) + "), leaving the clearance box empty");

  // Shipped as d = 0 with eta = 15 for a whole season, silently.
  if (a.eta > 0.0 && a.d_max <= 0.0)
    fail("eta > 0 rewards clearance but d_max is " + std::to_string(a.d_max) +
         "; that pins the clearance variable to zero, so eta buys nothing and "
         "only the ro_obs contact penalty repels obstacles (upstream uses "
         "d_min 0.1, d_max 1.0)");

  if (c.collision_threshold <= 0.0)
    fail("collision_threshold is " + std::to_string(c.collision_threshold) +
         "; stop would then require min_distance < 0, i.e. the footprint "
         "already overlapping, which is unrecoverable");
}

NeuPANPlanner::NeuPANPlanner(const Config& config)
    : cfg_(validated(config)),
      robot_(makeRobot(config)),
      ipath_(robot_, config.ref_speed, config.ipath),
      pan_(robot_, config.dune_checkpoint, config.adjust, config.pan),
      ref_speed_(config.ref_speed),
      cur_vel_array_(Mat2X::Zero(2, config.receding)) {}

NeuPANPlanner NeuPANPlanner::fromYaml(const std::string& yaml_path,
                                      const std::string& checkpoint_override) {
  const YAML::Node y = YAML::LoadFile(yaml_path);
  Config c;

  c.receding = y["receding"].as<int>(c.receding);
  c.step_time = y["step_time"].as<double>(c.step_time);
  c.ref_speed = y["ref_speed"].as<double>(c.ref_speed);
  c.collision_threshold =
      y["collision_threshold"].as<double>(c.collision_threshold);

  if (const auto r = y["robot"]) {
    const auto kin = r["kinematics"].as<std::string>("diff");
    if (kin != "diff")
      throw std::runtime_error("neupan_cpp currently supports only 'diff' "
                               "kinematics (got '" + kin + "')");
    if (r["max_speed"]) {
      const auto v = r["max_speed"].as<std::vector<double>>();
      c.max_speed = Vec2(v.at(0), v.at(1));
    }
    if (r["max_acce"]) {
      const auto v = r["max_acce"].as<std::vector<double>>();
      c.max_acce = Vec2(v.at(0), v.at(1));
    }
    c.length = r["length"].as<double>(c.length);
    c.width = r["width"].as<double>(c.width);
    c.wheelbase = r["wheelbase"].as<double>(0.0);
  }

  if (const auto ip = y["ipath"]) {
    const auto style = ip["curve_style"].as<std::string>("line");
    if (style != "line")
      throw std::runtime_error("neupan_cpp currently supports only the "
                               "'line' curve_style (got '" + style + "')");
    if (ip["waypoints"])
      for (const auto& w : ip["waypoints"]) {
        const auto v = w.as<std::vector<double>>();
        c.ipath.waypoints.emplace_back(v.at(0), v.at(1), v.at(2));
      }
    c.ipath.loop = ip["loop"].as<bool>(false);
    c.ipath.interval = ip["interval"].as<double>(-1.0);
    c.ipath.arrive_threshold =
        ip["arrive_threshold"].as<double>(c.ipath.arrive_threshold);
    c.ipath.close_threshold =
        ip["close_threshold"].as<double>(c.ipath.close_threshold);
    c.ipath.ind_range = ip["ind_range"].as<int>(c.ipath.ind_range);
    c.ipath.arrive_index_threshold =
        ip["arrive_index_threshold"].as<int>(c.ipath.arrive_index_threshold);
  }

  if (const auto p = y["pan"]) {
    c.pan.iter_num = p["iter_num"].as<int>(c.pan.iter_num);
    c.pan.dune_max_num = p["dune_max_num"].as<int>(c.pan.dune_max_num);
    c.pan.iter_threshold = p["iter_threshold"].as<double>(c.pan.iter_threshold);
    c.adjust.max_num = p["nrmp_max_num"].as<int>(c.adjust.max_num);
    c.dune_checkpoint = p["dune_checkpoint"].as<std::string>("");
  }

  if (const auto a = y["adjust"]) {
    if (a["q_s"]) {
      if (a["q_s"].IsSequence()) {
        const auto v = a["q_s"].as<std::vector<double>>();
        c.adjust.q_s = Vec3(v.at(0), v.at(1), v.at(2));
      } else {
        c.adjust.q_s = Vec3::Constant(a["q_s"].as<double>());
      }
    }
    c.adjust.p_u = a["p_u"].as<double>(c.adjust.p_u);
    c.adjust.eta = a["eta"].as<double>(c.adjust.eta);
    c.adjust.d_max = a["d_max"].as<double>(c.adjust.d_max);
    c.adjust.d_min = a["d_min"].as<double>(c.adjust.d_min);
    c.adjust.ro_obs = a["ro_obs"].as<double>(c.adjust.ro_obs);
    c.adjust.bk = a["bk"].as<double>(c.adjust.bk);
  }

  if (!checkpoint_override.empty()) c.dune_checkpoint = checkpoint_override;
  if (c.adjust.max_num > 0 && c.pan.dune_max_num > 0 &&
      c.dune_checkpoint.empty())
    throw std::runtime_error("neupan_cpp: dune_checkpoint (.bin) is required");

  return NeuPANPlanner(c);
}

Vec2 NeuPANPlanner::forward(const Vec3& state, const Mat2X& points,
                            Info& info) {
  return forward(state, points, Mat2X(2, 0), info);
}

Vec2 NeuPANPlanner::forward(const Vec3& state, const Mat2X& points,
                            const Mat2X& point_velocities, Info& info) {
  info = Info{};

  if (ipath_.checkArrive(state)) {
    info.arrive = true;
    return Vec2::Zero();
  }

  const InitialPath::NomRef nom =
      ipath_.generateNomRefState(state, cur_vel_array_, ref_speed_);

  const PAN::Output out =
      pan_.forward(nom.nom_s, nom.nom_u, nom.ref_s, nom.ref_us, points,
                   point_velocities);

  cur_vel_array_ = out.opt_u;

  info.solved = out.solved;
  info.solver_status = out.solver_status;
  info.min_distance = out.min_distance;
  info.opt_s = out.opt_s;
  info.opt_u = out.opt_u;
  info.ref_s = nom.ref_s;
  info.dune_points = out.dune_points;
  info.nrmp_points = out.nrmp_points;

  if (out.min_distance < cfg_.collision_threshold) {
    info.stop = true;
    return Vec2::Zero();
  }

  // it only bounds a command that a degraded solve might otherwise let slip through to the robot.
  return out.opt_u.col(0)
      .cwiseMax(-robot_.max_speed)
      .cwiseMin(robot_.max_speed);
}

void NeuPANPlanner::reset() {
  ipath_.reset();
  pan_.reset();
  cur_vel_array_.setZero();
}

}  // namespace neupan
