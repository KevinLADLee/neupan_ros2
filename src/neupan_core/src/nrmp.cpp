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

#include "neupan/nrmp.hpp"

#include <algorithm>
#include <cstdlib>
#include <stdexcept>

namespace neupan {

namespace {
// real inf in the bounds makes the v1.0 duality-gap
// termination check produce NaN and the solver never reports Solved.
constexpr double kInf = OSQP_INFTY;
}

NRMP::NRMP(const Robot& robot, const NRMPParams& params)
    : robot_(robot),
      p_(params),
      no_obs_(params.max_num == 0),
      T_(robot.T),
      M_(params.max_num) {
  if (M_ < 0) throw std::invalid_argument("nrmp: max_num must be >= 0");
  off_u_ = 3 * (T_ + 1);
  off_d_ = off_u_ + 2 * T_;
  off_e_ = off_d_ + (no_obs_ ? 0 : T_);
  nz_ = off_e_ + M_ * T_;

  // rows: init eq (3) | dynamics (3T) | speed (2T) | acce (2(T-1)) |
  //       d box (T) | avoidance (M*T) | e >= 0 (M*T)
  nc_ = 3 + 3 * T_ + 2 * T_ + 2 * (T_ - 1) +
        (no_obs_ ? 0 : T_ + 2 * M_ * T_);

  P_.resize(nz_, nz_);
  A_.resize(nc_, nz_);
  q_.resize(nz_);
  lb_.resize(nc_);
  ub_.resize(nc_);
}

NRMP::~NRMP() = default;
NRMP::NRMP(NRMP&&) noexcept = default;
NRMP& NRMP::operator=(NRMP&&) noexcept = default;

void NRMP::buildFaFb(const std::vector<Mat>& mu_list,
                     const std::vector<Mat2X>& lam_list,
                     const std::vector<Mat2X>& point_list,
                     std::vector<Mat>& fa_list, std::vector<Vec>& fb_list) const {
  if (no_obs_) {
    fa_list.clear();
    fb_list.clear();
    return;
  }

  fa_list.assign(T_, Mat::Zero(M_, 2));
  fb_list.assign(T_, Vec::Zero(M_));

  if (mu_list.empty()) return;

  for (int t = 0; t < T_; ++t) {
    const Mat& mu = mu_list[t + 1];
    const Mat2X& lam = lam_list[t + 1];
    const Mat2X& pts = point_list[t + 1];

    const int pn = std::min<int>(static_cast<int>(mu.cols()), M_);
    for (int k = 0; k < pn; ++k) {
      fa_list[t].row(k) = lam.col(k).transpose();
      fb_list[t](k) = lam.col(k).dot(pts.col(k)) + mu.col(k).dot(robot_.h);
    }
    // Upstream pads missing rows with the closest point's constraint.
    for (int k = pn; k < M_; ++k) {
      fa_list[t].row(k) = fa_list[t].row(0);
      fb_list[t](k) = fb_list[t](0);
    }
  }
}

void NRMP::assemble(const Mat3X& nom_s, const Mat2X& nom_u, const Mat3X& ref_s,
                    const Vec& ref_us, const std::vector<Mat>& fa_list,
                    const std::vector<Vec>& fb_list) {
  const Vec3& qs = p_.q_s;

  // ---- Hessian P (diagonal) and gradient q ----
  // cvxpy objective J(z) = z' Q z + c' z; OSQP form 0.5 z' P z + q' z with
  // P = 2 Q. Terms: ||q_s .* s - q_s .* ref_s||^2, ||p_u u_v - p_u ref_us||^2,
  // 0.5 bk ||s - nom_s||^2, -eta sum(d), 0.5 ro_obs ||e||^2.
  std::vector<Eigen::Triplet<double>> pTrip;
  if (!matrix_structure_initialized_) pTrip.reserve(nz_);
  const auto setP = [&](int row, int col, double value) {
    if (matrix_structure_initialized_)
      P_.coeffRef(row, col) = value;
    else
      pTrip.emplace_back(row, col, value);
  };
  q_.setZero();

  for (int t = 0; t <= T_; ++t) {
    for (int r = 0; r < 3; ++r) {
      const int j = 3 * t + r;
      setP(j, j, 2.0 * qs(r) * qs(r) + p_.bk);
      const double gamma_a = qs(r) * ref_s(r, t);
      q_(j) = -2.0 * qs(r) * gamma_a - p_.bk * nom_s(r, t);
    }
  }
  for (int t = 0; t < T_; ++t) {
    const int jv = off_u_ + 2 * t;
    setP(jv, jv, 2.0 * p_.p_u * p_.p_u);
    setP(jv + 1, jv + 1, 0.0);
    q_(jv) = -2.0 * p_.p_u * (p_.p_u * ref_us(t));
  }
  if (!no_obs_)
    for (int t = 0; t < T_; ++t) {
      const int jd = off_d_ + t;
      setP(jd, jd, 0.0);
      q_(jd) = -p_.eta;
    }
  for (int j = off_e_; j < nz_; ++j) setP(j, j, p_.ro_obs);

  if (!matrix_structure_initialized_) {
    P_.setZero();
    P_.setFromTriplets(pTrip.begin(), pTrip.end());
    P_.makeCompressed();
  }

  // ---- Constraints ----
  std::vector<Eigen::Triplet<double>> aTrip;
  if (!matrix_structure_initialized_)
    aTrip.reserve(20 * T_ + 6 * M_ * T_);
  const auto setA = [&](int row, int col, double value) {
    if (matrix_structure_initialized_)
      A_.coeffRef(row, col) = value;
    else
      aTrip.emplace_back(row, col, value);
  };
  int row = 0;

  // init: s_0 == nom_s[:, 0]
  for (int r = 0; r < 3; ++r) {
    setA(row, r, 1.0);
    lb_(row) = ub_(row) = nom_s(r, 0);
    ++row;
  }

  // dynamics: A_t s_t + B_t u_t - s_{t+1} == -C_t
  for (int t = 0; t < T_; ++t) {
    Mat33 A;
    Mat32 B;
    Vec3 C;
    robot_.linearize(nom_s.col(t), nom_u.col(t), A, B, C);

    for (int r = 0; r < 3; ++r) {
      // Keep a fixed sparsity pattern: insert all entries that can ever be
      // nonzero for the diff model, even when currently zero.
      setA(row, 3 * t + r, A(r, r));
      if (r < 2) setA(row, 3 * t + 2, A(r, 2));
      if (r < 2)
        setA(row, off_u_ + 2 * t, B(r, 0));
      else
        setA(row, off_u_ + 2 * t + 1, B(r, 1));
      setA(row, 3 * (t + 1) + r, -1.0);
      lb_(row) = ub_(row) = -C(r);
      ++row;
    }
  }

  // speed box: |u| <= max_speed
  for (int t = 0; t < T_; ++t) {
    for (int r = 0; r < 2; ++r) {
      setA(row, off_u_ + 2 * t + r, 1.0);
      lb_(row) = -robot_.max_speed(r);
      ub_(row) = robot_.max_speed(r);
      ++row;
    }
  }

  // acceleration box: |u_{t+1} - u_t| <= max_acce * dt
  for (int t = 0; t + 1 < T_; ++t) {
    for (int r = 0; r < 2; ++r) {
      setA(row, off_u_ + 2 * (t + 1) + r, 1.0);
      setA(row, off_u_ + 2 * t + r, -1.0);
      lb_(row) = -robot_.acce_bound(r);
      ub_(row) = robot_.acce_bound(r);
      ++row;
    }
  }

  // distance box: d_min <= d <= d_max (not part of the no-obstacle problem)
  if (!no_obs_)
    for (int t = 0; t < T_; ++t) {
      setA(row, off_d_ + t, 1.0);
      // indep_dis is declared nonnegative in CVXPY in addition to d >= d_min.
      lb_(row) = std::max(0.0, p_.d_min);
      ub_(row) = p_.d_max;
      ++row;
    }

  // avoidance: fa . s_xy(t+1) - d_t + e_{t,k} >= fb
  for (int t = 0; t < T_; ++t) {
    for (int k = 0; k < M_; ++k) {
      const double fax = fa_list.empty() ? 0.0 : fa_list[t](k, 0);
      const double fay = fa_list.empty() ? 0.0 : fa_list[t](k, 1);
      const double fb = fb_list.empty() ? 0.0 : fb_list[t](k);
      setA(row, 3 * (t + 1) + 0, fax);
      setA(row, 3 * (t + 1) + 1, fay);
      setA(row, off_d_ + t, -1.0);
      setA(row, off_e_ + M_ * t + k, 1.0);
      lb_(row) = fb;
      ub_(row) = kInf;
      ++row;
    }
  }

  // e >= 0
  for (int j = 0; j < M_ * T_; ++j) {
    setA(row, off_e_ + j, 1.0);
    lb_(row) = 0.0;
    ub_(row) = kInf;
    ++row;
  }

  if (row != nc_) throw std::logic_error("nrmp: constraint row mismatch");

  if (!matrix_structure_initialized_) {
    A_.setZero();
    A_.setFromTriplets(aTrip.begin(), aTrip.end());
    A_.makeCompressed();
    p_nonzeros_ = P_.nonZeros();
    a_nonzeros_ = A_.nonZeros();
    matrix_structure_initialized_ = true;
  } else if (!P_.isCompressed() || !A_.isCompressed() ||
             P_.nonZeros() != p_nonzeros_ || A_.nonZeros() != a_nonzeros_) {
    throw std::logic_error("nrmp: sparse matrix pattern changed during update");
  }
}

NRMP::Result NRMP::solve(const Mat3X& nom_s, const Mat2X& nom_u,
                         const Mat3X& ref_s, const Vec& ref_us,
                         const std::vector<Mat>& fa_list,
                         const std::vector<Vec>& fb_list) {
  assemble(nom_s, nom_u, ref_s, ref_us, fa_list, fb_list);

  if (!initialized_) {
    solver_ = std::make_unique<OsqpEigen::Solver>();
    solver_->settings()->setVerbosity(std::getenv("NEUPAN_OSQP_VERBOSE") != nullptr);
    solver_->settings()->setWarmStart(true);
    solver_->settings()->setPolish(true);
    solver_->settings()->setAbsoluteTolerance(1e-5);
    solver_->settings()->setRelativeTolerance(1e-5);
    solver_->settings()->setMaxIteration(10000);
    solver_->data()->setNumberOfVariables(nz_);
    solver_->data()->setNumberOfConstraints(nc_);
    if (!solver_->data()->setHessianMatrix(P_) ||
        !solver_->data()->setGradient(q_) ||
        !solver_->data()->setLinearConstraintsMatrix(A_) ||
        !solver_->data()->setLowerBound(lb_) ||
        !solver_->data()->setUpperBound(ub_) || !solver_->initSolver())
      throw std::runtime_error("nrmp: OSQP init failed");
    initialized_ = true;
  } else {
    // OsqpEigen 0.11.2 compares updates with its original Data snapshot. In a
    // multi-frame planner that can miss coefficients which return to their
    // initial value. The solver stack is pinned to OSQP 1.0.0, so update every
    // numeric A value through its stable v1 API while preserving the fixed CSC
    // pattern and warm start.
    if (osqp_update_data_mat(solver_->solver().get(), nullptr, nullptr, 0,
                             A_.valuePtr(), nullptr, A_.nonZeros()) != 0 ||
        !solver_->updateGradient(q_) || !solver_->updateBounds(lb_, ub_))
      throw std::runtime_error("nrmp: OSQP update failed");
  }

  Result res;
  const bool no_error =
      solver_->solveProblem() == OsqpEigen::ErrorExitFlag::NoError;
  res.status = static_cast<int>(solver_->getStatus());
  res.success = no_error && solver_->getStatus() == OsqpEigen::Status::Solved;

  const Vec z = solver_->getSolution();
  res.s = Eigen::Map<const Mat>(z.data(), 3, T_ + 1);
  res.u = Eigen::Map<const Mat>(z.data() + off_u_, 2, T_);
  res.d = no_obs_ ? Vec{} : z.segment(off_d_, T_);
  res.objective =
      objective(res.s, res.u, res.d, nom_s, ref_s, ref_us, fa_list, fb_list);
  return res;
}

double NRMP::objective(const Mat3X& s, const Mat2X& u, const Vec& d,
                       const Mat3X& nom_s, const Mat3X& ref_s,
                       const Vec& ref_us, const std::vector<Mat>& fa_list,
                       const std::vector<Vec>& fb_list) const {
  double J = 0.0;
  for (int t = 0; t <= T_; ++t)
    for (int r = 0; r < 3; ++r) {
      const double diff = p_.q_s(r) * s(r, t) - p_.q_s(r) * ref_s(r, t);
      J += diff * diff;
    }
  for (int t = 0; t < T_; ++t) {
    const double diff = p_.p_u * u(0, t) - p_.p_u * ref_us(t);
    J += diff * diff;
  }
  J += 0.5 * p_.bk * (s - nom_s).squaredNorm();
  if (!no_obs_) J -= p_.eta * d.sum();
  for (int t = 0; t < T_; ++t)
    for (int k = 0; k < M_; ++k) {
      const double fax = fa_list.empty() ? 0.0 : fa_list[t](k, 0);
      const double fay = fa_list.empty() ? 0.0 : fa_list[t](k, 1);
      const double fb = fb_list.empty() ? 0.0 : fb_list[t](k);
      const double I = fax * s(0, t + 1) + fay * s(1, t + 1) - fb - d(t);
      const double neg = std::max(-I, 0.0);
      J += 0.5 * p_.ro_obs * neg * neg;
    }
  return J;
}

}  // namespace neupan
