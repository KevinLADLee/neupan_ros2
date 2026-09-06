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

#include <OsqpEigen/OsqpEigen.h>

#include <memory>
#include <stdexcept>
#include <vector>

#include "neupan/robot.hpp"
#include "neupan/types.hpp"

namespace neupan {

// Adjustable weights, mirroring the NRMP / adjust kwargs in nrmp.py.
struct NRMPParams {
  int max_num = 10;     // nrmp_max_num
  Vec3 q_s = Vec3::Ones();  // scalar q_s == all three entries equal
  double p_u = 1.0;
  double eta = 10.0;
  double d_max = 1.0;
  double d_min = 0.1;
  double ro_obs = 400.0;
  double bk = 0.1;
};

// Port of neupan/blocks/nrmp.py with the cvxpy problem restated as a QP for
// OSQP. The neg()-penalty cost 0.5*ro_obs*||neg(I)||^2 is reformulated with
// slack variables e >= max(0, -I), which is exact at the optimum.
//
// With obstacles, the decision vector is
// z = [ s (3*(T+1)) | u (2*T) | d (T) | e (max_num*T) ].  When max_num is
// zero, d/e are omitted exactly as in the original CVXPY problem.
class NRMP {
 public:
  NRMP(const Robot& robot, const NRMPParams& params);
  ~NRMP();

  // The user-declared destructor suppresses the implicit move members; bring
  // them back so NRMP (and PAN/NeuPANPlanner above it) stay movable. Defined
  // in the .cpp where OsqpEigen::Solver is a complete type.
  NRMP(NRMP&&) noexcept;
  NRMP& operator=(NRMP&&) noexcept;

  struct Result {
    Mat3X s;           // (3, T+1)
    Mat2X u;           // (2, T)
    Vec d;             // (T,)
    double objective;  // recomputed in the original cvxpy form
    bool success = false;
    int status = -1;  // raw OsqpEigen::Status value, for diagnostics
  };

  // fa_list: T of (max_num, 2); fb_list: T of (max_num,). Pass empty lists
  // for the no-point case (equivalent to zero fa/fb upstream).
  Result solve(const Mat3X& nom_s, const Mat2X& nom_u, const Mat3X& ref_s,
               const Vec& ref_us, const std::vector<Mat>& fa_list,
               const std::vector<Vec>& fb_list);

  // Port of generate_coefficient_parameter_value: build fa/fb for stages
  // 1..T from the DUNE outputs (mu/lam/points are distance-sorted, the
  // first max_num points are kept, short stages padded with point 0).
  void buildFaFb(const std::vector<Mat>& mu_list,
                 const std::vector<Mat2X>& lam_list,
                 const std::vector<Mat2X>& point_list,
                 std::vector<Mat>& fa_list, std::vector<Vec>& fb_list) const;

  // Changing params alters the Hessian; force a solver re-init on next solve.
  void updateParams(const NRMPParams& params) {
    if (params.max_num != M_)
      throw std::invalid_argument(
          "nrmp: max_num changes the QP dimensions; reconstruct NRMP");
    p_ = params;
    initialized_ = false;
  }
  const NRMPParams& params() const { return p_; }

  // Objective in the original cvxpy form, evaluated at a given solution.
  double objective(const Mat3X& s, const Mat2X& u, const Vec& d,
                   const Mat3X& nom_s, const Mat3X& ref_s, const Vec& ref_us,
                   const std::vector<Mat>& fa_list,
                   const std::vector<Vec>& fb_list) const;

 private:
  void assemble(const Mat3X& nom_s, const Mat2X& nom_u, const Mat3X& ref_s,
                const Vec& ref_us, const std::vector<Mat>& fa_list,
                const std::vector<Vec>& fb_list);

  Robot robot_;
  NRMPParams p_;
  bool no_obs_;
  int T_, M_, nz_, nc_;
  int off_u_, off_d_, off_e_;

  Eigen::SparseMatrix<double> P_, A_;
  Vec q_, lb_, ub_;

  std::unique_ptr<OsqpEigen::Solver> solver_;
  Eigen::Index p_nonzeros_ = 0;
  Eigen::Index a_nonzeros_ = 0;
  bool matrix_structure_initialized_ = false;
  bool initialized_ = false;
};

}  // namespace neupan
