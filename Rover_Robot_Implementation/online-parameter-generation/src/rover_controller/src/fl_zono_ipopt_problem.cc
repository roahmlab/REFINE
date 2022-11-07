#include "fl_zono_ipopt_problem.hpp"

#include <cassert>  // for assert
#include <cstddef>  // for NULL
#include <ostream>  // for operator<<, basic_ostream<>::__ostream_type

#include "ros/console.h"  // for LogLocation, ROS_INFO_STREAM

namespace roahm {

namespace fl_zono_ipopt_problem {
CostFcnInfo::CostAndDerivs FlZonoIpoptProblem::ComputeCostAndDerivs(
    const double k, bool eval_opt) {
  const auto f_evals = cost_fcn_info_.ComputeCosts(k, k_min_, k_max_, eval_opt);
  cost_k_ = f_evals.cost_k_;
  jac_k_ = f_evals.jac_k_;
  hess_k_ = f_evals.hess_k_;
  cost_used_k_ = k;
  return f_evals;
}

bool FlZonoIpoptProblem::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                                      Index& nnz_h_lag,
                                      IndexStyleEnum& index_style) {
  // Number of variables
  n = 1;
  // Number of constraints
  m = zono_startpoints_.size();
  // Nonzero jacobian entries
  nnz_jac_g = n * m;
  // Nonzero hessian of lagrangian
  nnz_h_lag = 1;
  index_style = C_STYLE;
  return true;
}

bool FlZonoIpoptProblem::get_bounds_info(Index n, Number* x_l, Number* x_u,
                                         Index m, Number* g_l, Number* g_u) {
  // min max stuff
  x_l[0] = k_min_;
  x_u[0] = k_max_;

  // Constraint bounds
  for (Index r = 0; r < m; ++r) {
    g_l[r] = -1.0e19;
  }
  for (Index i = 0; i < m; ++i) {
    g_u[i] = 0.0;
  }

  return true;
}

bool FlZonoIpoptProblem::get_starting_point(Index n, bool init_x, Number* x,
                                            bool init_z, Number* z_L,
                                            Number* z_U, Index m,
                                            bool init_lambda, Number* lambda) {
  assert(init_x == true);
  assert(init_z == false);
  assert(init_lambda == false);

  x[0] = ComputeCostAndDerivs(cost_fcn_info_.c_k_, true).optimal_k_in_rng_;

  return true;
}

bool FlZonoIpoptProblem::eval_f(Index n, const Number* x, bool new_x,
                                Number& obj_value) {
  const Number k_curr = x[0];
  ComputeCostAndDerivs(k_curr);
  obj_value = cost_k_;
  return true;
}

bool FlZonoIpoptProblem::eval_grad_f(Index n, const Number* x, bool new_x,
                                     Number* grad_f) {
  // Already computed with the cost to ComputeCosts in eval_f
  if (cost_used_k_ != x[0]) {  // JL: add safeguard
    ComputeCostAndDerivs(x[0]);
  }
  grad_f[0] = jac_k_;

  return true;
}

double FlZonoIpoptProblem::ComputeConstraint(const Number* x, Index m,
                                             Number* g) {
  const Number k_curr = x[0];
  constr_used_k_ = k_curr;
  const auto* const a_ptr = a_mat_.get();
  const auto* const b_ptr = b_mat_.get();
  const Number lambda_val =
      (k_curr - cost_fcn_info_.c_k_) / cost_fcn_info_.g_k_;
  static_assert(
      std::numeric_limits<double>::is_iec559,
      "Must have negative infinity available (guaranteed by IEEE 754)");
  constexpr double kNegInfDbl = -std::numeric_limits<double>::infinity();
  double max_of_all_constraints = kNegInfDbl;
  for (Index zono_idx = 0; zono_idx < m; ++zono_idx) {
    const Index start_idx = zono_startpoints_.at(zono_idx);
    const Index end_idx = start_idx + zono_obs_sizes_.at(zono_idx);
    Number min_val = std::numeric_limits<Number>::max();
    Index min_idx = start_idx;
    for (Index i = start_idx; i < end_idx; ++i) {
      const Number curr_val = -(a_ptr[i] * lambda_val - b_ptr[i]);
      if (curr_val < min_val) {
        min_val = curr_val;
        min_idx = i;
      }
    }
    max_of_all_constraints = std::max(max_of_all_constraints, min_val);
    g[zono_idx] = min_val;
    min_indices_.at(zono_idx) = min_idx;
  }
  return max_of_all_constraints;
}

bool FlZonoIpoptProblem::eval_g(Index n, const Number* x, bool new_x, Index m,
                                Number* g) {
  double max_of_all_constraints = ComputeConstraint(x, m, g);

  // Keep track of the lowest cost parameter that has been feasible
  if (max_of_all_constraints <= 0.0) {
    const auto curr_cost = ComputeCostAndDerivs(x[0]).cost_k_;
    if ((not feasible_found_) or (curr_cost < prev_min_cost_)) {
      // If we haven't previously found a feasible solution, or this is the
      // lowest cost feasible solution, store it.
      prev_min_cost_ = curr_cost;
      prev_min_k_ = x[0];
    }
    feasible_found_ = true;
  }

  return true;
}

bool FlZonoIpoptProblem::eval_jac_g(Index n, const Number* x, bool new_x,
                                    Index m, Index nele_jac, Index* iRow,
                                    Index* jCol, Number* values) {
  if (values == NULL) {
    // Return structure of the jacobian of the constraints
    // element at i,j: grad_{x_j} g_{i}(x)
    for (Index r = 0; r < m; ++r) {
      for (Index c = 0; c < n; ++c) {
        Index idx = (r * n) + c;
        iRow[idx] = r;
        jCol[idx] = c;
      }
    }
  } else {
    // return the values of the jacobian of the constraints
    // element at i,j: grad_{x_j} g_{i}(x)
    if (constr_used_k_ != x[0])
      ComputeConstraint(x, m,
                        values);  // JL: add safeguard. Notices values has
    // the same length as g, but values will be
    // recomputed in the next for loop

    auto* a_ptr = a_mat_.get();
    for (Index r = 0; r < m; ++r) {
      const Index idx = min_indices_.at(r);
      values[r] = -a_ptr[idx] / cost_fcn_info_.g_k_;
    }
  }

  return true;
}

bool FlZonoIpoptProblem::eval_h(Index n, const Number* x, bool new_x,
                                Number obj_factor, Index m,
                                const Number* lambda, bool new_lambda,
                                Index nele_hess, Index* iRow, Index* jCol,
                                Number* values) {
  if (values == NULL) {
    // return the structure. This is a symmetric matrix, fill the lower left
    // triangle only.

    // element at 1,1: grad^2_{x1,x1} L(x,lambda)
    iRow[0] = 0;
    jCol[0] = 0;
  } else {
    // return the values
    // element at 1,1: grad^2_{x1,x1} L(x,lambda)
    if (cost_used_k_ != x[0])  // JL: add safeguard
      ComputeCostAndDerivs(x[0]);
    values[0] = obj_factor * hess_k_;
  }

  return true;
}

void FlZonoIpoptProblem::finalize_solution(
    SolverReturn status, Index n, const Number* x, const Number* z_L,
    const Number* z_U, Index m, const Number* g, const Number* lambda,
    Number obj_value, const IpoptData* ip_data,
    IpoptCalculatedQuantities* ip_cq) {
  ROS_INFO_STREAM("K: " << x[0] << "out of [" << k_min_ << ", " << k_max_
                        << "]");
  sln_k_ = x[0];
}

bool FlZonoIpoptProblem::FoundFeasible() const { return feasible_found_; }
double FlZonoIpoptProblem::GetFeasibleCost() const { return prev_min_cost_; }
double FlZonoIpoptProblem::GetFeasibleParam() const { return prev_min_k_; }
}  // namespace fl_zono_ipopt_problem
}  // namespace roahm