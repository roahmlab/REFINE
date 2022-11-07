// Copyright (C) 2004, 2006 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: MyNLP.hpp 1861 2010-12-21 21:34:47Z andreasw $
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2004-11-05

#ifndef __MYNLP_HPP__
#define __MYNLP_HPP__

#include <cassert>
#include <iostream>
#include <memory>
#include <numeric>
#include <ros/ros.h>

#include "IpTimingStatistics.hpp"
#include "IpIpoptData.hpp"
#include "IpTNLP.hpp"
#include "common.h"
#include "bench_helper.h"
#include "gencon.h"
#define ENABLE_F_TIMINGS true
#define PRINT_TIMINGS true

using namespace Ipopt;

/** C++ Example NLP for interfacing a problem with IPOPT.
 *  MyNLP implements a C++ example showing how to interface with IPOPT
 *  through the TNLP interface. This example is designed to go along with
 *  the tutorial document (see Examples/CppTutorial/).
 *  This class implements the following NLP.
 *
 * min_x f(x) = -(x2-2)^2
 *  s.t.
 *       0 = x1^2 + x2 - 1
 *       -1 <= x1 <= 1
 *
 */
namespace roahm {
class MyNLP : public TNLP {
 private:
  using MatPtr = std::shared_ptr<Number[]>;
  MatPtr a_mat_;
  MatPtr b_mat_;
  Index num_ab_rows_;
  std::vector<Index> min_indices_;
  std::vector<IndexT> zono_startpoints_;
  std::vector<IndexT> zono_obs_sizes_;
  roahm::CostFcnInfo cost_fcn_info_;
  double k_rng_;
  double k_min_;
  double k_max_;
#if ENABLE_F_TIMINGS
 public:
  std::vector<double> total_eval_f_times_;
  std::vector<double> total_eval_grad_f_times_;
  std::vector<double> total_eval_g_times_;
  std::vector<double> total_eval_jac_g_times_;
  std::vector<double> total_eval_h_times_;
  double total_eval_jac_g_init_time_;
  double total_eval_h_init_time_;
 private:
#endif
  double cost_k_;
  double jac_k_;
  double hess_k_;
 public:
  double sln_k_;
  bool feasible_found_;
  double prev_min_cost_;
  double prev_min_k_;

 public:
  /** default constructor */
  // MyNLP();
  MyNLP(Index num_ab_rows, MatPtr a_mat, MatPtr b_mat, roahm::CostFcnInfo cost_fcn_info, 
      std::vector<IndexT> zono_startpoints,
      std::vector<IndexT> zono_obs_sizes, double k_rng)
      : num_ab_rows_{num_ab_rows},
        zono_startpoints_{zono_startpoints},
        zono_obs_sizes_{zono_obs_sizes},
        a_mat_{a_mat},
        b_mat_{b_mat},
        cost_fcn_info_{cost_fcn_info},
        k_rng_{k_rng},
        k_min_{cost_fcn_info.c_k_ - k_rng},
        k_max_{cost_fcn_info.c_k_ + k_rng},
#if ENABLE_F_TIMINGS
  total_eval_f_times_{},
  total_eval_grad_f_times_{},
  total_eval_g_times_{},
  total_eval_jac_g_times_{},
  total_eval_h_times_{},
  total_eval_jac_g_init_time_{0},
  total_eval_h_init_time_{0},
#endif
        cost_k_{0},
        jac_k_{0},
        hess_k_{0},
        sln_k_{0},
        feasible_found_{false},
        prev_min_cost_{std::numeric_limits<double>::infinity()}, 
        prev_min_k_{std::numeric_limits<double>::infinity()} {
    min_indices_.resize(zono_startpoints_.size());
  }

  auto ComputeCosts(const double k, bool eval_opt=false) {
    const auto f_evals = cost_fcn_info_.ComputeCosts(k, k_min_, k_max_, eval_opt);
    cost_k_ = f_evals.cost_k_;
    jac_k_  = f_evals.jac_k_;
    hess_k_ = f_evals.hess_k_;
    return f_evals;
  }

  /** default destructor */
  virtual ~MyNLP() {
  }

  /**@name Overloaded from TNLP */
  //@{
  /** Method to return some info about the nlp */
  virtual bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                            Index& nnz_h_lag, IndexStyleEnum& index_style) {
    // Number of variables
    n = 1;
    // Number of constraints
    m = zono_startpoints_.size();
    std::cout << "(IPDBG) Num Constraints: " << m << std::endl;
    // Nonzero jacobian entries
    nnz_jac_g = n * m;
    // Nonzero hessian of lagrangian
    nnz_h_lag = 1;
    index_style = C_STYLE;
    return true;
  }

  /** Method to return the bounds for my problem */
  virtual bool get_bounds_info(Index n, Number* x_l, Number* x_u, Index m,
                               Number* g_l, Number* g_u) {
    const double k_rng = k_rng_;

    /* TODO
#if PRINT_TIMINGS
    PRINT_VAR(cost_fcn_info_.c_k_)
    PRINT_VAR(k_rng)
#endif
*/
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

  /** Method to return the starting point for the algorithm */
  virtual bool get_starting_point(Index n, bool init_x, Number* x, bool init_z,
                                  Number* z_L, Number* z_U, Index m,
                                  bool init_lambda, Number* lambda) {
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);

    x[0] = ComputeCosts(cost_fcn_info_.c_k_, true).optimal_k_in_rng_;

    return true;
  }

  /** Method to return the objective value */
  virtual bool eval_f(Index n, const Number* x, bool new_x, Number& obj_value) {
#if ENABLE_F_TIMINGS
    const auto tick0 = Tick();
#endif

    const Number k_curr = x[0];
    ComputeCosts(k_curr);
    obj_value = cost_k_;

#if ENABLE_F_TIMINGS
    const auto tick1 = Tick();
    total_eval_f_times_.push_back(GetDeltaS(tick1, tick0));
#endif
    /*
     * TODO
#if PRINT_TIMINGS
    std::cout << "Eval F: " << GetDeltaS(tick1, tick0)*1000.0 << "ms" << std::endl;
    std::cout << "Eval F: " << GetDeltaS(tick1, tick0)*1000000.0 << "us" << std::endl;
#endif
*/
    return true;
  }

  /** Method to return the gradient of the objective */
  virtual bool eval_grad_f(Index n, const Number* x, bool new_x,
                           Number* grad_f) {
#if ENABLE_F_TIMINGS
    const auto tick0 = Tick();
#endif

    // Already computed with the cost to ComputeCosts in eval_f
    std::cout << "(IPDBG) Gradient F: " << jac_k_ << std::endl;
    ComputeCosts(x[0]);
    grad_f[0] = jac_k_;

#if ENABLE_F_TIMINGS
    const auto tick1 = Tick();
    total_eval_grad_f_times_.push_back(GetDeltaS(tick1, tick0));
#endif
    return true;
  }

  /** Method to return the constraint residuals */
  virtual bool eval_g(Index n, const Number* x, bool new_x, Index m,
                      Number* g) {
#if ENABLE_F_TIMINGS
    const auto eval_g_start_tick = Tick();
#endif
    const Number k_curr = x[0];
    const auto *const a_ptr = a_mat_.get();
    const auto *const b_ptr = b_mat_.get();
    const Number lambda_val = (k_curr - cost_fcn_info_.c_k_) / cost_fcn_info_.g_k_;
    static_assert(std::numeric_limits<double>::is_iec559, 
        "Must have negative infinity available (guranteed by IEEE 754)");
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
      max_of_all_constraints = Max(max_of_all_constraints, min_val);
      g[zono_idx] = min_val;
      min_indices_.at(zono_idx) = min_idx;
    }

    std::cout << "MAX: " << max_of_all_constraints << std::endl;
    // Keep track of lowest cost parameter that has been feasible
    if (max_of_all_constraints <= 0.0) {
      std::cout << "[Max <= 0]" << std::endl;
      const auto curr_cost = ComputeCosts(k_curr).cost_k_;
      if ((not feasible_found_) or (curr_cost < prev_min_cost_)) {
        // If we haven't previously found a feasible solution, or this is the
        // lowest cost feasible solution, store it.
        prev_min_cost_ = curr_cost;
        prev_min_k_ = k_curr;
      }
      feasible_found_ = true;
    }

#if ENABLE_F_TIMINGS
    const auto eval_g_end_tick = Tick();
    total_eval_g_times_.push_back(GetDeltaS(eval_g_end_tick, eval_g_start_tick));
#endif
#if PRINT_TIMINGS
    std::cout << "EVAL G: " << GetDeltaS(eval_g_end_tick, eval_g_start_tick)*1000.0<< "ms\n";
    std::cout << "EVAL G: " << GetDeltaS(eval_g_end_tick, eval_g_start_tick)*1000000.0<< "us\n";
    std::cout << "EVAL G: " << GetDeltaS(eval_g_end_tick, eval_g_start_tick)*1000000.0/((double)m)<< "us per constraint\n";
#endif
    return true;
  }

  /** Method to return:
   *   1) The structure of the jacobian (if "values" is NULL)
   *   2) The values of the jacobian (if "values" is not NULL)
   */
  virtual bool eval_jac_g(Index n, const Number* x, bool new_x, Index m,
                          Index nele_jac, Index* iRow, Index* jCol,
                          Number* values) {
#if ENABLE_F_TIMINGS
    const auto eval_jac_g_start_tick = Tick();
#endif
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
#if ENABLE_F_TIMINGS
    const auto eval_jac_g_end_tick = Tick();
    total_eval_jac_g_init_time_ = GetDeltaS(eval_jac_g_end_tick, eval_jac_g_start_tick);
#endif
#if PRINT_TIMINGS
    std::cout << "EVAL JAC G INIT: " << GetDeltaS(eval_jac_g_end_tick, eval_jac_g_start_tick)*1000.0<< "ms\n";
    std::cout << "EVAL JAC G INIT: " << GetDeltaS(eval_jac_g_end_tick, eval_jac_g_start_tick)*1000000.0<< "us\n";
    std::cout << "EVAL JAC G INIT: " << GetDeltaS(eval_jac_g_end_tick, eval_jac_g_start_tick)*1000000.0/m << "us per constraint\n";
#endif
    } else {
      // return the values of the jacobian of the constraints
      // element at i,j: grad_{x_j} g_{i}(x)
      auto* a_ptr = a_mat_.get();
      for (Index r = 0; r < m; ++r) {
        const Index idx = min_indices_.at(r);
        values[r] = -a_ptr[idx] / cost_fcn_info_.g_k_;
      }
#if ENABLE_F_TIMINGS
    const auto eval_jac_g_end_tick = Tick();
    total_eval_jac_g_times_.push_back(GetDeltaS(eval_jac_g_end_tick, eval_jac_g_start_tick));
#endif
#if PRINT_TIMINGS
    std::cout << "EVAL JAC G: " << GetDeltaS(eval_jac_g_end_tick, eval_jac_g_start_tick)*1000.0<< "ms\n";
    std::cout << "EVAL JAC G: " << GetDeltaS(eval_jac_g_end_tick, eval_jac_g_start_tick)*1000000.0<< "us\n";
    std::cout << "EVAL JAC G: " << GetDeltaS(eval_jac_g_end_tick, eval_jac_g_start_tick)*1000000.0/m << "us per constraint\n";
#endif
    }

    return true;
  }

  /** Method to return:
   *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
   *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
   */
  virtual bool eval_h(Index n, const Number* x, bool new_x, Number obj_factor,
                      Index m, const Number* lambda, bool new_lambda,
                      Index nele_hess, Index* iRow, Index* jCol,
                      Number* values) {
#if ENABLE_F_TIMINGS
    const auto t0 = Tick();
#endif
    if (values == NULL) {
      // return the structure. This is a symmetric matrix, fill the lower left
      // triangle only.

      // element at 1,1: grad^2_{x1,x1} L(x,lambda)
      iRow[0] = 0;
      jCol[0] = 0;
#if ENABLE_F_TIMINGS
    const auto t1 = Tick();
    total_eval_h_init_time_ = GetDeltaS(t1, t0);
#endif
    } else {
      // return the values
      // element at 1,1: grad^2_{x1,x1} L(x,lambda)
      ComputeCosts(x[0]);
      values[0] = obj_factor * hess_k_;
#if ENABLE_F_TIMINGS
    const auto t1 = Tick();
    total_eval_h_times_.push_back(GetDeltaS(t1, t0));
#endif
    }

    return true;
  }

  //@}

  /** @name Solution Methods */
  //@{
  /** This method is called when the algorithm is complete so the TNLP can
   * store/write the solution */
  virtual void finalize_solution(SolverReturn status, Index n, const Number* x,
                                 const Number* z_L, const Number* z_U, Index m,
                                 const Number* g, const Number* lambda,
                                 Number obj_value, const IpoptData* ip_data,
                                 IpoptCalculatedQuantities* ip_cq) {
//#if !TIMING_RUNNING
    ROS_INFO_STREAM("K: " << x[0] << "out of [" << k_min_ << ", " << k_max_ << "]");
//#endif
    sln_k_ = x[0];
  }
  //@}

  bool FoundFeasible() const {
    return feasible_found_;
  }
  double GetFeasibleCost() const {
    return prev_min_cost_;
  }
  double GetFeasibleParam() const {
    return prev_min_k_;
  }

 private:
  /**@name Methods to block default compiler methods.
   * The compiler automatically generates the following three methods.
   *  Since the default compiler implementation is generally not what
   *  you want (for all but the most simple classes), we usually
   *  put the declarations of these methods in the private section
   *  and never implement them. This prevents the compiler from
   *  implementing an incorrect "default" behavior without us
   *  knowing. (See Scott Meyers book, "Effective C++")
   *
   */
  //@{
  //  MyNLP();
  MyNLP(const MyNLP&);
  MyNLP& operator=(const MyNLP&);
  //@}
};
}

#endif
