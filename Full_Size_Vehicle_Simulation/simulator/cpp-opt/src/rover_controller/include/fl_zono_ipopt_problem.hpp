#ifndef FL_ZONO_IPOPT_PROBLEM_HPP_
#define FL_ZONO_IPOPT_PROBLEM_HPP_

#include <algorithm>  // for max
#include <limits>     // for numeric_limits
#include <memory>     // for shared_ptr
#include <vector>     // for vector

#include "IpAlgTypes.hpp"  // for SolverReturn
#include "IpTNLP.hpp"      // for IpoptCalculatedQuantities, IpoptData, TNLP
#include "IpTypes.hpp"     // for Index, Number, Ipopt
#include "common.hpp"      // for IndexT
#include "cost_fcn.hpp"    // for CostFcnInfo, CostFcnInfo::CostAndDerivs

/// @file fl_zono_ipopt_problem.hpp Contains the IPOPT NLP to run the
/// optimization

namespace roahm {
namespace fl_zono_ipopt_problem {
using namespace Ipopt;
/// The IPOPT NLP problem definition, where the optimization actually runs
class FlZonoIpoptProblem : public Ipopt::TNLP {
 private:
  /// Convenience to avoid fully qualifying type
  using Number = Ipopt::Number;
  /// Convenience to avoid fully qualifying type
  using MatPtr = std::shared_ptr<Number[]>;
  /// Convenience to avoid fully qualifying type
  using Index = Ipopt::Index;
  /// Constraint matrix, contains A_i matrices in A_i x <= b_i
  MatPtr a_mat_;
  /// Constraint vector, contains b_i vectors in A_i x <= b_i
  MatPtr b_mat_;
  /// The indices of the minimum entry in -(A_i x - b_i) value for each
  /// constraint set
  std::vector<IndexT> min_indices_;
  /// Locations of new zonotope constraints starting
  std::vector<IndexT> zono_startpoints_;
  /// Number of rows corresponding to a zono-obstacle constraint pair
  std::vector<IndexT> zono_obs_sizes_;
  /// Contains information pertaining to the cost function computation
  roahm::CostFcnInfo cost_fcn_info_;
  /// Minimum value that the decision variable can take
  double k_min_;
  /// Maximum value that the decision variable can take
  double k_max_;
  /// Previously computed cost
  double cost_k_;
  /// Previously computed jacobian
  double jac_k_;
  /// Previously computed hessian of the lagrangian
  double hess_k_;

  // JL: add safeguard to make sure eval_f and eval_grad_f use the same k
  /// Parameter value used at the last time the cost was recomputed
  Number cost_used_k_;

  // JL: add safeguard to make sure eval_g and eval_jac_g use the same k
  /// Parameter value used at the last time the constraints were recomputed
  Number constr_used_k_;

 public:
  /// The solution as provided by finalize_solution, only contains a valid value
  /// if IPOPT was successful as of finalize_solution
  double sln_k_;
  /// True if any feasible solution has been found in the past
  bool feasible_found_;
  /// The previous minimum objective cost, only contains a valid value if a
  /// feasible solution has been found in the past
  double prev_min_cost_;
  /// The previous minimum decision variable, only contains a valid value if
  /// a feasible solution has been found in the past
  double prev_min_k_;

  ManuType manu_type_;
 public:
  // TODO why don't we have k_rng in cost_fcn_info?

  /// The optimization problem to solve, provided to IPOPT
  /// \param a_mat constraint matrix containing the \f$ A_i \f$ matrices in the
  /// \f$ A_i x \preceq b_i \f$ constraint equations
  /// \param b_mat constraint matrix containing the \f$ b_i \f$ matrices in the
  /// \f$ A_i x \preceq b_i \f$ constraint equations
  /// \param cost_fcn_info struct containing information necessary to compute
  /// the objective/cost function
  /// \param zono_startpoints the locations of new constraints starting in the
  /// constraint matrices
  /// \param zono_obs_sizes the number of rows in each individual constraint
  /// \param k_rng the sliceable "radius" for the decision variable about the
  /// center point
  FlZonoIpoptProblem(MatPtr a_mat, MatPtr b_mat,
                     roahm::CostFcnInfo cost_fcn_info,
                     std::vector<IndexT> zono_startpoints,
                     std::vector<IndexT> zono_obs_sizes, double k_rng,
                     ManuType manu_type)
      : a_mat_{a_mat},
        b_mat_{b_mat},
        min_indices_{},
        zono_startpoints_{zono_startpoints},
        zono_obs_sizes_{zono_obs_sizes},
        cost_fcn_info_{cost_fcn_info},
        k_min_{cost_fcn_info.c_k_ - k_rng},
        k_max_{cost_fcn_info.c_k_ + k_rng},
        cost_k_{0},
        jac_k_{0},
        hess_k_{0},
        cost_used_k_{100000},    // Initialize with uselessly large number
        constr_used_k_{100000},  // Initialize with uselessly large number
        sln_k_{0},
        feasible_found_{false},
        prev_min_cost_{std::numeric_limits<double>::infinity()},
        prev_min_k_{std::numeric_limits<double>::infinity()},
        manu_type_{manu_type} {
    min_indices_.resize(zono_startpoints_.size());
  }

	double ComputeDeltaY(double k) const {
  	return cost_fcn_info_.ComputeDeltaY(k, k_min_, k_max_);
	}
	
	double ComputeDeltaH(double k) const {
  	return cost_fcn_info_.ComputeDeltaH(k, k_min_, k_max_);
	}

  /// Computes the cost and derivatives
  /// \param k the decision variable / point to compute the cost and derivatives
  /// at
  /// \param eval_opt whether to try to compute an optimal point ignoring
  /// constraints
  /// \return the cost and derivatives
  CostFcnInfo::CostAndDerivs ComputeCostAndDerivs(double k,
                                                  bool eval_opt = false);

  /// Sets information about the problem
  /// \param n out param to store dimensionality of the decision variable
  /// \param m out param to store the number of constraints
  /// \param nnz_jac_g out param to store number of nonzero Jacobian entries
  /// \param nnz_h_lag out param to store number of nonzero Hessian entries
  /// \param index_style whether to 0 (C style) or 1-indexed (Fortran style)
  /// \return true iff successful
  bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g, Index& nnz_h_lag,
                    IndexStyleEnum& index_style) override;

  /// Sets bounds on variables and constraints
  /// \param n dimensionality of the decision variable, x
  /// \param x_l lower bounds on each dimension of the decision variable
  /// \param x_u upper bounds on each dimension of the decision variable
  /// \param m number of constraints
  /// \param g_l lower bounds for each of the constraints
  /// \param g_u upper bounds for each of the constraints
  /// \return true iff successful
  bool get_bounds_info(Index n, Number* x_l, Number* x_u, Index m, Number* g_l,
                       Number* g_u) override;

  /// Gets the starting point for the optimization
  /// \param n dimensionality of the decision variable, x
  /// \param init_x if true, an initial guess must be provided for \p x
  /// \param x out param for providing initial guess for the decision variable
  /// \param init_z if true, initial guess for bound multipliers \p z_l and
  /// \p z_u must be provided
  /// \param z_L out param for initial value for lower bound for bound multipliers
  /// \param z_U out param for initial value for upper bound for bound multipliers
  /// \param m number of constraints
  /// \param init_lambda if true, must provide initial value for the constraint
  /// multipliers \p lambda
  /// \param lambda out param for initial value for constraint multipliers
  /// \return true iff successful
  bool get_starting_point(Index n, bool init_x, Number* x, bool init_z,
                          Number* z_L, Number* z_U, Index m, bool init_lambda,
                          Number* lambda) override;

  /// Evaluates the cost function to compute the objective value
  /// \param n the dimensionality of the decision variable, \p x
  /// \param x the evaluation point to call \f$ f(x) \f$
  /// \param new_x false if any eval_* function has been called with the same
  /// value of \p x, true otherwise
  /// \param obj_value out param to store the value of the objective function
  /// \return true iff successful
  bool eval_f(Index n, const Number* x, bool new_x, Number& obj_value) override;

  /// Computes the gradient of the objective function
  /// \param n dimensionality of the decision variable, \p x
  /// \param x the value of the decision variable to evaluate the gradient at
  /// \param new_x false if any eval_* method was called with the same value
  /// in \p x, true otherwise
  /// \param grad_f out param to store the values of the gradient of the
  /// objective function
  /// \return true iff successful
  bool eval_grad_f(Index n, const Number* x, bool new_x,
                   Number* grad_f) override;

  // JL: add a help function to evaluate g
  /// Evaluates constraints and returns the maximum constraint value to check
  /// whether the result was feasible
  /// \param x the decision variable
  /// \param m the number of constraints
  /// \param g out param to store the constraint values
  /// \return the maximum of all constraints (the "most" violated)
  double ComputeConstraint(const Number* x, Index m, Number* g);

  /// Computes the constraint values
  /// \param n the dimensionality of the decision variable, \p x
  /// \param x the decision variable
  /// \param new_x false if any eval_* was called with the same value in \p x
  /// \param m the number of constraints
  /// \param g out param to store the constraint values
  /// \return true iff successful
  bool eval_g(Index n, const Number* x, bool new_x, Index m,
              Number* g) override;

  /// Evaluates the jacobian of the gradient, and sets the structure (location
  /// of nonzero entries) if it is the first evaluation
  /// \param n the dimensionality of the decision variable, \p x
  /// \param x the decision variable
  /// \param new_x false if any eval_* was called with the same value in \p x
  /// \param m the number of constraints
  /// \param nele_jac the number of nonzero elements in the Jacobian
  /// \param iRow out param. First call: array of length \p nele_jac to store
  /// the row indices of entries in the Jacobian of the constraints.
  /// Later calls: NULL
  /// \param jCol out param. First call: array of length \p nele_jac to store
  /// the column indices of entries in the Jacobian of the constraints.
  /// Later calls: NULL
  /// \param values out param. First call: NULL. Later calls: array of length
  /// \p nele_jac to store the values of the entries in the Jacobian of the
  /// constraints
  /// \return true iff successful
  bool eval_jac_g(Index n, const Number* x, bool new_x, Index m, Index nele_jac,
                  Index* iRow, Index* jCol, Number* values) override;

  /// First call: returns the structure of the hessian of the lagrangian.
  /// Secondary calls: evaluates the hessian of the lagrangian.
  /// \param n the dimensionality of the decision variable, \p x
  /// \param x the decision variable
  /// \param new_x false if any eval_* has been called with the same \p x
  /// \param obj_factor the factor in front of the objective term in the
  /// Hessian
  /// \param m the number of constraints
  /// \param lambda the constraint multipliers to evaluate the Hessian at
  /// \param new_lambda false if any eval_* method was called with the same
  /// values in \p lambda
  /// \param nele_hess the number of nonzero elements in the Hessian
  /// \param iRow out param. First call: array of length \p nele_hess to store
  /// row indices of entries in the Hessian. Later calls: NULL
  /// \param jCol out param. First call: array of length \p nele_hess to store
  /// column indices of entries in the Hessian. Later calls: NULL
  /// \param values out param. First call: NULL. Later calls: array of length
  /// \p nele_hess to store values of entries in the Hessian
  /// \return true iff successful
  bool eval_h(Index n, const Number* x, bool new_x, Number obj_factor, Index m,
              const Number* lambda, bool new_lambda, Index nele_hess,
              Index* iRow, Index* jCol, Number* values) override;

  /// Called when IPOPT if finished to handle the solution or failure cases
  /// \param status provides the status of the algorithm
  /// \param n the dimensionality of the decision variable, \p x
  /// \param x the final decision variable
  /// \param z_L the final lower bound multipliers
  /// \param z_U the final upper bound multipliers
  /// \param m the number of constraints
  /// \param g the final constraint function values
  /// \param lambda the final constraint multipliers
  /// \param obj_value the final objective function value
  /// \param ip_data extra data
  /// \param ip_cq extra data
  void finalize_solution(Ipopt::SolverReturn status, Index n, const Number* x,
                         const Number* z_L, const Number* z_U, Index m,
                         const Number* g, const Number* lambda,
                         Number obj_value, const Ipopt::IpoptData* ip_data,
                         Ipopt::IpoptCalculatedQuantities* ip_cq) override;

  /// Returns true iff a feasible solution has been found at any point
  /// \return true iff a feasible solution has been found at any point
  bool FoundFeasible() const;

  /// Returns the cost associated with the minimum-cost feasible solution, if
  /// a feasible solution has been found at any point.
  /// \return if a feasible solution has been found at any point, returns the
  /// cost associated with the minimum-cost feasible solution, otherwise there
  /// are no guarantees on return value.
  double GetFeasibleCost() const;

  /// Returns the solution associated with the minimum-cost feasible solution,
  /// if a feasible solution has been found at any point.
  /// \return if a feasible solution has been found at any point, returns the
  /// solution associated with the minimum-cost feasible solution that has been
  /// found, otherwise there are no guarantees on the return value.
  double GetFeasibleParam() const;

  ~FlZonoIpoptProblem() override = default;
  FlZonoIpoptProblem() = delete;
  FlZonoIpoptProblem(const FlZonoIpoptProblem&) = delete;
  FlZonoIpoptProblem& operator=(const FlZonoIpoptProblem&) = delete;
};
}  // namespace fl_zono_ipopt_problem
}  // namespace roahm

#endif
