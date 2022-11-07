#ifndef ROAHM_COST_FCN_HPP_
#define ROAHM_COST_FCN_HPP_

#include "common.hpp"      // for IndexT
#include "frs_loader.hpp"  // for Vehrs

/// @file cost_fcn.hpp Contains structs for the NLP cost function evaluation

namespace roahm {
/// TODO
struct CostFcnInfo {
  /// TODO
  double c_x_;
  /// TODO
  double c_y_;
  /// TODO
  double c_h_;
  /// TODO
  double c_k_;
  /// TODO
  double g_x_;
  /// TODO
  double g_y_;
  /// TODO
  double g_h_;
  /// TODO
  double g_k_;
  /// TODO
  double x_des_;
  /// TODO
  double y_des_;
  /// TODO
  double h_des_;
  /// TODO
  struct CostAndDerivs {
    /// TODO
    double cost_k_;
    /// TODO
    double jac_k_;
    /// TODO
    double hess_k_;
    /// TODO
    double optimal_k_in_rng_;
  };
  /// TODO
  /// \param k TODO
  /// \param k_min TODO
  /// \param k_max TODO
  /// \param eval_opt TODO
  /// \return TODO
  CostAndDerivs ComputeCosts(double k, double k_min, double k_max,
                             bool eval_opt) const;
};
/// TODO
/// \param vehrs TODO
/// \param desired_idx TODO
/// \param state_u TODO
/// \param state_v TODO
/// \param state_r TODO
/// \param x_des TODO
/// \param y_des TODO
/// \param h_des TODO
/// \return TODO
CostFcnInfo GenCostFcn(const Vehrs& vehrs, IndexT desired_idx, double state_u,
                       double state_v, double state_r, double x_des,
                       double y_des, double h_des);
}  // namespace roahm
#endif  // ROAHM_COST_FCN_HPP_
