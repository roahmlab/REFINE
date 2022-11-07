#include "cost_fcn.hpp"

#include <cmath>     // for sqrt, isfinite
#include <iostream>  // for operator<<, basic_ostream, basic_ostream<...

#include "simple_util.hpp"  // for Square, InClosedInterval

namespace roahm {
namespace {
/// Raises an argument to the (3/2) power
/// \param x the value to raise to the \f$ \frac{3}{2} \f$ power
/// \return the input argument to the (3/2) power, \f$ x^\frac{3}{2} \f$
inline double ThreeHalvesPower(double x) {
  const double sqrt_expr = std::sqrt(x);
  return sqrt_expr * sqrt_expr * sqrt_expr;
}

}  // namespace
CostFcnInfo GenCostFcn(const Vehrs& vehrs, const IndexT desired_idx,
                       const double state_u, const double state_v,
                       const double state_r, const double x_des,
                       const double y_des, const double h_des) {
  CostFcnInfo cost_fcn_info;
  const auto& slc =
      vehrs.slc_infos_.at(desired_idx).Slice(state_u, state_v, state_r);
  const double c_x = vehrs.xy_centers_[desired_idx * 2 + 0] + slc.x_sliced_sum_;
  const double c_y = vehrs.xy_centers_[desired_idx * 2 + 1] + slc.y_sliced_sum_;
  const double c_h = vehrs.h_centers_[desired_idx] + slc.h_sliced_sum_;
  cost_fcn_info.c_x_ = c_x;
  cost_fcn_info.c_y_ = c_y;
  cost_fcn_info.c_h_ = ToAngle(c_h);
  cost_fcn_info.c_k_ = slc.center_slc_val_;
  cost_fcn_info.g_x_ = slc.slc_x_;
  cost_fcn_info.g_y_ = slc.slc_y_;
  cost_fcn_info.g_h_ = slc.slc_h_;
  cost_fcn_info.g_k_ = slc.slc_val_;
  cost_fcn_info.x_des_ = x_des;
  cost_fcn_info.y_des_ = y_des;
  cost_fcn_info.h_des_ = h_des;
  return cost_fcn_info;
}

CostFcnInfo::CostAndDerivs CostFcnInfo::ComputeCosts(const double k,
                                                     const double k_min,
                                                     const double k_max,
                                                     bool eval_opt) const {
  CostAndDerivs ret_val{};
  const double c_x = c_x_;
  const double c_y = c_y_;
  const double c_h = c_h_;
  const double c_k = c_k_;
  const double g_x = g_x_;
  const double g_y = g_y_;
  const double g_h = g_h_;
  const double g_k = g_k_;
  const double gx2 = Square(g_x);
  const double gy2 = Square(g_y);
  const double gh2 = Square(g_h);
  const double gk2 = Square(g_k);
  const double x_des = x_des_;
  const double y_des = y_des_;
  const double h_des = h_des_;

  eval_opt = false;

#if SIMULATION
  double weight_xy = 0.1;
  double weight_h = 10;

  if (std::abs(h_des) < 0.5) {  // slightly less than 30[deg]
    weight_h = 0.1;
    weight_xy = 10;
  }
#else
  double weight_xy = 4.0;
  double weight_h = 10.0;

  if (std::abs(h_des) < 0.15) {  // slightly less than 30[deg]
    weight_h = 1.0;
    weight_xy = 10.0;
  }
#endif

  ret_val.cost_k_ =
      weight_h * std::sqrt(Square(h_des - c_h + (g_h * (c_k - k)) / g_k)) +
      weight_xy * std::sqrt(Square(x_des - c_x + (g_x * (c_k - k)) / g_k) +
                            Square(y_des - c_y + (g_y * (c_k - k)) / g_k));
  ret_val.jac_k_ =
      -(weight_xy *
        ((2.0 * g_x * (x_des - c_x + (g_x * (c_k - k)) / g_k)) / g_k +
         (2.0 * g_y * (y_des - c_y + (g_y * (c_k - k)) / g_k)) / g_k)) /
          (2.0 * std::sqrt(Square(x_des - c_x + (g_x * (c_k - k)) / g_k) +
                           Square(y_des - c_y + (g_y * (c_k - k)) / g_k))) -
      (g_h * weight_h * (h_des - c_h + (g_h * (c_k - k)) / g_k)) /
          (g_k * std::sqrt(Square(h_des - c_h + (g_h * (c_k - k)) / g_k)));
  ret_val.hess_k_ =
      (weight_xy * ((2.0 * gx2) / gk2 + (2.0 * gy2) / gk2)) /
          (2.0 * std::sqrt(Square(x_des - c_x + (g_x * (c_k - k)) / g_k) +
                           Square(y_des - c_y + (g_y * (c_k - k)) / g_k))) -
      (weight_xy *
       Square((2.0 * g_x * (x_des - c_x + (g_x * (c_k - k)) / g_k)) / g_k +
              (2.0 * g_y * (y_des - c_y + (g_y * (c_k - k)) / g_k)) / g_k)) /
          (4 *
           ThreeHalvesPower(Square(x_des - c_x + (g_x * (c_k - k)) / g_k) +
                            Square(y_des - c_y + (g_y * (c_k - k)) / g_k))) +
      (gh2 * weight_h) /
          (gk2 * std::sqrt(Square(h_des - c_h + (g_h * (c_k - k)) / g_k))) -
      (gh2 * weight_h * Square(h_des - c_h + (g_h * (c_k - k)) / g_k)) /
          (gk2 *
           ThreeHalvesPower(Square(h_des - c_h + (g_h * (c_k - k)) / g_k)));

  // We can have discontinuous derivatives, just set jacobian to zero
  // at the discontinouous points.
  const auto handle_discontinuity = [](double d) {
    if (not std::isfinite(d)) {
      return 0.0;
    }
    return d;
  };
  ret_val.jac_k_ = handle_discontinuity(ret_val.jac_k_);
  ret_val.hess_k_ = handle_discontinuity(ret_val.hess_k_);

  if (eval_opt) {
    double jac_0 = (c_k * gh2 + c_k * gx2 + c_k * gy2 - c_h * g_h * g_k -
                    c_x * g_k * g_x - c_y * g_k * g_y + g_h * g_k * h_des +
                    g_k * g_x * x_des + g_k * g_y * y_des) /
                   (gh2 + gx2 + gy2);
    const auto optimal_evals = ComputeCosts(jac_0, k_min, k_max, false);
    const double optimal_cost = optimal_evals.cost_k_;
    const double optimal_jac = optimal_evals.jac_k_;
    const double optimal_hess = optimal_evals.hess_k_;
    const double delta_optimal_cost =
        ComputeCosts(jac_0 + 0.05, k_min, k_max, false).cost_k_;
    const bool concave_up = optimal_cost < delta_optimal_cost;
    const double dist_to_min = std::abs(jac_0 - k_min);
    const double dist_to_max = std::abs(jac_0 - k_max);
    const bool jac_0_in_thresh = InClosedInterval(jac_0, k_min, k_max);
    if (concave_up) {
      if (jac_0_in_thresh) {
        ret_val.optimal_k_in_rng_ = jac_0;
      } else {
        ret_val.optimal_k_in_rng_ = (dist_to_max > dist_to_min) ? k_max : k_min;
      }
    } else {
      ret_val.optimal_k_in_rng_ = (dist_to_max > dist_to_min) ? k_min : k_max;
    }
    const double optimal_cost_in_rng =
        ComputeCosts(ret_val.optimal_k_in_rng_, k_min, k_max, false).cost_k_;
  }
  return ret_val;
}
}  // namespace roahm