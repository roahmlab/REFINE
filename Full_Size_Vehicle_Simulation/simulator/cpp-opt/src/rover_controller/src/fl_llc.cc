#include "fl_llc.hpp"

#include <cmath>  // for sin, tanh, cos, atan, sqrt

#include "consts.hpp"          // for kLf, kLr, kM, kCaf1, kC2, kCaf2, kCar1
#include "rover_state.hpp"     // for RoverState
#include "simple_util.hpp"     // for AngleDiff
#include "trajectory_sym.hpp"  // for TrajectoryValue, Trajectory

namespace roahm {

RoverCmd FeedbackLinController(RoverState& state, double dt, double t,
                               const Trajectory& traj,
                               const RoverDriveMode& drive_mode_) {
  //
  // Load constants
  //
  const double c_bar_alpha_f = 24.03;
  const double c_bar_alpha_r = 43.71;
  const double u_cri = RtdConsts::kUReallySlow;  // 0.5
  const double mu_bar = RtdConsts::kMuBar;       // 0.95
  const double g = RtdConsts::kGravConst;        // 9.8
  const double m = RtdConsts::kRoverMass;        // 4.956
  const double L = RtdConsts::kLengthTotal;      // 0.31
  const double l_r = RtdConsts::kLr;             // 0.107
  const double l_f = 0.203;
  const double I_zz = 0.11;
  const double K_h = 8.0;
  const double K_r = 5.0;
  const double K_u = 4.0;
  const double M_r = 4.86364;
  const double M_u = 0.3;  // TODO
  const double C_us = (m / L) * ((l_r / c_bar_alpha_f) - (l_f / c_bar_alpha_r));
  const double r_w = 0.05461;
  const double kappa_1_u = 1.2;
  const double kappa_2_u = 0.8;
  const double phi_1_u = 1.2;
  const double phi_2_u = 0.8;
  const double kappa_1_r = 0.4;
  const double kappa_2_r = 0.9;
  const double phi_1_r = 0.4;
  const double phi_2_r = 0.9;

  //
  // Load state variables
  //
  const double u = state.u_;
  const double v = state.v_;
  const double r = state.r_;
  const double h = state.GetHeading();
  const double w = state.w_;

  //
  // Load trajectory variables
  //
  const auto traj_des_vals = traj.EvalAt(t);
  const double u_des = traj_des_vals.ud_;
  const double u_dot_des = traj_des_vals.uddot_;
  const double v_des = traj_des_vals.vd_;
  const double v_dot_des = traj_des_vals.vddot_;
  const double r_des = traj_des_vals.rd_;
  const double h_des = traj_des_vals.hd_;
  const double r_dot_des = traj_des_vals.rddot_;

  // Update error terms
  const double r_err = r - r_des;
  const double h_err = AngleDiff(h, h_des);
  const double u_err = u - u_des;

  // Whether it should be braking or not
  const bool is_braking = u_dot_des < 0.0;

  // Update the integrated error^2 terms
  state.r_err_sum_ += (r_err * r_err) * dt;
  state.h_err_sum_ += (h_err * h_err) * dt;
  state.u_err_sum_ += (u_err * u_err) * dt;

  // Compute integral of |err|^2
  const double r_err2_int = state.r_err_sum_;
  const double h_err2_int = state.h_err_sum_;
  const double u_err2_int = state.u_err_sum_;

  // Eq (26-28)
  const double kappa_r = kappa_1_r + (kappa_2_r * r_err2_int);
  const double phi_r = phi_1_r + (phi_2_r * r_err2_int);
  const double tau_r = -((kappa_r * M_r) + phi_r) * r_err;

  // Eq (20-22)
  const double kappa_u = kappa_1_u + kappa_2_u * u_err2_int;
  const double phi_u = phi_1_u + phi_2_u * u_err2_int;
  const double tau_u = -((kappa_u * M_u) + phi_u) * u_err;

  // Eq (72)
  // l_f F_xf = l_r F_xr
  // F_xr = (l_f / l_r) F_xf
  // F_xf + F_xr = F_xf + (l_f / l_r) F_xf
  //             = (1 + l_f / l_r) F_xf
  // = -m*K_u*u(t) + m*K_u*u_des(t,p) +
  //    m*u^dot_des(t,p) - m*v(t)*r(t) + m*tau_u(t, p)
  const double F_xf = ((-m * K_u * u) + (m * K_u * u_des) + (m * u_dot_des) -
                       (m * v * r) + (m * tau_u)) /
                      (1.0 + l_f / l_r);

  // Eq (34)
  const double w_cmd_intermediate_term = L * F_xf / (mu_bar * m * g * l_r);
  double w_cmd{};
  if (is_braking) {
    w_cmd = (u / r_w) * (w_cmd_intermediate_term + 1.0);
  } else {
    w_cmd = (u / r_w) * (1.0 / (1.0 - w_cmd_intermediate_term));
  }

  // Eq (8)
  const double u_denom = (std::abs(u) < 0.05) ? 0.05 : u;
  const double alpha_r = -(v - (l_r * r)) / u_denom;
  // Eq (12)
  const double F_yr = c_bar_alpha_r * alpha_r;

  // Eq (25)
  const double F_yf = -((I_zz * K_r * r_err) / l_f) +
                      ((I_zz / l_f) * r_dot_des) +
                      -((I_zz * K_h * h_err) / l_f) + ((l_r / l_f) * F_yr) +
                      ((I_zz / l_f) * tau_r);
  double delta = 0.0;
  if (u > u_cri) {
    // Eq (35)
    delta = (F_yf / c_bar_alpha_f) + ((v + (l_f * r)) / u);
  } else {
    // Eq (36)
    delta = (r_des * (L + C_us * (u * u))) / u;
  }

  // Write outputs
  RoverCmd ret_val{};
  ret_val.w_ = w_cmd;
  ret_val.delta_ = delta;
  ret_val.ud_ = u_des;
  ret_val.uddot_ = u_dot_des;
  ret_val.vd_ = v_des;
  ret_val.vddot_ = v_dot_des;
  ret_val.rd_ = r_des;
  ret_val.hd_ = h_des;
  ret_val.rddot_ = r_dot_des;
  return ret_val;
}
}  // namespace roahm
