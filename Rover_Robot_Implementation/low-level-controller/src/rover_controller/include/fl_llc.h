#ifndef ROAHM_FL_LLC_H_
#define ROAHM_FL_LLC_H_
#include <fmt/chrono.h>
#include <fmt/compile.h>
#include <fmt/core.h>
#include <fmt/os.h>
#include <fmt/ostream.h>
#include <fmt/ranges.h>

#include <cmath>
#include <utility>
#include <vector>

#define ROAHM_RTD_CONSTS_VERS_ 3
#include "consts.h"
#include "simple_util.h"
#include "trajectory_sym.h"
namespace roahm {

struct TrajDesVals {
  double ud_;
  double uddot_;
  double vd_;
  double vddot_;
  double rd_;
  double hd_;
};

struct RoverCmd {
  double delta_;
  double w_;
  double ud_;
  double uddot_;
  double vd_;
  double vddot_;
  double rd_;
  double hd_;
  double rddot_;
};

enum class RoverDriveMode { kCurrentControl, kSpeedControl };

template <typename T>
void LinspaceFill(T& container, double min_v, double max_v) {
  if (container.size() <= 0) {
    return;
  } else if (container.size() == 1) {
    container[0] = max_v;
    return;
  } else {
    const double rng = max_v - min_v;
    for (std::size_t i = 0; i < container.size(); ++i) {
      container[i] =
          ((static_cast<double>(i) / container.size()) * rng) + min_v;
    }
  }
}

RoverCmd FeedbackLinControllerNew(ComplexHeadingState& state, double w_cmd_prev,
                                  double dt, double t, const Trajectory& traj,
                                  const RoverDriveMode& drive_mode_) {
  //
  // Set constants
  //
  const double c_fric_1 = 8.0897;
  const double c_fric_2 = 10.5921;
  const double kt = 0.4642;
  const double cbar_alpha_f = 24.03;
  const double cbar_alpha_r = 43.71;
  const double u_cri = RtdConsts::kUReallySlow;
  const double mu_bar = 0.95;
  const double l_f = 0.203;
  const double l_r = 0.107;
  const double l = 0.31;
  const double g = 9.8;
  const double m = 4.956;
  const double i_zz = 0.11;
  const double k_h = 8.0;
  const double k_r = 5.0;
  const double k_u = 4.0;
  const double m_r = 4.86364;
  const double m_u = 0.3;
  const double c_us = (m / l) * ((l_r / cbar_alpha_f) - (l_f / cbar_alpha_r));
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
  const Trajectory::TrajectoryValue traj_des_vals = traj.EvalAt(t);
  const double ud = traj_des_vals.ud_;
  const double uddot = traj_des_vals.uddot_;
  const double vd = traj_des_vals.vd_;
  const double vddot = traj_des_vals.vddot_;
  const double rd = traj_des_vals.rd_;
  const double hd = traj_des_vals.hd_;
  const double rddot = traj_des_vals.rddot_;

  // Update error terms
  const double r_err = r - rd;
  const double h_err = AngleDiff(h, hd);
  const double u_err = u - ud;

  // Whether it should be braking or not
  const bool is_braking = uddot < 0.0;

  // Update the integrated error^2 terms
  state.r_err_sum_ += (r_err * r_err) * dt;
  state.h_err_sum_ += (h_err * h_err) * dt;
  state.u_err_sum_ += (u_err * u_err) * dt;

  // Compute integral of |err|^2
  // NOTABLE change from previous: r_err_sum used to be = h_err
  const double r_err2_int = state.r_err_sum_;
  const double h_err2_int = state.h_err_sum_;
  const double u_err2_int = state.u_err_sum_;

  // Eq (26-28)
  const double kappa_r = kappa_1_r + (kappa_2_r * r_err2_int);
  const double phi_r = phi_1_r + (phi_2_r * r_err2_int);
  const double tau_r = -((kappa_r * m_r) + phi_r) * r_err;

  // Eq (20-22)
  const double kappa_u = kappa_1_u + kappa_2_u * u_err2_int;
  const double phi_u = phi_1_u + phi_2_u * u_err2_int;
  const double tau_u = -((kappa_u * m_u) + phi_u) * u_err;

  // Eq (72)
  // l_f f_xf = l_r f_xr
  // f_xr = (l_f / l_r) f_xf
  // f_xf + f_xr = f_xf + (l_f / l_r) f_xf
  //             = (1 + l_f / l_r) f_xf
  // = -m*K_u*u(t) + m*K_u*u_des(t,p) +
  //    m*u^dot_des(t,p) - m*v(t)*r(t) + m*tau_u(t, p)
  const double f_xf = ((-m * k_u * u) + (m * k_u * ud) + (m * uddot) -
                       (m * v * r) + (m * tau_u)) /
                      (1.0 + l_f / l_r);

  // Eq (34)
  const double w_cmd_intermediate_term = l * f_xf / (mu_bar * m * g * l_r);
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
  const double fyr = cbar_alpha_r * alpha_r;

  // NOTABLE change from previous: h_err sign reversed in this computation
  // Eq (25)
  const double f_yf = -((i_zz * k_r * r_err) / l_f) + ((i_zz / l_f) * rddot) +
                      -((i_zz * k_h * h_err) / l_f) + ((l_r / l_f) * fyr) +
                      ((i_zz / l_f) * tau_r);
  double delta = 0.0;
  if (u > u_cri) {
    // Eq (35)
    delta = (f_yf / cbar_alpha_f) + ((v + (l_f * r)) / u);
  } else {
    // Eq (36)
    delta = (rd * (l + c_us * (u * u))) / u;
  }

  // (((1.0 / (r_w * w_cmd / u) + 1.0) * (mu_bar * m * g * l_r) / l) + fric) /
  // kt
  const double fric = c_fric_1 * std::tanh(c_fric_2 * u);
  const double numerator1 = mu_bar * m * g * l_r * (1.0 - (u / (w_cmd * r_w)));
  const double numerator2 = fric + (numerator1 / l);
  w_cmd = numerator2 / kt;

  // TODO keep?
  if (u < 0.05) {
    delta = 0;
  }

  // TODO why?
  if (ud == 0.0) {
    w_cmd = -1.0;
  }

  RoverCmd ret_val;
  ret_val.w_ = w_cmd;
  ret_val.delta_ = delta;
  ret_val.ud_ = ud;
  ret_val.uddot_ = uddot;
  ret_val.vd_ = vd;
  ret_val.vddot_ = vddot;
  ret_val.rd_ = rd;
  ret_val.hd_ = hd;
  ret_val.rddot_ = rddot;
  return ret_val;
}

}  // namespace roahm
#endif
