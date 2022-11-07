#ifndef ROAHM_POLY_PREDICTOR_H_
#define ROAHM_POLY_PREDICTOR_H_
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <numeric>
#include <string>

#include "complex_heading_state.h"

namespace roahm {

struct PredictInfo {
  // Variables for polynomials:
  //   Spd:     (poly_var0_, poly_var1_) = (Au, u0)
  //   Dir/Lan: (poly_var0_, poly_var1_) = (Au, Ay)
  double poly_var0_;
  double poly_var1_;

  // State at start of trajectory
  double x0_;
  double y0_;
  double h0_;
  double u0_;
  double v0_;
  double r0_;

  int t0_idx_c_style_;
  bool use_15_30_;
  // This could be set to false if, for example, we get a starting state
  // that is stationary.
  bool should_predict_;
  RManuType manu_type_;
  PredictInfo()
      : poly_var0_{0.0},
        poly_var1_{0.0},
        x0_{0.0},
        y0_{0.0},
        h0_{0.0},
        u0_{0.0},
        v0_{0.0},
        r0_{0.0},
        t0_idx_c_style_{0},
        use_15_30_{false},
        should_predict_{false},
        manu_type_{RManuType::kSpdChange} {}

  ComplexHeadingState ToComplexHeadingState() const {
    return ComplexHeadingState(x0_, y0_, h0_, u0_, v0_, r0_, 0.0, 0.0, 0.0);
  }
  std::string StateString() const {
    return StringXYHUVR(x0_, y0_, h0_, u0_, v0_, r0_);
  }
};

template <typename T, std::size_t N>
double InnerProduct(const std::array<T, N>& a, const std::array<T, N>& b) {
  // Just a wrapper around std::inner_product to simplify code.
  return std::inner_product(a.begin(), a.end(), b.begin(), 0.0);
}

struct StateCoeffs {
  // Coefficients for each state variable's prediction polynomials
  std::array<double, 6> x_coeffs_;
  std::array<double, 6> y_coeffs_;
  std::array<double, 6> h_coeffs_;
  std::array<double, 6> u_coeffs_;
  std::array<double, 6> v_coeffs_;
  std::array<double, 6> r_coeffs_;

  ComplexHeadingState GenPrediction(const ComplexHeadingState& in_state,
                                    const std::array<double, 6>& terms) const {
    // The prediction polynomials return deltas for each state variable xyhuvr.
    // For direction and lane change manuevers, au = u0.
    //
    // Since x, y and h return the delta in the local frame of reference, they
    // must be transformed into the global frame of reference.
    //
    // u, v, and r do not change with rotation, so they can just be added to
    // the current state.
    const double in_heading = in_state.GetHeading();
    const double dh = InnerProduct(h_coeffs_, terms);
    const double h_out = ToAngle(in_heading + dh);

    // Convert local delta x, y to deltas in the world frame
    const double dx_local = InnerProduct(x_coeffs_, terms);
    const double dy_local = InnerProduct(y_coeffs_, terms);
    const double cos_h = std::cos(in_heading);
    const double sin_h = std::sin(in_heading);
    const double dx_world = cos_h * dx_local - sin_h * dy_local;
    const double dy_world = sin_h * dx_local + cos_h * dy_local;
    const double x_out = in_state.x_ + dx_world;
    const double y_out = in_state.y_ + dy_world;

    const double u0 = in_state.u_;
    const double v0 = in_state.v_;
    const double r0 = in_state.r_;
    const double u = InnerProduct(u_coeffs_, terms) + u0;
    const double v = InnerProduct(v_coeffs_, terms) + v0;
    const double r = InnerProduct(r_coeffs_, terms) + r0;

    // Since we have no prediction polynomial for w, approximate it as u
    // This currently doesn't make a difference for polynomials anyways.
    // TODO
    const double w = u;

    ComplexHeadingState ret_val{x_out, y_out, 0.0, u, v, r, w, 0.0, 0.0};
    ret_val.SetHeading(h_out);
    return ret_val;
  }
};

bool LoadSingleStateCoeffs(std::istream& f_in,
                           std::array<double, 6>& state_coeffs,
                           std::string expected_val) {
  // Load one state variable's coefficients.
  // Return true if successful, false otherwise.
  std::string in_str;
  f_in >> in_str;
  if (in_str != expected_val) {
    std::cout << "in_str != \"" << expected_val << "\" ('" << in_str << "')\n";
    return false;
  }
  for (int i = 0; i < state_coeffs.size(); ++i) {
    double tmp_val;
    if (!(f_in >> tmp_val)) {
      return false;
    }
    state_coeffs.at(i) = tmp_val;
  }
  return true;
}

bool LoadStateCoeffs(std::istream& f_in, StateCoeffs& state_coeffs) {
  // Load one set of state coefficients from input stream.
  // Return true if successful, false otherwise.
  return LoadSingleStateCoeffs(f_in, state_coeffs.x_coeffs_, "X") and
         LoadSingleStateCoeffs(f_in, state_coeffs.y_coeffs_, "Y") and
         LoadSingleStateCoeffs(f_in, state_coeffs.h_coeffs_, "H") and
         LoadSingleStateCoeffs(f_in, state_coeffs.u_coeffs_, "U") and
         LoadSingleStateCoeffs(f_in, state_coeffs.v_coeffs_, "V") and
         LoadSingleStateCoeffs(f_in, state_coeffs.r_coeffs_, "R");
}

class PolyPredictor {
 private:
  StateCoeffs spd_coeffs_;
  StateCoeffs dir_coeffs_;
  StateCoeffs lan_coeffs_00_15_;
  StateCoeffs lan_coeffs_15_30_;
  bool successful_;

 public:
  bool SuccessfullyLoaded() const { return successful_; }

  PolyPredictor(std::string fname) {
    // Load polynomial prediction coefficients from file.
    // Set successful_ flag to true if successful, false otherwise.
    std::cout << "Loading " << fname << std::endl;
    successful_ = false;
    std::ifstream f_in(fname);
    std::string in_str;
    f_in >> in_str;
    if (in_str != "SPD") {
      std::cout << "Failed to load, in_str != \"SPD\" ('" << in_str << "')\n";
      return;
    }
    if (!LoadStateCoeffs(f_in, spd_coeffs_)) {
      std::cout << "Failed loading speed polynomial coefficients\n";
      return;
    }

    f_in >> in_str;
    if (in_str != "DIR") {
      std::cout << "Failed to load, in_str != \"DIR\"\n";
      return;
    }
    if (!LoadStateCoeffs(f_in, dir_coeffs_)) {
      std::cout << "Failed loading dir polynomial coefficients\n";
      return;
    }

    f_in >> in_str;
    if (in_str != "LAN00_15") {
      std::cout << "Failed to load, in_str != \"LAN00_15\"\n";
      return;
    }
    if (!LoadStateCoeffs(f_in, lan_coeffs_00_15_)) {
      std::cout << "Failed loading lan 0.0-1.5s polynomial coefficients\n";
      return;
    }
    f_in >> in_str;
    if (in_str != "LAN15_30") {
      std::cout << "Failed to load, in_str != \"LAN15_30\"\n";
      return;
    }
    if (!LoadStateCoeffs(f_in, lan_coeffs_15_30_)) {
      std::cout << "Failed loading lan 1.5-3.0s polynomial coefficients\n";
      return;
    }
    f_in.close();
    successful_ = true;
    std::cout << "Successfully loaded poly predictor\n";
  }
  ComplexHeadingState Predict(const ComplexHeadingState& in_state,
                              const RManuType manu_type, const double p0,
                              const double p1, const bool use_15_30) const {
    // Variables for polynomials:
    //   Spd:     (p0, p1) = (Au, u0)
    //   Dir/Lan: (p0, p1) = (Au, Ay)
    ROS_INFO_STREAM("Predicting [" << ToString(manu_type) << "] [p0: " << p0
                                   << "] [p1:" << p1 << "]");
    const std::array<double, 6> terms = {p0 * p0, p0 * p1, p0,
                                         p1 * p1, p1,      1.0};
    if (manu_type == RManuType::kSpdChange) {
      return spd_coeffs_.GenPrediction(in_state, terms);
    } else if (manu_type == RManuType::kDirChange) {
      return dir_coeffs_.GenPrediction(in_state, terms);
    } else /*if (manu_type == RManuType::kLanChange)*/ {
      if (!use_15_30) {
        return lan_coeffs_00_15_.GenPrediction(in_state, terms);
      } else {
        return lan_coeffs_15_30_.GenPrediction(in_state, terms);
      }
    }
  }

  ComplexHeadingState Predict(const PredictInfo& predict_info) const {
    ComplexHeadingState state_0 = predict_info.ToComplexHeadingState();
    ComplexHeadingState pred_state = state_0;
    const bool should_predict = predict_info.should_predict_;
    if (should_predict) {
      const auto manu_type = predict_info.manu_type_;
      const bool use_15_30 = predict_info.use_15_30_;
      const double p0 = predict_info.poly_var0_;
      // TODO REMOVE this multiplier. It only exists because of a one-time issue
      // with Hansen's latest polynomials
      const double p1_mult = IsDirLan(manu_type) ? -1.0 : 1.0;
      const double p1 = p1_mult * predict_info.poly_var1_;
      pred_state = Predict(state_0, manu_type, p0, p1, use_15_30);
      ROS_INFO_STREAM("Predicted from: " << predict_info.StateString());
      ROS_INFO_STREAM("Predicted to:   " << pred_state.ToStringXYHUVR());
      ROS_INFO_STREAM("PRED_P0: " << p0);
      ROS_INFO_STREAM("PRED_P1: " << p1);
      ROS_INFO_STREAM("Prediction p0/p1/manu/1530: "
                      << p0 << " / " << p1 << " / " << ToString(manu_type)
                      << " / " << use_15_30);
    } else {
      TGUARD(ROS_INFO_STREAM("Not predicting, just using state_0"));
    }
    TGUARD_ROS_INFO_STREAM("Predicted to " << pred_state.ToStringXYHUVR());
    return pred_state;
  }
};
}  // namespace roahm
#endif  // ROAHM_POLY_PREDICTOR_H_
