#include "poly_predictor.hpp"

#include <cmath>     // for cos, sin
#include <cstddef>   // for size_t
#include <fstream>   // for operator<<, basic_ostream, ostream, ifstream
#include <iostream>  // for cout
#include <numeric>   // for inner_product
#include <utility>   // for swap

#include "ros/console.h"    // for LogLocation, ROS_INFO_STREAM
#include "simple_util.hpp"  // for ToAngle

namespace roahm {

namespace {
/// Computes the inner product, \f$ \langle a, b \rangle \f$, of two length-N
/// vectors \f$ a \f$ and \f$ b \f$ where \f$ a, b \f$ are both std::arrays
/// \tparam T the type contained in the arrays
/// \tparam N the length of each vector
/// \param a the first vector
/// \param b the second vector
/// \return the inner product \f$ \langle a, b \rangle \f$ of \f$ a, b \f$
template <typename T, std::size_t N>
double InnerProduct(const std::array<T, N>& a, const std::array<T, N>& b) {
  // Just a wrapper around std::inner_product to simplify code.
  return std::inner_product(a.begin(), a.end(), b.begin(), 0.0);
}

/// Loads polynomial prediction coefficients for one dimension/state, for one
/// trajectory type
/// \param f_in the input stream to read the coefficients from
/// \param state_coeffs out param to write the coefficients to
/// \param expected_val the expected header to read prior to the numerical
/// values
/// \return true iff successful
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
  const int num_coeffs = static_cast<int>(state_coeffs.size());
  for (int i = 0; i < num_coeffs; ++i) {
    double tmp_val;
    if (!(f_in >> tmp_val)) {
      return false;
    }
    state_coeffs.at(i) = tmp_val;
  }
  return true;
}

/// Loads polynomial prediction coefficients for one trajectory type, for each
/// state
/// \param f_in the input stream to read the coefficients from
/// \param state_coeffs out param to write the coefficients to
/// \return true iff successful
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

}  // namespace

RoverState StateCoeffs::GenPrediction(
    const RoverState& in_state, const std::array<double, 6>& terms) const {
  // The prediction polynomials return deltas for each state variable xyhuvr.
  // For direction and lane change maneuvers, au = u0.
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
  const double delta_u = InnerProduct(u_coeffs_, terms);
  ROS_INFO_STREAM("in_state.u_: " << in_state.u_ << ", delta u: " << delta_u);
  const double u = delta_u + u0;
  const double v = InnerProduct(v_coeffs_, terms) + v0;
  const double r = InnerProduct(r_coeffs_, terms) + r0;

  // Since we have no prediction polynomial for w, approximate it as u
  // This currently doesn't make a difference for polynomials anyways.
  // TODO
  const double w = u;

  RoverState ret_val{x_out, y_out, 0.0, u, v, r, w};
  ret_val.SetHeading(h_out);
  return ret_val;
}

PolyPredictor::PolyPredictor(const std::string& fname) {
  // Load polynomial prediction coefficients from file.
  // Set successful_ flag to true if successful, false otherwise.
  std::cout << "Loading " << fname << std::endl;
  successfully_loaded_ = false;
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
  successfully_loaded_ = true;
  std::cout << "Successfully loaded poly predictor\n";
}

RoverState PolyPredictor::Predict(const RoverState& in_state,
                                  const ManuType manu_type, double p0,
                                  double p1, const bool use_15_30) const {
  // Variables for polynomials:
  //   Spd:     (p0, p1) = (Au, u0)
  //   Dir/Lan: (p0, p1) = (Au, Ay)
  ROS_INFO_STREAM("Predicting [" << ToString(manu_type) << "] [p0: " << p0
                                 << "] [p1:" << p1 << "]");
  // TODO: this is just due to an error in the computed data
  if (manu_type == ManuType::kSpdChange) {
    std::swap(p0, p1);
  }
  const std::array<double, 6> terms = {p0 * p0, p0 * p1, p0, p1 * p1, p1, 1.0};
  if (manu_type == ManuType::kSpdChange) {
    return spd_coeffs_.GenPrediction(in_state, terms);
  } else if (manu_type == ManuType::kDirChange) {
    return dir_coeffs_.GenPrediction(in_state, terms);
  } else /*if (manu_type == ManuType::kLanChange)*/ {
    if (!use_15_30) {
      return lan_coeffs_00_15_.GenPrediction(in_state, terms);
    } else {
      return lan_coeffs_15_30_.GenPrediction(in_state, terms);
    }
  }
}

RoverState PolyPredictor::Predict(const PredictInfo& predict_info) const {
  RoverState state_0 = predict_info.GetInitRoverState();
  RoverState pred_state = state_0;
  const bool should_predict = predict_info.should_predict_;
  if (should_predict) {
    const auto manu_type = predict_info.manu_type_;
    const bool use_15_30 = predict_info.use_15_30_;
    const double p0 = predict_info.poly_var0_;
    // TODO REMOVE this multiplier. It only exists because of a one-time issue
    // with Hansen's latest polynomials
    const double p1_mult = 1.0;  // IsDirLan(manu_type) ? -1.0 : 1.0;
    const double p1 = p1_mult * predict_info.poly_var1_;
    pred_state = Predict(state_0, manu_type, p0, p1, use_15_30);
    ROS_INFO_STREAM("Predicted from: "
                    << predict_info.GetInitRoverState().ToStringXYHUVR());
    ROS_INFO_STREAM("Predicted to:   " << pred_state.ToStringXYHUVR());
    ROS_INFO_STREAM("PRED_P0: " << p0);
    ROS_INFO_STREAM("PRED_P1: " << p1);
    ROS_INFO_STREAM("Prediction p0/p1/manu/1530: " << p0 << " / " << p1 << " / "
                                                   << ToString(manu_type)
                                                   << " / " << use_15_30);
  } else {
    ROS_INFO_STREAM("Not predicting, just using state_0");
  }
  ROS_INFO_STREAM("Predicted to " << pred_state.ToStringXYHUVR());
  return pred_state;
}

}  // namespace roahm