#ifndef ROAHM_POLY_PREDICTOR_HPP_
#define ROAHM_POLY_PREDICTOR_HPP_

#include <array>   // for array
#include <string>  // for string

#include "manu_type.hpp"    // for ManuType, ManuType::kSpdChange
#include "rover_state.hpp"  // for RoverState

/// @file poly_predictor.hpp contains functions to run polynomial prediction
/// of future states

namespace roahm {

/// Contains the information necessary to compute a polynomial prediction
struct PredictInfo {
  // Variables for polynomials:
  //   Spd:     (poly_var0_, poly_var1_) = (Au, u0)
  //   Dir/Lan: (poly_var0_, poly_var1_) = (Au, Ay)

  /// First polynomial variable, Au
  double poly_var0_;

  /// Second polynomial variable, u0 for speed changes and Ay for dir/lane
  /// changes
  double poly_var1_;

  // TODO could just make these all one state
  // State at start of trajectory
  /// The initial x location [m]
  double x0_;
  /// The initial y location [m]
  double y0_;
  /// The initial heading [rad/s]
  double h0_;
  /// The initial longitudinal velocity [m/s]
  double u0_;
  /// The initial lateral velocity [m/s]
  double v0_;
  /// The initial yaw rate [rad/s]
  double r0_;

  /// Whether to use the 1.5-3 second prediction polynomial, e.g. for second
  /// half of a lane change
  bool use_15_30_;
  /// Whether to actually predict the future state or not.
  /// This could be set to false if, for example, we get a starting state
  bool should_predict_;
  /// The manuever type to predict
  ManuType manu_type_;
  /// Default constructor
  PredictInfo()
      : poly_var0_{0.0},
        poly_var1_{0.0},
        x0_{0.0},
        y0_{0.0},
        h0_{0.0},
        u0_{0.0},
        v0_{0.0},
        r0_{0.0},
        use_15_30_{false},
        should_predict_{false},
        manu_type_{ManuType::kSpdChange} {}

  /// Returns the initial state
  /// \return the initial state
  RoverState GetInitRoverState() const {
    return {x0_, y0_, h0_, u0_, v0_, r0_, u0_};
  }
};

/// TODO
struct StateCoeffs {
  // Coefficients for each state variable's prediction polynomials
  /// The polynomial coefficients to generate the \f$ \Delta x \f$ prediction
  std::array<double, 6> x_coeffs_;
  /// The polynomial coefficients to generate the \f$ \Delta y \f$ prediction
  std::array<double, 6> y_coeffs_;
  /// The polynomial coefficients to generate the \f$ \Delta h \f$ prediction
  std::array<double, 6> h_coeffs_;
  /// The polynomial coefficients to generate the \f$ \Delta u \f$ prediction
  std::array<double, 6> u_coeffs_;
  /// The polynomial coefficients to generate the \f$ \Delta v \f$ prediction
  std::array<double, 6> v_coeffs_;
  /// The polynomial coefficients to generate the \f$ \Delta r \f$ prediction
  std::array<double, 6> r_coeffs_;

  // TODO better description than "amount to turn"

  /// Generates a predicted state, \f$ x_\text{new} = x_\text{init} +
  /// \begin{bmatrix}
  ///   x_\text{coeffs}^\top \\
  ///   y_\text{coeffs}^\top \\
  ///   h_\text{coeffs}^\top \\
  ///   u_\text{coeffs}^\top \\
  ///   v_\text{coeffs}^\top \\
  ///   r_\text{coeffs}^\top \\
  /// \end{bmatrix}
  /// \begin{bmatrix}
  /// p_0^2 \\ p_0 p_1 \\ p_0 \\ p_1^2 \\ p_1 \\ 1
  /// \end{bmatrix} \f$
  /// \param in_state the initial state \f$ x_\text{init} \f$ of the
  /// trajectory whose parameters correspond to the given terms
  /// \param terms the polynomial variables \f$ (p_0, p_1) \f$, which are
  /// defined as \f$ (Au, u_0) \f$ for speed changes [where \f$ u_0 \f$ is the
  /// initial speed and \f$ Au \f$ is the desired speed],
  /// and \f$ (Au, Ay) \f$ for direction and lane changes [where \f$ Au \f$ is
  /// the speed to maintain, and \f$ Ay \f$ is the "amount" to turn]
  /// \return the predicted state at the end of the trajectory
  RoverState GenPrediction(const RoverState& in_state,
                           const std::array<double, 6>& terms) const;
};

/// Computes polynomial prediction given coefficients, initial state, and
/// trajectory information
class PolyPredictor {
 private:
  /// The coefficients to use for speed change predictions
  StateCoeffs spd_coeffs_;
  /// The polynomial coefficients to use for direction change coefficients
  StateCoeffs dir_coeffs_;
  /// The polynomial coefficients to use for the first portion of lane changes
  StateCoeffs lan_coeffs_00_15_;
  /// The polynomial coefficients to use for the second portion of lane changes
  StateCoeffs lan_coeffs_15_30_;
  /// Whether the coefficients were successfully loaded
  bool successfully_loaded_;

 public:
  /// Returns true iff the polynomial prediction coefficients were successfully
  /// loaded
  /// \return true iff the coefficients were successfully loaded on construction
  bool SuccessfullyLoaded() const { return successfully_loaded_; }

  /// Constructor
  /// \param fname the file to load the coefficients from
  PolyPredictor(const std::string& fname);

  /// Generates a prediction of the state after running a trajectory for a fixed
  /// amount of time
  /// \param in_state the starting state
  /// \param manu_type the trajectory manuever type
  /// \param p0 the first polynomial variable, Au
  /// \param p1 the second polynomial variable, u0 for speed changes and Ay
  /// for dir/lane changes
  /// \param use_15_30 whether to use the second half of longer trajectories
  /// \return the predicted state after the duration of the trajectory execution
  RoverState Predict(const RoverState& in_state, ManuType manu_type, double p0,
                     double p1, bool use_15_30) const;

  /// Predict the future rover state given prediction information
  /// \param predict_info the trajectory and state information needed to predict
  /// the future state
  /// \return the predicted future rover state after a fixed duration of
  /// trajectory execution
  RoverState Predict(const PredictInfo& predict_info) const;
};
}  // namespace roahm
#endif  // ROAHM_POLY_PREDICTOR_HPP_
