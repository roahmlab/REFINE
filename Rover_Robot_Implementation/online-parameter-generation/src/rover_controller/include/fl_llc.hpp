#ifndef ROAHM_FL_LLC_HPP_
#define ROAHM_FL_LLC_HPP_
#include <cstdlib>  // for size_t

#include "rover_state.hpp"     // for RoverState
#include "trajectory_sym.hpp"  // for Trajectory

/// @file fl_llc.hpp Feedback linearization low level controller

namespace roahm {

/// The speed and steering commands to give the rover, along with the desired
/// values associated with the trajectory it is following, if applicable.
struct RoverCmd {
  /// Commanded steering command [rad]
  double delta_;
  /// Commanded wheel speed [m/s]
  double w_;
  /// The desired longitudinal vehicle speed [m/s]
  double ud_;
  /// The desired longitudinal vehicle acceleration [m/(s^2)]
  double uddot_;
  /// The desired lateral vehicle speed [m/s]
  double vd_;
  /// The desired lateral vehicle acceleration [m/(s^2)]
  double vddot_;
  /// The desired yaw rate [rad/s]
  double rd_;
  /// The desired heading [rad]
  double hd_;
  /// The desired yaw acceleration [rad/(s^2)]
  double rddot_;
};

/// The available control modes for the rover. Note: speed control mode is
/// the only mode that allows the rover to drive backwards and should be used
/// for human driving.
enum class RoverDriveMode { kCurrentControl, kSpeedControl };

/// Runs the low level feedback linearization controller
/// \param state the current rover state
/// \param dt the time delta between loops
/// \param t the time along the trajectory
/// \param traj the currently executing trajectory
/// \param drive_mode_ whether speed or current control mode are being used
/// \return the controller's output commands to be sent to the rover
RoverCmd FeedbackLinController(RoverState& state, double dt, double t,
                               const Trajectory& traj,
                               const RoverDriveMode& drive_mode_);

}  // namespace roahm
#endif
