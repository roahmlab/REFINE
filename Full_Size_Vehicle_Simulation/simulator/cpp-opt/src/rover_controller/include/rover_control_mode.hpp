#ifndef ROAHM_ROVER_CONTROL_MODE_HPP_
#define ROAHM_ROVER_CONTROL_MODE_HPP_

/// @file rover_control_mode.hpp Contains utilities for the rover controller
/// state machine's states

namespace roahm {
/// TODO
enum class RoverControlMode {
  kTeleop,
  kRampGenParam,
  kRampUp,
  kRampChain,
  kExec,
  kExecChain,
  kExecGenParam,
  kNone
};

/// TODO
/// \param control_mode TODO
/// \return TODO
std::string ToString(const RoverControlMode& control_mode) {
  switch (control_mode) {
    case RoverControlMode::kTeleop:
      return "kTeleop";
    case RoverControlMode::kRampGenParam:
      return "kRampGenParam";
    case RoverControlMode::kRampUp:
      return "kRampUp";
    case RoverControlMode::kRampChain:
      return "kRampChain";
    case RoverControlMode::kExec:
      return "kExec";
    case RoverControlMode::kExecChain:
      return "kExecChain";
    case RoverControlMode::kExecGenParam:
      return "kExecGenParam";
    case RoverControlMode::kNone:
      return "kNone";
  }
  return "Unknown mode";
}

/// TODO
/// \param mode TODO
/// \return TODO
bool IsExecMode(RoverControlMode mode) {
  return (mode == RoverControlMode::kExec) ||
         (mode == RoverControlMode::kExecChain) ||
         (mode == RoverControlMode::kExecGenParam);
}

/// TODO
/// \param mode TODO
/// \return TODO
bool IsRampMode(RoverControlMode mode) {
  return (mode == RoverControlMode::kRampUp) ||
         (mode == RoverControlMode::kRampChain) ||
         (mode == RoverControlMode::kRampGenParam);
}

/// TODO
/// \param mode TODO
/// \return TODO
bool IsRunningAuto(RoverControlMode mode) {
  return (mode == RoverControlMode::kExec) ||
         (mode == RoverControlMode::kExecChain) ||
         (mode == RoverControlMode::kExecGenParam);
}
}  // namespace roahm
#endif  // ROAHM_ROVER_CONTROL_MODE_HPP_
