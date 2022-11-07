#ifndef ROVER_CONTROL_MODE_H_
#define ROVER_CONTROL_MODE_H_
namespace roahm {
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

bool IsExecMode(RoverControlMode mode) {
  return (mode == RoverControlMode::kExec) ||
         (mode == RoverControlMode::kExecChain) ||
         (mode == RoverControlMode::kExecGenParam);
}
bool IsRampMode(RoverControlMode mode) {
  return (mode == RoverControlMode::kRampUp) ||
         (mode == RoverControlMode::kRampChain) ||
         (mode == RoverControlMode::kRampGenParam);
}
bool IsRunningAuto(RoverControlMode mode) {
  return (mode == RoverControlMode::kExec) ||
         (mode == RoverControlMode::kExecChain) ||
         (mode == RoverControlMode::kExecGenParam);
}
}  // namespace roahm
#endif  // ROVER_CONTROL_MODE_H_
