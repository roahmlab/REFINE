#ifndef ROAHM_CONSTS_HPP_
#define ROAHM_CONSTS_HPP_
namespace roahm {

/// @file consts.hpp Contains constants for the controller
namespace RtdConsts {

// Controller Outputs
/// Maximum servo command angle [radians]
constexpr static double kMaxServoCmd = 0.3;
/// TODO
constexpr static double kMaxFyrUncertainty = 5;

/// TODO
constexpr static double kAMax = 1.5;
/// TODO
constexpr static double kTpk = 3;
/// TODO
constexpr static double kTpkDir = 1.5;
/// TODO
constexpr static double kUReallySlow = 0.5;

// Controller constants needed across multiple files
/// TODO
constexpr static double kMuBar = 0.95;
/// TODO
constexpr static double kGravConst = 9.8;
/// TODO
constexpr static double kRoverMass = 4.956;
/// TODO
constexpr static double kLr = 0.107;
/// TODO
constexpr static double kLengthTotal = 0.31;
/// TODO
constexpr static double kWheelRadius = 0.05461;

/// Controller loop frequency [Hz]
constexpr static double kControlLoopFreqHz = 50;
static_assert(kControlLoopFreqHz > 0, "Control loop frequency must be > 0 Hz");
/// Time delta between controller loops [seconds]
constexpr static double kControlLoopDt = 1.0 / kControlLoopFreqHz;
};  // namespace RtdConsts
}  // namespace roahm
#endif
