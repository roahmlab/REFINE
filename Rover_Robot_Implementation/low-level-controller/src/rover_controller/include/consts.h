#ifndef ROAHM_CONSTS_H_
#define ROAHM_CONSTS_H_
namespace roahm {

namespace RtdConsts {

// Controller Outputs
constexpr double kMaxServoCmd = 0.3;        // [rad]
constexpr double kMaxServoCmdTeleop = 0.3;  // [rad]

// Controller Gains
constexpr static double kAMax = 1.5;
constexpr static double kTpk = 3;
constexpr static double kTbrk = 2;
constexpr static double kTpkDir = 1.5;
constexpr static double kUReallySlow = 0.5;

constexpr double kControlLoopFreqHz = 50;
static_assert(kControlLoopFreqHz > 0, "Control loop frequency must be > 0 Hz");
constexpr double kControlLoopDt = 1.0 / kControlLoopFreqHz;
};  // namespace RtdConsts
}  // namespace roahm
#endif
