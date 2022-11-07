#ifndef ROAHM_TRAJECTORY_SYM_
#define ROAHM_TRAJECTORY_SYM_
#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <iostream>
#include <numeric>
#include <vector>

#include "consts.h"
#include "rmanu_type.h"

namespace roahm {

struct Trajectory {
 public:
  struct TimeBound {
    double lb_;  // Lower bound
    double ub_;  // Upper bound
    TimeBound(double lb, double ub) : lb_{lb}, ub_{ub} {}
    double Lb() const { return lb_; }
    double Ub() const { return ub_; }
  };
  struct TrajectoryValue {
    double ud_;
    double vd_;
    double rd_;
    double uddot_;
    double vddot_;
    double rddot_;
    double hd_;
    TrajectoryValue(double ud, double vd, double rd, double uddot, double vddot,
                    double rddot, double hd)
        : ud_{ud},
          vd_{vd},
          rd_{rd},
          uddot_{uddot},
          vddot_{vddot},
          rddot_{rddot},
          hd_{hd} {}
    friend std::ostream& operator<<(std::ostream& out,
                                    const TrajectoryValue& v) {
      return out << v.ud_ << ", " << v.vd_ << ", " << v.rd_ << ", " << v.uddot_
                 << ", " << v.vddot_ << ", " << v.rddot_ << ", " << v.hd_
                 << std::endl;
    }
  };

  double t_min_;
  double au_;
  double ay_;
  double u0_;
  double h0_;
  double u0_goal_;
  RManuType manu_type_;
  double t0_offset_;

 private:
  struct ComputedTrajInfo {
    double delta_spd_;
    double t_to_use_;
    double tbrk1_;
    double tbrk2_;
    double t_eval_max_;
  };

 public:
  constexpr static double kDoubleInfty =
      std::numeric_limits<double>::infinity();
  constexpr static double kAuMin = 0.5;
  constexpr static double kFiniteDiffTime = 1.0e-6;
  constexpr static double kFourThirdPi = 4.0 * M_PI / 3.0;
  constexpr static double kHalfPi = 0.5 * M_PI;
  Trajectory(double au, double ay, double u0_goal, RManuType manu_type)
      : Trajectory(au, ay, 0.0, u0_goal, manu_type, 0.0, 0.0) {}
  // Trajectory(double au, double ay, double u0, double u0_goal,
  //           RManuType manu_type)
  //    : Trajectory(au, ay, u0, u0_goal, manu_type, 0.0, 0.0) {}
  Trajectory(double au, double ay, double u0, double u0_goal,
             RManuType manu_type, double t0_offset)
      : Trajectory(au, ay, u0, u0_goal, manu_type, t0_offset, 0.0) {}
  Trajectory(double au, double ay, double u0, double u0_goal,
             RManuType manu_type, double t0_offset, double h0)
      : t_min_{0.0},
        au_{Clamp(au, kAuMin, kDoubleInfty)},
        ay_{ay},
        u0_{u0},
        h0_{h0},
        u0_goal_{u0_goal},
        manu_type_{manu_type},
        t0_offset_{t0_offset} {}
  Trajectory() : Trajectory(0.0, 0.0, 0.0, RManuType::kNone) {}

  TrajectoryValue EvalAt(double t) const {
    t = Clamp(t, t_min_, TEvalMax());
    double ud = Ud(t);
    double vd = Vd(t);
    double rd = Rd(t);
    double uddot = UdDot(t);
    double vddot = VdDot(t);
    double rddot = RdDot(t);
    double hd = Hd(t);
    return {ud, vd, rd, uddot, vddot, rddot, hd};
  }
  double MaxTrajTime() const { return TEvalMax(); }
  bool HasValidType() const { return IsValid(manu_type_); }

 private:
  ComputedTrajInfo ComputeInfo() const {
    ComputedTrajInfo cti;
    cti.delta_spd_ = Clamp(au_ - RtdConsts::kUReallySlow, 0, kDoubleInfty);
    cti.t_to_use_ = OneHump() ? RtdConsts::kTpkDir : RtdConsts::kTpk;
    cti.tbrk1_ = cti.delta_spd_ / RtdConsts::kAMax;
    cti.tbrk2_ = 1.0;
    cti.t_eval_max_ = cti.t_to_use_ + cti.tbrk1_ + cti.tbrk2_;
    return cti;
  }

  // These can be converted to member variables instead iff there is no
  // place where the parameters are updated after they are computed.
  double TBrk1() const { return ComputeInfo().tbrk1_; }
  double TBrk2() const { return ComputeInfo().tbrk2_; }
  double TToUse() const { return ComputeInfo().t_to_use_; }
  double DeltaSpd() const { return ComputeInfo().delta_spd_; }
  double TEvalMax() const { return ComputeInfo().t_eval_max_; }
  double T1Max() const { return TToUse(); }
  double T2Max() const { return T1Max() + TBrk1(); }
  double T3Max() const { return T2Max() + TBrk2(); }
  // TODO
  bool OneHump() const { return not IsLan(manu_type_); }
  bool TimeInRange(double t) const { return t >= t_min_ and t <= TEvalMax(); }

  int RangeNo(double t) const {
    if (t < 0) {
      return -1;
    } else if (t < T1Max()) {
      return 1;
    } else if (t < T2Max()) {
      return 2;
    } else if (t < T3Max()) {
      return 3;
    }
    return 4;
  }
  bool TimeInSameRange(double t0, double t1) const {
    return RangeNo(t0) == RangeNo(t1);
  }
  bool TimeBoundIsOk(TimeBound bound) const {
    const bool lb_ok = TimeInRange(bound.Lb());
    const bool ub_ok = TimeInRange(bound.Ub());
    const bool bounds_same_rng = TimeInSameRange(bound.Lb(), bound.Ub());
    return lb_ok & ub_ok & bounds_same_rng;
  }
  TimeBound FiniteDiffTimes(double t) const {
    // Return times to evaluate function at
    // Assumptions:
    //   There exists some bound [a, b] such that:
    //     a <= t <= b
    //     b - a = kFiniteDiffTime
    //     a, b in [t_min_, t_max_]
    //     a, b in the same time "range" (t1, brake, after brake)

    constexpr double kHalfDiffTime = kFiniteDiffTime / 2.0;
    const TimeBound centered(t - kHalfDiffTime, t + kHalfDiffTime);
    const TimeBound left_biased(t - kFiniteDiffTime, t);
    const TimeBound right_biased(t, t + kFiniteDiffTime);
    if (TimeBoundIsOk(centered)) {
      return centered;
    } else if (TimeBoundIsOk(left_biased)) {
      return left_biased;
    } else if (TimeBoundIsOk(right_biased)) {
      return right_biased;
    }
    std::cerr << "NOTHING OKAY" << std::endl;
    return centered;
  }

  double Ud(double t) const {
    if (t <= T1Max()) {
      return ((au_ - u0_) / TToUse()) * t + u0_;
    } else if (t <= T2Max()) {
      return au_ - ((au_ - RtdConsts::kUReallySlow) / TBrk1()) * (t - TToUse());
    }
    return 0.0;
  }

  double Rd(double t) const {
    if (t <= T1Max()) {
      if (OneHump()) {
        return ay_ * 0.5 * (std::sin(t * kFourThirdPi - kHalfPi) + 1.0);
      } else {
        return -ay_ * std::exp(-2.7 * Square(t - 1.5)) * (4.0 * t - 6.0);
      }
    }
    return 0.0;
  }

  double Vd(double t) const { return Rd(t) / 13.0; }

  double Hd(double t) const {
    // TODO +h0
    if (t <= T1Max()) {
      double ret_val = 0.0;
      if (OneHump()) {
        ret_val =
            ay_ * (t / 2.0 - (3.0 / (8.0 * M_PI)) * std::sin(kFourThirdPi * t));
      } else {
        ret_val = -ay_ *
                  (-20.0 / 27.0 * std::exp(-27.0 / 40.0 * Square(3.0 - 2 * t)));
      }
      return ToAngle(ret_val + h0_);
    }
    // hd should remain constant during braking
    return Hd(T1Max());
  }

  //
  // As long as these are based on differences, they don't vary based on
  // maneuver type.
  //

  double UdDot(double t) const {
    const auto eval_times = FiniteDiffTimes(t);
    const double delta = (Ud(eval_times.Ub()) - Ud(eval_times.Lb()));
    return delta / kFiniteDiffTime;
  }

  double VdDot(double t) const {
    const auto eval_times = FiniteDiffTimes(t);
    const double delta = (Vd(eval_times.Ub()) - Vd(eval_times.Lb()));
    return delta / kFiniteDiffTime;
  }

  double RdDot(double t) const {
    const auto eval_times = FiniteDiffTimes(t);
    const double delta = (Rd(eval_times.Ub()) - Rd(eval_times.Lb()));
    return delta / kFiniteDiffTime;
  }
};
}  // namespace roahm
#endif  // ROAHM_TRAJECTORY_SYM_
