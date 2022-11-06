#include "trajectory_sym.hpp"

#include "consts.hpp"  // for kUReallySlow, kAMax, kTpk, kTpkDir

namespace roahm {

double Trajectory::MaxTrajTime() const { return TEvalMax(); }
bool Trajectory::HasValidType() const { return IsValid(manu_type_); }

TrajectoryValue Trajectory::EvalAt(double t) const {
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

Trajectory::ComputedTrajInfo Trajectory::ComputeInfo() const {
  ComputedTrajInfo cti{};
  cti.delta_spd_ = Clamp(au_ - RtdConsts::kUReallySlow, 0, kDoubleInfty);
  cti.t_to_use_ = OneHump() ? RtdConsts::kTpkDir : RtdConsts::kTpk;
  cti.tbrk1_ = cti.delta_spd_ / RtdConsts::kAMax;
  cti.tbrk2_ = 1.0;
  cti.t_eval_max_ = cti.t_to_use_ + cti.tbrk1_ + cti.tbrk2_;
  return cti;
}
double Trajectory::TBrk1() const { return ComputeInfo().tbrk1_; }
double Trajectory::TBrk2() const { return ComputeInfo().tbrk2_; }
double Trajectory::TToUse() const { return ComputeInfo().t_to_use_; }
double Trajectory::TEvalMax() const { return ComputeInfo().t_eval_max_; }
double Trajectory::T1Max() const { return TToUse(); }
double Trajectory::T2Max() const { return T1Max() + TBrk1(); }
double Trajectory::T3Max() const { return T2Max() + TBrk2(); }
bool Trajectory::OneHump() const { return not IsLan(manu_type_); }
bool Trajectory::TimeInRange(double t) const {
  return t >= t_min_ and t <= TEvalMax();
}
int Trajectory::RangeNo(double t) const {
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
bool Trajectory::TimeInSameRange(double t0, double t1) const {
  return RangeNo(t0) == RangeNo(t1);
}
bool Trajectory::TimeIntervalIsOk(Interval time_interval) const {
  const bool min_ok = TimeInRange(time_interval.Min());
  const bool max_ok = TimeInRange(time_interval.Max());
  const bool bounds_same_rng =
      TimeInSameRange(time_interval.Min(), time_interval.Max());
  return min_ok and max_ok and bounds_same_rng;
}
Interval Trajectory::FiniteDiffTimes(double t) const {
  constexpr static double kFiniteDiffTime = 1.0e-6;
  constexpr static double kHalfDiffTime = kFiniteDiffTime / 2.0;
  // Return times to evaluate function at
  // Assumptions:
  //   There exists some bound [a, b] such that:
  //     a <= t <= b
  //     b - a = kFiniteDiffTime
  //     a, b in [t_min_, t_max_]
  //     a, b in the same time "range" (t1, brake, after brake)

  const Interval centered(t - kHalfDiffTime, t + kHalfDiffTime);
  const Interval left_biased(t - kFiniteDiffTime, t);
  const Interval right_biased(t, t + kFiniteDiffTime);
  if (TimeIntervalIsOk(centered)) {
    return centered;
  } else if (TimeIntervalIsOk(left_biased)) {
    return left_biased;
  } else if (TimeIntervalIsOk(right_biased)) {
    return right_biased;
  }
  std::cerr << "NOTHING OKAY" << std::endl;
  return centered;
}
double Trajectory::Ud(double t) const {
  if (t <= T1Max()) {
    return ((au_ - u0_) / TToUse()) * t + u0_;
  } else if (t <= T2Max()) {
    return au_ - ((au_ - RtdConsts::kUReallySlow) / TBrk1()) * (t - TToUse());
  }
  return 0.0;
}

double Trajectory::Rd(double t) const {
  constexpr static double kHalfPi = 0.5 * M_PI;
  if (t <= T1Max()) {
    if (OneHump()) {
      return ay_ * 0.5 * (std::sin(t * kFourThirdPi - kHalfPi) + 1.0);
    } else {
      return -ay_ * std::exp(-2.7 * Square(t - 1.5)) * (4.0 * t - 6.0);
    }
  }
  return 0.0;
}

double Trajectory::Vd(double t) const { return Rd(t) / 13.0; }

double Trajectory::Hd(double t) const {
  t = Clamp(t, 0.0, T1Max());
  double ret_val{};
  if (OneHump()) {
    ret_val =
        ay_ * (t / 2.0 - (3.0 / (8.0 * M_PI)) * std::sin(kFourThirdPi * t));
  } else {
    ret_val =
        -ay_ * (-20.0 / 27.0 * std::exp(-27.0 / 40.0 * Square(3.0 - 2 * t)));
  }
  return ToAngle(ret_val + h0_);
}

double Trajectory::UdDot(double t) const {
  const auto eval_times = FiniteDiffTimes(t);
  const double delta = (Ud(eval_times.Max()) - Ud(eval_times.Min()));
  return delta / eval_times.Width();
}

double Trajectory::VdDot(double t) const {
  const auto eval_times = FiniteDiffTimes(t);
  const double delta = (Vd(eval_times.Max()) - Vd(eval_times.Min()));
  return delta / eval_times.Width();
}

double Trajectory::RdDot(double t) const {
  const auto eval_times = FiniteDiffTimes(t);
  const double delta = (Rd(eval_times.Max()) - Rd(eval_times.Min()));
  return delta / eval_times.Width();
}
}  // namespace roahm
