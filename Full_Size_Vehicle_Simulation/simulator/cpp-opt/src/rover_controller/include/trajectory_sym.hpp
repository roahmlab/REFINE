#ifndef ROAHM_TRAJECTORY_SYM_HPP_
#define ROAHM_TRAJECTORY_SYM_HPP_

#include <cmath>     // for M_PI
#include <iostream>  // for ostream, endl, basic_ostream
#include <limits>    // for numeric_limits
#include <string>    // for operator<<

#include "manu_type.hpp"    // for ManuType, ManuType::kNone
#include "simple_util.hpp"  // for Clamp, Interval, VariadicListToStr

/// @file trajectory_sym.hpp Contains structures and functions to represent
/// and evaluate trajectories symbolically

namespace roahm {

/// The desired trajectory values at one time step
struct TrajectoryValue {
  /// The desired longitudinal speed [m/s]
  double ud_;
  /// The desired lateral speed [m/s]
  double vd_;
  /// The desired yaw rate [rad/s]
  double rd_;
  /// The desired longitudinal acceleration [m/(s^2)]
  double uddot_;
  /// The desired lateral acceleration [m/(s^2)]
  double vddot_;
  /// The desired angular acceleration about the vertical axis (which
  /// heading/yaw is also measured against) [rad/(s^2)]
  double rddot_;
  /// The desired heading [rad]
  double hd_;

  /// Constructor
  /// \param ud the desired longitudinal speed [m/s]
  /// \param vd the desired lateral speed [m/s]
  /// \param rd the desired longitudinal acceleration [m/(s^2)]
  /// \param uddot the desired longitudinal acceleration [m/(s^2)]
  /// \param vddot the desired lateral acceleration [m/(s^2)]
  /// \param rddot the desired angular acceleration about the vertical axis (which
  //  //  heading/yaw is also measured against) [rad/(s^2)]t
  /// \param hd the desired heading [rad]
  TrajectoryValue(double ud, double vd, double rd, double uddot, double vddot,
                  double rddot, double hd)
      : ud_{ud},
        vd_{vd},
        rd_{rd},
        uddot_{uddot},
        vddot_{vddot},
        rddot_{rddot},
        hd_{hd} {}
  /// Prints the trajectory value to an ostream
  /// \param out the ostream to print to
  /// \param v the trajectory value to print
  /// \return the ostream after printing the trajectory values
  friend std::ostream& operator<<(std::ostream& out, const TrajectoryValue& v) {
    return out << VariadicListToStr(v.ud_, v.vd_, v.rd_, v.uddot_, v.vddot_,
                                    v.rddot_, v.hd_)
               << std::endl;
  }
};

/// TODO
struct Trajectory {
 public:
  /// TODO
  double t_min_;
  /// TODO
  double au_;
  /// TODO
  double ay_;
  /// TODO
  double u0_;
  /// TODO
  double h0_;
  /// TODO
  double u0_goal_;
  /// TODO
  ManuType manu_type_;
  /// TODO
  double t0_offset_;

 private:
  struct ComputedTrajInfo {
    /// TODO
    double delta_spd_;
    /// TODO
    double t_to_use_;
    /// TODO
    double tbrk1_;
    /// TODO
    double tbrk2_;
    /// TODO
    double t_eval_max_;
  };

 public:
  constexpr static double kDoubleInfty =
      std::numeric_limits<double>::infinity();
  constexpr static double kAuMin = 0.5;
  constexpr static double kFourThirdPi = 4.0 * M_PI / 3.0;

  /// TODO
  /// \param au TODO
  /// \param ay TODO
  /// \param u0_goal TODO
  /// \param manu_type TODO
  Trajectory(double au, double ay, double u0_goal, ManuType manu_type)
      : Trajectory(au, ay, 0.0, u0_goal, manu_type, 0.0, 0.0) {}
  /// TODO
  /// \param au TODO
  /// \param ay TODO
  /// \param u0 TODO
  /// \param u0_goal TODO
  /// \param manu_type TODO
  /// \param t0_offset TODO
  Trajectory(double au, double ay, double u0, double u0_goal,
             ManuType manu_type, double t0_offset)
      : Trajectory(au, ay, u0, u0_goal, manu_type, t0_offset, 0.0) {}
  /// TODO
  /// \param au TODO
  /// \param ay TODO
  /// \param u0 TODO
  /// \param u0_goal TODO
  /// \param manu_type TODO
  /// \param t0_offset TODO
  /// \param h0 TODO
  Trajectory(double au, double ay, double u0, double u0_goal,
             ManuType manu_type, double t0_offset, double h0)
      : t_min_{0.0},
        au_{Clamp(au, kAuMin, kDoubleInfty)},
        ay_{ay},
        u0_{u0},
        h0_{h0},
        u0_goal_{u0_goal},
        manu_type_{manu_type},
        t0_offset_{t0_offset} {}
  /// TODO
  Trajectory() : Trajectory(0.0, 0.0, 0.0, ManuType::kNone) {}

  /// TODO
  /// \param t TODO
  /// \return TODO
  TrajectoryValue EvalAt(double t) const;

  /// TODO
  /// \return TODO
  double MaxTrajTime() const;

  /// TODO
  /// \return TODO
  bool HasValidType() const;

 private:
  /// TODO
  /// \return TODO
  ComputedTrajInfo ComputeInfo() const;

  // These can be converted to member variables instead iff there is no
  // place where the parameters are updated after they are computed.

  /// TODO
  /// \return TODO
  double TBrk1() const;

  /// TODO
  /// \return TODO
  double TBrk2() const;

  /// TODO
  /// \return TODO
  double TToUse() const;

  /// TODO
  /// \return TODO
  double TEvalMax() const;

  /// TODO
  /// \return TODO
  double T1Max() const;

  /// TODO
  /// \return TODO
  double T2Max() const;

  /// TODO
  /// \return TODO
  double T3Max() const;

  /// TODO
  /// \return TODO
  bool OneHump() const;

  /// TODO
  /// \return TODO
  bool TimeInRange(double t) const;

  /// TODO
  /// \param t TODO
  /// \return TODO
  int RangeNo(double t) const;
  /// TODO
  /// \param t0 TODO
  /// \param t1 TODO
  /// \return TODO
  bool TimeInSameRange(double t0, double t1) const;
  /// TODO
  /// \param time_interval TODO
  /// \return TODO
  bool TimeIntervalIsOk(Interval time_interval) const;
  /// TODO
  /// \param t TODO
  /// \return TODO
  Interval FiniteDiffTimes(double t) const;

  /// Returns the desired longitudinal speed [m/s] at time \p t
  /// \param t the duration [seconds] into the trajectory
  /// \return the desired longitudinal speed [m/s] at time \p t
  double Ud(double t) const;

  /// Returns the desired yaw rate [rad/s] at time \p t
  /// \param t the duration [seconds] into the trajectory
  /// \return the desired yaw rate [rad/s] at time \p t
  double Rd(double t) const;

  /// Returns the desired lateral speed [m/s] at time \p t
  /// \param t the duration [seconds] into the trajectory
  /// \return the desired lateral speed [m/s] at time \p t
  double Vd(double t) const;

  /// Returns the desired heading [rad] at time \p t
  /// \param t the duration [seconds] into the trajectory
  /// \return the desired heading [rad] at time \p t
  double Hd(double t) const;

  //
  // As long as these are based on differences, they don't vary based on
  // maneuver type.
  //

  /// Returns the desired longitudinal acceleration [m/(s^2)] at time \p t
  /// \param t the duration [seconds] into the trajectory
  /// \return the desired longitudinal acceleration [m/(s^2)] at time \p t
  double UdDot(double t) const;

  /// Returns the desired lateral acceleration [m/(s^2)] at time \p t
  /// \param t the duration [seconds] into the trajectory
  /// \return the desired lateral acceleration [m/(s^2)] at time \p t
  double VdDot(double t) const;

  /// Returns the desired angular acceleration about the vertical axis (which
  /// yaw is also measured against) [rad/(s^2)] at time \p t
  /// \param t the duration [seconds] into the trajectory
  /// \return the desired angular acceleration about the vertical axis (which
  /// yaw is also measured against) [rad/(s^2)] at time \p t
  double RdDot(double t) const;
};
}  // namespace roahm
#endif  // ROAHM_TRAJECTORY_SYM_HPP_
