#ifndef ROAHM_COMPLEX_HEADING_STATE_
#define ROAHM_COMPLEX_HEADING_STATE_
#include <rover_control_msgs/RoverDebugStateStamped.h>

#include <array>
#include <cmath>
#include <complex>
#include <string>

namespace roahm {

std::string StringXYHUVR(double x, double y, double h, double u, double v,
                         double r) {
  return std::to_string(x) + ", " + std::to_string(y) + ", " +
         std::to_string(h) + ", " + std::to_string(u) + ", " +
         std::to_string(v) + ", " + std::to_string(r);
}

inline std::complex<double> ToComplexSafe(double heading) {
  if (not std::isfinite(heading)) {
    // Guard against potential NaN, inf
    heading = 0.0;
  }
  return std::complex<double>(std::cos(heading), std::sin(heading));
}
class ComplexHeadingState {
 public:
  // Stores heading information as complex number in its
  // sine and cosine components to allow for averaging
  // multiple states, and gurantees that output heading
  // will remain within [-pi, pi]
  double x_;
  double y_;
  std::complex<double> h_;
  double u_;
  double v_;
  double r_;
  double w_;
  double r_err_sum_;
  double h_err_sum_;
  double u_err_sum_;

  // TODO decide whether u_err_sum should be in msg
  //
  // IMPORTANT:
  // Since double is implicitly convertible to std::complex<double>,
  // make sure to have constructors for both double and std::complex<double>.
  //
  // If, for example you have
  // ComplexHeadingState(double x, double y, std::complex<double> h)
  // Calling it, e.g. ComplexHeadingState(0, 0, h) will generate the
  // complex number h+0i, rather than cos(h)+sin(h)i
  //

  // Default constructor
  ComplexHeadingState()
      : ComplexHeadingState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) {}

  // Convert from message
  ComplexHeadingState(const rover_control_msgs::RoverDebugStateStamped& msg)
      : ComplexHeadingState(msg.x, msg.y, msg.h, msg.u, msg.v, msg.r, msg.w,
                            msg.r_err_sum, msg.h_err_sum) {}

  // XYHUVRW Constructors
  ComplexHeadingState(double x, double y, double h, double u, double v,
                      double r, double w)
      : ComplexHeadingState(x, y, ToComplexSafe(h), u, v, r, w) {}
  ComplexHeadingState(double x, double y, std::complex<double> h, double u,
                      double v, double r, double w)
      : x_{x},
        y_{y},
        h_{h},
        u_{u},
        v_{v},
        r_{r},
        w_{w},
        r_err_sum_{0},
        h_err_sum_{0},
        u_err_sum_{0} {}

  // XYHUVRW,R_sum,H_sum Constructors
  ComplexHeadingState(double x, double y, double h, double u, double v,
                      double r, double w, double r_err_sum, double h_err_sum)
      : ComplexHeadingState(x, y, ToComplexSafe(h), u, v, r, w, r_err_sum,
                            h_err_sum) {}
  ComplexHeadingState(double x, double y, std::complex<double> h, double u,
                      double v, double r, double w, double r_err_sum,
                      double h_err_sum)
      : x_{x},
        y_{y},
        h_{h},
        u_{u},
        v_{v},
        r_{r},
        w_{w},
        r_err_sum_{r_err_sum},
        h_err_sum_{h_err_sum},
        u_err_sum_{0} {}

  double GetHeading() const {
    const double h_out = std::arg(h_);
    if (not std::isfinite(h_out)) {
      // Guard against potential NaN, inf
      return 0.0;
    }
    return h_out;
  }

  void SetHeading(double heading) { h_ = ToComplexSafe(heading); }

  std::string ToStringXYHUVR() const {
    return StringXYHUVR(x_, y_, GetHeading(), u_, v_, r_);
  }
};
}  // namespace roahm
#endif
