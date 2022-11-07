#ifndef ROAHM_SIMPLE_UTIL_H_
#define ROAHM_SIMPLE_UTIL_H_
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>            // for Imu
#include <tf2/LinearMath/Matrix3x3.h>   // for Matrix3x3
#include <tf2/LinearMath/Quaternion.h>  // for Quaternion

#include <algorithm>
#include <cmath>
#include <complex>
#include <vector>

#include "point_xy.h"
namespace roahm {

template <typename T, class A>
auto MinAbsDist(const std::vector<T, A>& vec, const T& val) {
  // Returns iterator to minimum absolute value
  return std::min_element(vec.begin(), vec.end(),
                          [val](const double& a, const double& b) {
                            return std::abs(a - val) < std::abs(b - val);
                          });
}

template <typename T, class A>
std::size_t MinAbsDistIdx(const std::vector<T, A>& vec, const T& val) {
  const auto min_dist_iter = MinAbsDist(vec, val);
  return std::distance(vec.begin(), min_dist_iter);
}

template <typename ContainerType, typename Op>
auto MinElement(const ContainerType& container, const Op& op) {
  return std::min_element(container.begin(), container.end(), op);
}

template <typename ContainerType, typename Op>
auto MinElementIdx(const ContainerType& container, const Op& op) {
  return std::distance(container.begin(), MinElement(container, op));
}

double ThreeHalvesPower(double expr) {
  const double sqrt_expr = std::sqrt(expr);
  return sqrt_expr * sqrt_expr * sqrt_expr;
}

double ToAngle(double h) {
  if (not std::isfinite(h)) {
    return 0;
  }
  return std::arg(std::complex<double>{std::cos(h), std::sin(h)});
}
double GetQuatYaw(const tf2::Quaternion& q) {
  double roll;
  double pitch;
  double yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}
double GetOrientationYaw(const geometry_msgs::Quaternion& q_in) {
  tf2::Quaternion q(q_in.x, q_in.y, q_in.z, q_in.w);
  return GetQuatYaw(q);
}
double GetImuYaw(const sensor_msgs::Imu& msg) {
  return GetOrientationYaw(msg.orientation);
}

// Modified from https://stackoverflow.com/a/9154394
// https://stackoverflow.com/questions/257288/templated-check-for-the-
// existence-of-a-class-member-function
// (Xeo, Brad Larson)
namespace detail {
template <class>
struct sfinae_true : std::true_type {};

template <class T, class A0>
static auto test_stream(int)
    -> sfinae_true<decltype(std::declval<T>().operator<<(std::declval<A0>()))>;
template <class, class A0>
static auto test_stream(long) -> std::false_type;
}  // namespace detail

template <class T, class Arg>
struct has_stream : decltype(detail::test_stream<T, Arg>(0)) {};

template <typename K, typename V>
V& GetWithDefault(std::map<K, V>& kv, const K& k, const V& default_val) {
  return (*(kv.try_emplace(k, default_val).first)).second;
}
template <typename K, typename V, typename KDefault>
V& GetWithDefaultWarn(std::map<K, V>& kv, const KDefault& k,
                      const V& default_val) {
  static_assert(std::is_convertible_v<KDefault, K>);
  const auto attempt = kv.try_emplace(K(k), default_val);
  if (attempt.second) {
    ROS_WARN_STREAM("Could not get parameter '"
                    << k << "', setting value to default: " << default_val);
  }
  return (*(attempt.first)).second;
}

template <typename T>
T GetRosParam(const std::string name, T default_val) {
  T ret_val;
  if (not ros::param::get(name, ret_val)) {
    if constexpr (has_stream<std::ostream, T>::value) {
      ROS_ERROR_STREAM("Parameter " << name << " not found. Setting to default "
                                    << default_val);
    } else {
      ROS_ERROR_STREAM("Parameter " << name
                                    << " not found. Setting to default.");
    }
    ret_val = default_val;
  }
  return ret_val;
}

std::vector<PointXY> Clockwise(const std::vector<PointXY>& unsorted) {
  double c_x = 0.0;
  double c_y = 0.0;
  for (const auto& pt : unsorted) {
    c_x += pt.x_;
    c_y += pt.y_;
  }
  c_x /= unsorted.size();
  c_y /= unsorted.size();
  auto ret = unsorted;
  std::sort(ret.begin(), ret.end(),
            [c_x, c_y](const PointXY& a, const PointXY& b) {
              const double delta_b_x = b.x_ - c_x;
              const double delta_a_x = a.x_ - c_x;
              const double delta_b_y = b.y_ - c_y;
              const double delta_a_y = a.y_ - c_y;
              if (delta_a_x >= 0 and delta_b_x < 0) {
                return true;
              }
              if (delta_a_x < 0 and delta_b_x >= 0) {
                return false;
              }
              if (delta_a_x == 0 && delta_b_x == 0) {
                if (delta_a_y >= 0 or delta_b_y >= 0) {
                  return a.y_ > b.y_;
                }
                return b.y_ > a.y_;
              }
              double det = delta_a_x * delta_b_y - delta_b_x * delta_a_y;
              if (det < 0) {
                return true;
              }
              if (det > 0) {
                return false;
              }

              double d1 = delta_a_x * delta_a_x + delta_a_y * delta_a_y;
              double d2 = delta_b_x * delta_b_x + delta_b_y * delta_b_y;
              return d1 > d2;
            });
  return ret;
}
std::vector<PointXY> CounterClockwise(const std::vector<PointXY>& unsorted) {
  auto ret = Clockwise(unsorted);
  std::reverse(ret.begin(), ret.end());
  return ret;
}
long Clamp(long v, long min_v, long max_v) {
  return std::min(std::max(v, min_v), max_v);
}
double Clamp(double v, double min_v, double max_v) {
  // NaN returns false for all comparisons, so this
  // returns min_v if v is NaN
  if (v >= min_v and v <= max_v) {
    return v;
  } else if (v > max_v) {
    return max_v;
  } else if (v < min_v) {
    return min_v;
  }
  return min_v;
}
inline double ClampWithWarn(double val, double min_v, double max_v,
                            std::string name) {
  if (val > max_v || val < min_v) {
    ROS_WARN_STREAM(name << " (" << val << ") not in [" << min_v << ", "
                         << max_v << "]");
  }
  return Clamp(val, min_v, max_v);
}
inline double Square(double val) { return val * val; }
inline double Pow3(double val) { return val * val * val; }
inline double Pow4(double val) { return Square(Square(val)); }
inline double Norm(double dx, double dy) {
  return std::sqrt(dx * dx + dy * dy);
}
inline double Min(double a, double b) { return b < a ? b : a; }
inline double Max(double a, double b) { return b < a ? a : b; }

// From
// https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
template <typename T>
int Sign(T val) {
  // Returns:
  //  Non-special and non-zero: the sign in {-1, 1}
  //  +/- 0:       0
  //  +/- Inf: +/- 1
  //  +/- NaN:     0
  //         (NaN's are typically defined to evaluate false
  //          on all inequality comparisons)
  return (T(0) < val) - (val < T(0));
}
template <typename T>
int SignPosNegOnly(T val) {
  // Returns:
  //  Non-special and non-zero: the sign in {-1, 1}
  //  +/- 0:       1
  //  +/- Inf: +/- 1
  //  +/- NaN:     1
  //         (NaN's are typically defined to evaluate false
  //          on all inequality comparisons)
  return !(T(0) > val) - (val < T(0));
}

double LimitRate(double val, double prev_val, double dt,
                 double max_rate_of_change) {
  double deriv = (val - prev_val) / dt;
  if (deriv > max_rate_of_change) {
    val = prev_val + max_rate_of_change * dt;
  } else if (deriv < -max_rate_of_change) {
    val = prev_val - max_rate_of_change * dt;
  }
  return val;
}

inline double ClampAbs(double val, double abs_bound) {
  // Clamps val to [-abs_bound, abs_bound]
  // abs_bound must be >= 0
  if (val > abs_bound) {
    return abs_bound;
  }
  if (val < -abs_bound) {
    return -abs_bound;
  }
  return val;
  // int lt = val < -abs_bound;
  // int gt = val > abs_bound;
  // return (val * (1 - (lt + gt))) + (abs_bound * (-lt + gt));
}
inline bool InOpenClosedInterval(double val, double min_v, double max_v) {
  // Returns true iff val in (min_v, max_v]
  // NaN min or max always return false
  return (min_v < val) && (val <= max_v);
}
inline bool InClosedOpenInterval(double val, double min_v, double max_v) {
  // Returns true iff val in [min_v, max_v)
  // NaN min or max always return false
  return (min_v <= val) && (val < max_v);
}
inline bool InClosedInterval(double val, double min_v, double max_v) {
  // Returns true iff val in [min_v, max_v]
  // NaN min or max always return false
  return (min_v <= val) && (val <= max_v);
}
inline bool InOpenInterval(double val, double min_v, double max_v) {
  // Returns true iff val in (min_v, max_v)
  // NaN min or max always return false
  return (min_v < val) && (val < max_v);
}

inline bool AllGreaterThan(std::initializer_list<double> vals, double max_v) {
  // Returns true iff all vals > max_v
  // NaN max_v will always return true
  for (const auto val : vals) {
    if (val <= max_v) {
      return false;
    }
  }
  return true;
}

inline bool AllLessThan(std::initializer_list<double> vals, double min_v) {
  // Returns true iff vals < min_v
  // NaN max_v will always return true
  for (const auto val : vals) {
    if (val >= min_v) {
      return false;
    }
  }
  return true;
}
double AngleDiff(double h1, double h2) {
  // returns angular difference ~ h1 - h2
  std::complex<double> h1_cmplx{std::cos(h1), std::sin(h1)};
  std::complex<double> h2_cmplx{std::cos(h2), std::sin(h2)};
  return std::arg(h1_cmplx * std::conj(h2_cmplx));
}

// https://stackoverflow.com/questions/16337610/how-to-know-if-a-type-is-a-specialization-of-stdvector
template <typename Test, template <typename...> class Ref>
struct is_specialization : std::false_type {};

template <template <typename...> class Ref, typename... Args>
struct is_specialization<Ref<Args...>, Ref> : std::true_type {};

template <typename T>
bool IsPopulatedMultidim(const T& container) {
  // Assumes that each subset of the container has identical elements.
  // i.e. the multidimensional vector is equivalent to a multidimensional
  // array. Therefore, we only recursively check the first subdimension
  // for emptiness.
  if constexpr (is_specialization<T, std::vector>::value) {
    return container.size() > 0 && IsPopulatedMultidim(container.at(0));
  } else {
    return true;
  }
}

struct Interval {
  double min_;
  double max_;
  bool Contains(double val) const { return InClosedInterval(val, min_, max_); }
  double DistanceTo(double val) const {
    // If val < min: val - min_ < 0
    // If val > max: max_ - val < 0
    const double max_dist = val - max_;
    const double min_dist = min_ - val;
    return Max(Max(max_dist, min_dist), 0.0);
  }
};

//
// Intersection Tests
//

Interval MinAndMax(double a, double b) { return {Min(a, b), Max(a, b)}; }

bool OnSegment(const PointXY& p, const PointXY& q, const PointXY& r) {
  // true if Q is on line segment PR when Q is assumed to be colinear with PR
  const auto [min_x, max_x] = MinAndMax(p.x_, r.x_);
  const auto [min_y, max_y] = MinAndMax(p.y_, r.y_);
  const bool x_ok = InClosedInterval(q.x_, min_x, max_x);
  const bool y_ok = InClosedInterval(q.y_, min_y, max_y);
  return x_ok and y_ok;
}
int GetOrientation(const PointXY& a, const PointXY& b, const PointXY& c) {
  const double v0 = ((c.y_ - a.y_) * (b.x_ - a.x_));
  const double v1 = ((b.y_ - a.y_) * (c.x_ - a.x_));
  if (std::abs(v0 - v1) < 1.0e-10) {
    return 0;
  } else if (v0 > v1) {
    return 1;  // CCW
  } else {
    return -1;  // CW
  }
}

bool LineSegmentsIntersect(const PointXY& p1, const PointXY& q1,
                           const PointXY& p2, const PointXY& q2) {
  // Modified from geeksforgeeks.com
  // Check intersection between line segments p1q1, p2q2
  const int o1 = GetOrientation(p1, q1, p2);
  const int o2 = GetOrientation(p1, q1, q2);
  const int o3 = GetOrientation(p2, q2, p1);
  const int o4 = GetOrientation(p2, q2, q1);

  // General case, no set of 3 colinear points
  if (o1 != 0 and o2 != 0 and o3 != 0 and o4 != 0) {
    return (o1 != o2) and (o3 != o4);
  }

  // e.g. p1, q1 and p2 are collinear and p2 lies on segment p1q1
  if (o1 == 0) {
    return OnSegment(p1, p2, q1);
  }
  if (o2 == 0) {
    return OnSegment(p1, q2, q1);
  }
  if (o3 == 0) {
    return OnSegment(p2, p1, q2);
  }
  if (o4 == 0) {
    return OnSegment(p2, q1, q2);
  }

  return false;
}

bool CouldIntersect(const std::vector<PointXY>& hull,
                    const std::vector<PointXY>& obstacle) {
  // Hull and obstacle must both be cyclic; i.e. hull.back() == hull.front()
  for (int obs_pt1_idx = 1; obs_pt1_idx < obstacle.size(); ++obs_pt1_idx) {
    const auto& obs_pt_a = obstacle.at(obs_pt1_idx - 1);
    const auto& obs_pt_b = obstacle.at(obs_pt1_idx);
    for (int hull_pt1_idx = 1; hull_pt1_idx < hull.size(); ++hull_pt1_idx) {
      const auto& hull_pt_a = hull.at(hull_pt1_idx - 1);
      const auto& hull_pt_b = hull.at(hull_pt1_idx);
      if (LineSegmentsIntersect(obs_pt_a, obs_pt_b, hull_pt_a, hull_pt_b)) {
        return true;
      }
    }
  }
  return false;
};

}  // namespace roahm
#endif
