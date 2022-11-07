#include "get_waypoint_from_path.hpp"

#include <algorithm>  // for max, max_element, min, min_element, trans...
#include <cmath>      // for sqrt, atan2, cos, pow, round, sin, M_PI
#include <complex>    // for arg, complex
#include <cstdio>     // for size_t, printf
#include <iostream>   // for operator<<, endl, cout, ostream, basic_os...
#include <limits>     // for numeric_limits
#include <memory>     // for allocator_traits<>::value_type
#include <numeric>    // for partial_sum

#include "point_xy.hpp"     // for PointXY
#include "ros/console.h"    // for LogLocation, ROS_INFO_STREAM, ROS_WARN
#include "simple_util.hpp"  // for Square, AngleDiff, Norm

namespace {

using ::roahm::AngleDiff;
using ::roahm::Norm;
using ::roahm::Square;
using PointXY = ::roahm::PointXY;
using PointXYH = ::roahm::PointXYH;
using PathT = ::roahm::PathT;

struct DistPointToPolylineRet {
  PointXYH p_out_;
  std::size_t p_idx_;
  DistPointToPolylineRet() : p_out_{}, p_idx_{} {}
};

std::vector<double> LineCurvature2D(const std::vector<PointXY>& vertices) {
  // From MATLAB function, D.Kroon University of Twente
  std::array<std::vector<double>, 2> lines;
  for (std::size_t i = 0; i < vertices.size() - 1; ++i) {
    lines.at(0).push_back(i);
    lines.at(1).push_back(i + 1);
  }
  std::vector<long> na(vertices.size(), -1);
  std::vector<long> nb(vertices.size(), -1);
  for (std::size_t i = 0; i < lines.at(0).size(); ++i) {
    na.at(lines.at(0).at(i)) = lines.at(1).at(i);
    nb.at(lines.at(1).at(i)) = lines.at(0).at(i);
  }

  auto naa = na;
  auto nbb = nb;
  std::vector<long> check_na;
  std::vector<long> check_nb;
  for (std::size_t i = 0; i < naa.size(); ++i) {
    if (naa.at(i) == -1) {
      naa.at(i) = i;
      check_na.push_back(i);
    }
  }
  for (std::size_t i = 0; i < nbb.size(); ++i) {
    if (nbb.at(i) == -1) {
      nbb.at(i) = i;
      check_nb.push_back(i);
    }
  }

  for (auto i : check_na) {
    na.at(i) = nbb.at(nbb.at(i));
  }
  for (auto i : check_nb) {
    nb.at(i) = naa.at(naa.at(i));
  }

  std::vector<double> ta;
  std::vector<double> tb;
  for (std::size_t i = 0; i < vertices.size(); ++i) {
    const double vx1 = vertices.at(i).x_;
    const double vy1 = vertices.at(i).y_;
    const double vxA = vertices.at(na.at(i)).x_;
    const double vyA = vertices.at(na.at(i)).y_;
    const double vxB = vertices.at(nb.at(i)).x_;
    const double vyB = vertices.at(nb.at(i)).y_;
    const double dxA = vx1 - vxA;
    const double dyA = vy1 - vyA;
    const double dxB = vx1 - vxB;
    const double dyB = vy1 - vyB;
    const double d_normA = std::sqrt(dxA * dxA + dyA * dyA);
    const double d_normB = std::sqrt(dxB * dxB + dyB * dyB);
    ta.push_back(-d_normA);
    tb.push_back(d_normB);
  }

  for (auto i : check_na) {
    ta.at(i) = -ta.at(i);
  }
  for (auto i : check_nb) {
    tb.at(i) = -tb.at(i);
  }

  std::vector<std::array<double, 3>> x;
  std::vector<std::array<double, 3>> y;
  for (std::size_t i = 0; i < na.size(); ++i) {
    x.push_back({vertices.at(na.at(i)).x_, vertices.at(i).x_,
                 vertices.at(nb.at(i)).x_});
    y.push_back({vertices.at(na.at(i)).y_, vertices.at(i).y_,
                 vertices.at(nb.at(i)).y_});
  }
  std::vector<std::vector<double>> m;
  for (std::size_t i = 0; i < tb.size(); ++i) {
    m.push_back({1.0, -ta.at(i), ta.at(i) * ta.at(i), 1.0, 0.0, 0.0, 1.0,
                 -tb.at(i), tb.at(i) * tb.at(i)});
  }

  std::vector<std::array<std::array<double, 3>, 3>> adjM;
  for (std::size_t i = 0; i < m.size(); ++i) {
    std::array<std::array<double, 3>, 3> temp_array;
    adjM.push_back(temp_array);
    auto& curr_adj = adjM.back();
    auto& curr_m = m.at(i);
    const double det_m = curr_m.at(0) * curr_m.at(4) * curr_m.at(8) -
                         curr_m.at(0) * curr_m.at(7) * curr_m.at(5) -
                         curr_m.at(3) * curr_m.at(1) * curr_m.at(8) +
                         curr_m.at(3) * curr_m.at(7) * curr_m.at(2) +
                         curr_m.at(6) * curr_m.at(1) * curr_m.at(5) -
                         curr_m.at(6) * curr_m.at(4) * curr_m.at(2);

    curr_adj.at(0).at(0) =
        (curr_m.at(4) * curr_m.at(8) - curr_m.at(7) * curr_m.at(5)) / det_m;
    curr_adj.at(0).at(1) =
        (-(curr_m.at(3) * curr_m.at(8) - curr_m.at(6) * curr_m.at(5))) / det_m;
    curr_adj.at(0).at(2) =
        (curr_m.at(3) * curr_m.at(7) - curr_m.at(6) * curr_m.at(4)) / det_m;

    curr_adj.at(1).at(0) =
        (-(curr_m.at(1) * curr_m.at(8) - curr_m.at(7) * curr_m.at(2))) / det_m;
    curr_adj.at(1).at(1) =
        (curr_m.at(0) * curr_m.at(8) - curr_m.at(6) * curr_m.at(2)) / det_m;
    curr_adj.at(1).at(2) =
        (-(curr_m.at(0) * curr_m.at(7) - curr_m.at(6) * curr_m.at(1))) / det_m;

    curr_adj.at(2).at(0) =
        (curr_m.at(1) * curr_m.at(5) - curr_m.at(4) * curr_m.at(2)) / det_m;
    curr_adj.at(2).at(1) =
        (-(curr_m.at(0) * curr_m.at(5) - curr_m.at(3) * curr_m.at(2))) / det_m;
    curr_adj.at(2).at(2) =
        (curr_m.at(0) * curr_m.at(4) - curr_m.at(3) * curr_m.at(1)) / det_m;
  }

  std::vector<std::array<double, 3>> a;
  std::vector<std::array<double, 3>> b;
  std::vector<double> k;
  for (std::size_t i = 0; i < adjM.size(); ++i) {
    std::array<double, 3> temp1;
    std::array<double, 3> temp2;
    a.push_back(temp1);
    b.push_back(temp2);
    auto& curr_a = a.back();
    auto& curr_b = b.back();
    const auto& curr_invm = adjM.at(i);
    const auto& curr_x = x.at(i);
    const auto& curr_y = y.at(i);

    curr_a.at(0) = curr_invm.at(0).at(0) * curr_x.at(0) +
                   curr_invm.at(1).at(0) * curr_x.at(1) +
                   curr_invm.at(2).at(0) * curr_x.at(2);
    curr_a.at(1) = curr_invm.at(0).at(1) * curr_x.at(0) +
                   curr_invm.at(1).at(1) * curr_x.at(1) +
                   curr_invm.at(2).at(1) * curr_x.at(2);
    curr_a.at(2) = curr_invm.at(0).at(2) * curr_x.at(0) +
                   curr_invm.at(1).at(2) * curr_x.at(1) +
                   curr_invm.at(2).at(2) * curr_x.at(2);

    curr_b.at(0) = curr_invm.at(0).at(0) * curr_y.at(0) +
                   curr_invm.at(1).at(0) * curr_y.at(1) +
                   curr_invm.at(2).at(0) * curr_y.at(2);
    curr_b.at(1) = curr_invm.at(0).at(1) * curr_y.at(0) +
                   curr_invm.at(1).at(1) * curr_y.at(1) +
                   curr_invm.at(2).at(1) * curr_y.at(2);
    curr_b.at(2) = curr_invm.at(0).at(2) * curr_y.at(0) +
                   curr_invm.at(1).at(2) * curr_y.at(1) +
                   curr_invm.at(2).at(2) * curr_y.at(2);

    k.push_back(2.0 *
                (curr_a.at(1) * curr_b.at(2) - curr_a.at(2) * curr_b.at(1)) /
                std::pow((Square(curr_a.at(1)) + Square(curr_b.at(1))), 1.5));
  }
  return k;
}

std::vector<double> DistPointToPoints(const double state_x,
                                      const double state_y,
                                      const PathT& hlp_path) {
  std::vector<double> distances;
  for (const auto& pt : hlp_path) {
    distances.push_back(Norm(pt.x_ - state_x, pt.y_ - state_y));
  }
  return distances;
}

DistPointToPolylineRet DistPointToPolyline(const double state_x,
                                           const double state_y,
                                           const PathT& hlp_path) {
  // const auto p2 = DistPointToPoints(state_x, state_y, hlp_path); //JL: not
  // used
  std::vector<double> t;
  auto pa = hlp_path;
  pa.pop_back();
  bool t_between_01 = false;

  for (std::size_t i = 1; i < hlp_path.size(); ++i) {
    const auto p0 = hlp_path.at(i - 1);
    const auto p1 = hlp_path.at(i);
    const double dp_x = p1.x_ - p0.x_;
    const double dp_y = p1.y_ - p0.y_;
    const double p2p_x = state_x - p0.x_;
    const double p2p_y = state_y - p0.y_;
    const double t_x = p2p_x * dp_x;
    const double t_y = p2p_y * dp_y;
    const double p2_val = Square(p1.x_ - p0.x_) + Square(p1.y_ - p0.y_);
    const double t_val = (t_x + t_y) / p2_val;
    t.push_back(t_val);
    if (t_val > 0 && t_val < 1) {
      t_between_01 = true;
      pa.at(i - 1).x_ = p0.x_ + t_val * dp_x;
      pa.at(i - 1).y_ = p0.y_ + t_val * dp_y;
    }
  }

  if (t_between_01) {
    pa.push_back(hlp_path.back());
  }
  auto d_out = DistPointToPoints(state_x, state_y, pa);
  const auto min_it = std::min_element(d_out.begin(), d_out.end());
  const auto p_idx = std::distance(d_out.begin(), min_it);
  const auto p_out = pa.at(p_idx);
  DistPointToPolylineRet ret_val;
  ret_val.p_out_ = p_out;
  ret_val.p_idx_ = p_idx;
  return ret_val;
}

PointXYH MeanPt(const PointXYH& p0, const PointXYH& p1, double pct) {
  const double h_unsafe = (pct * AngleDiff(p1.h_, p0.h_)) + p0.h_;
  // TODO just call function here
  const double h_safe =
      std::arg(std::complex<double>(std::cos(h_unsafe), std::sin(h_unsafe)));
  const double x = pct * (p1.x_ - p0.x_) + p0.x_;
  const double y = pct * (p1.y_ - p0.y_) + p0.y_;
  return {x, y, h_safe};
}

PointXYH MatchTrajectories(const double lookup_val,
                           const std::vector<double>& d_best,
                           const PathT& hlp_path) {
  long idx_above = -1;
  long idx_below = -1;
  double dist_above = std::numeric_limits<double>::max();
  double dist_below = std::numeric_limits<double>::lowest();
  for (std::size_t i = 0; i < d_best.size(); ++i) {
    const double dist = d_best.at(i) - lookup_val;
    if (dist > 0) {
      if (dist < dist_above) {
        dist_above = dist;
        idx_above = i;
      }
    }
    if (dist <= 0) {
      if (dist >= dist_below) {
        dist_below = dist;
        idx_below = i;
      }
    }
  }
  if (idx_below == -1 && idx_above == -1) {
    return {};  // JL: should return an error or something
  } else if (idx_below != -1 && idx_above != -1) {
    const double rng = d_best.at(idx_above) - d_best.at(idx_below);
    if (std::abs(rng) <= 1.0e-4) {
      return hlp_path.at(idx_below);
    }
    const double pct = (lookup_val - d_best.at(idx_below)) / rng;
    return MeanPt(hlp_path.at(idx_below), hlp_path.at(idx_above), pct);
  } else if (idx_below != -1) {
    return hlp_path.at(idx_below);
  } else /* if (idx_above != -1) */ {
    return hlp_path.at(idx_above);
  }
}

}  // namespace

namespace roahm {

DistPoint DistPointOnPolylineSimon(const double state_x, const double state_y,
                                   const PathT& hlp_path) {
  // Compute point-to-point distances
  std::vector<double> dp_dists;
  for (std::size_t i = 1; i < hlp_path.size(); ++i) {
    const auto& p0 = hlp_path.at(i - 1);
    const auto& p1 = hlp_path.at(i);
    const double dx = p1.x_ - p0.x_;
    const double dy = p1.y_ - p0.y_;
    const double norm_v = Norm(dx, dy);
    dp_dists.push_back(norm_v);
  }

  // Compute cumulative distances from initial point
  std::vector<double> p_dists;
  p_dists.push_back(0.0);
  std::partial_sum(dp_dists.begin(), dp_dists.end(),
                   std::back_inserter(p_dists));

  const double last_dist = p_dists.back();
  std::vector<double> p_percents;
  std::transform(
      p_dists.begin(), p_dists.end(), std::back_inserter(p_percents),
      [last_dist](const double dist_i) { return dist_i / last_dist; });
  const auto dist_pt_polyline_ret =
      DistPointToPolyline(state_x, state_y, hlp_path);
  const auto p_idx = dist_pt_polyline_ret.p_idx_;
  const double d0 = p_dists.at(p_idx);
  const double d1 =
      Norm(dist_pt_polyline_ret.p_out_.x_ - hlp_path.at(p_idx).x_,
           dist_pt_polyline_ret.p_out_.y_ - hlp_path.at(p_idx).y_);
  const double d = d0 + d1;

  DistPoint ret;
  ret.d_ = d;
  ret.p_idx_ = p_idx;
  ret.d_along_ = p_dists;

  return ret;
}

std::array<PointXYH, 3> GetWaypointFromPath(
    const RoverState& state, const PathT& hlp_path,
    const WaypointInterpParams& wp_params) {
  // TODO NOTE: this is currently unused, should be removed from launch?
  // const auto max_lookahead = wp_params.max_lookahead_;
  const auto peak_velocity = wp_params.peak_velocity_;
  const auto points_per_meter = wp_params.points_per_meter_;
  const auto low_curvature_lookahead_mult =
      wp_params.low_curvature_lookahead_mult_;
  const auto high_curvature_lookahead_mult =
      wp_params.high_curvature_lookahead_mult_;
  const auto low_curvature_heading_plus = wp_params.low_curvature_heading_plus_;
  const auto high_curvature_heading_plus =
      wp_params.high_curvature_heading_plus_;
  const auto high_curvature_cutoff_min = wp_params.high_curvature_cutoff_min_;
  const auto reduce_curve_lookahead = wp_params.reduce_curve_lookahead_;
  const auto use_JL_idea = wp_params.use_JL_idea_;

  const auto dist_pts_simon =
      DistPointOnPolylineSimon(state.x_, state.y_, hlp_path);

  const auto d_best = dist_pts_simon.d_along_;
  const auto p_idx = dist_pts_simon.p_idx_;
  const double d = dist_pts_simon.d_;

  double lookahead_distance = 0.0;
  const long max_curve_pts =
      static_cast<long>(std::round(reduce_curve_lookahead * points_per_meter));
  bool is_high_curvature = false;

  if (use_JL_idea == 1.0 && (p_idx + 2 * max_curve_pts + 1) < hlp_path.size()) {
    // JL's idea: Look at 2*max_curve_pts to decide lookahead distance. the
    // 2*max_curve_pts is
    //            divided into 2 parts equally. If curvature of both parts are
    //            small, then use larger lookahead distance. If curvature of
    //            part 1 is small and part 2 is large, then use smaller
    //            lookahead distance to slow down. If curvature of part 1 is
    //            large, directly go with smaller lookahead distance.
    ROS_INFO_STREAM("Use JL's idea with doubled lookahead points");

    // fetch two groups of points
    std::vector<PointXY> pxy1;
    std::vector<PointXY> pxy2;
    for (std::size_t i = p_idx; i <= p_idx + max_curve_pts; ++i) {
      pxy1.emplace_back(hlp_path.at(i).x_, hlp_path.at(i).y_);
      pxy2.emplace_back(hlp_path.at(i + max_curve_pts).x_,
                        hlp_path.at(i + max_curve_pts).y_);
    }

    // compute curvature
    const auto k_arr1 = LineCurvature2D(pxy1);
    const auto k_arr2 = LineCurvature2D(pxy2);
    const double k1 = *std::max_element(k_arr1.begin(), k_arr1.end(),
                                        [](const double& a, const double& b) {
                                          return std::abs(a) > std::abs(b);
                                        });
    const double k2 = *std::max_element(k_arr2.begin(), k_arr2.end(),
                                        [](const double& a, const double& b) {
                                          return std::abs(a) > std::abs(b);
                                        });
    if ((k1 > high_curvature_cutoff_min) || (k2 > high_curvature_cutoff_min)) {
      lookahead_distance = peak_velocity * high_curvature_lookahead_mult +
                           high_curvature_heading_plus;
      is_high_curvature = true;
    } else {
      lookahead_distance = peak_velocity * low_curvature_lookahead_mult +
                           low_curvature_heading_plus;
    }
  } else {
    if ((p_idx + max_curve_pts + 1) < hlp_path.size()) {
      std::vector<PointXY> pxy;

      for (std::size_t i = p_idx; i <= p_idx + max_curve_pts; ++i) {
        pxy.emplace_back(hlp_path.at(i).x_, hlp_path.at(i).y_);
      }

      const auto k_arr = LineCurvature2D(pxy);

      // Max absolute value
      const double k = *std::max_element(k_arr.begin(), k_arr.end(),
                                         [](const double& a, const double& b) {
                                           return std::abs(a) > std::abs(b);
                                         });

      if (k > high_curvature_cutoff_min) {
        lookahead_distance = peak_velocity * high_curvature_lookahead_mult;
        is_high_curvature = true;
      } else {
        lookahead_distance = peak_velocity * low_curvature_lookahead_mult;
      }
    } else {
      lookahead_distance = 1.0;
    }
  }

  const double d_best_end = d_best.back();
  const double d_lkhd_lin = std::min(d_best_end, d + lookahead_distance);
  const double d_lkhd1_lin = std::min(d_best_end, d + lookahead_distance / 3.0);
  const double d_lkhd2_lin =
      std::min(d_best_end, d + lookahead_distance / 3.0 * 2.0);
  const auto wp = MatchTrajectories(d_lkhd_lin, d_best, hlp_path);
  const auto wp1 = MatchTrajectories(d_lkhd1_lin, d_best, hlp_path);
  const auto wp2 = MatchTrajectories(d_lkhd2_lin, d_best, hlp_path);
  const double heading_additional_lookahead =
      is_high_curvature ? high_curvature_heading_plus
                        : low_curvature_heading_plus;  // JL: bug. it was _mult
  const double heading_lookahead =
      lookahead_distance + heading_additional_lookahead;
  const double d_lkhd_head = std::min(d_best_end, d + heading_lookahead);
  const double d_lkhd1_head = std::min(d_best_end, d + heading_lookahead / 3.0);
  const double d_lkhd2_head =
      std::min(d_best_end, d + heading_lookahead / 3.0 * 2.0);
  const auto wph = MatchTrajectories(d_lkhd_head, d_best, hlp_path);
  const auto wp1h = MatchTrajectories(d_lkhd1_head, d_best, hlp_path);
  const auto wp2h = MatchTrajectories(d_lkhd2_head, d_best, hlp_path);
  std::array<PointXYH, 3> waypoints;
  waypoints[0] = PointXYH{wp.x_, wp.y_, wph.h_};
  waypoints[1] = PointXYH{wp1.x_, wp1.y_, wp1h.h_};
  waypoints[2] = PointXYH{wp2.x_, wp2.y_, wp2h.h_};
  return waypoints;
}

}  // namespace roahm