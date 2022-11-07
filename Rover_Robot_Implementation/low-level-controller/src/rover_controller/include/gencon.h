#ifndef ROAHM_GENCON_H_
#define ROAHM_GENCON_H_
#include <geometry_msgs/PolygonStamped.h>
#include <jsk_recognition_msgs/PolygonArray.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <numeric>
#include <random>

#include "common.h"
#include "complex_heading_state.h"
#include "frs_loader.h"
#include "point_xy.h"
#include "point_xyh.h"
#include "simple_util.h"

namespace roahm {

double PointToLineSegDist(double x0, double x1, double x2, double y0, double y1,
                          double y2) {
  // https://math.stackexchange.com/questions/330269/the-distance-from-a-point-to-a-line-segment
  const double t_star =
      (x0 - x1) * (x2 - x1) +
      (y0 - y1) * (y2 - y1) / (Square(x2 - x1) + Square(y2 - y1));
  const double x_star = x1 + t_star * (x2 - x1);
  const double y_star = y1 + t_star * (y2 - y1);
  const double dist = std::sqrt(Square(x0 - x_star) + Square(y0 - y_star));
  return dist;
}

struct CostFcnInfo {
  double c_x_;
  double c_y_;
  double c_h_;
  double c_k_;
  double g_x_;
  double g_y_;
  double g_h_;
  double g_k_;
  double x_des_;
  double y_des_;
  double h_des_;
  struct CostAndDerivs {
    double cost_k_;
    double jac_k_;
    double hess_k_;
    double optimal_k_in_rng_;
    double optimal_cost_in_rng_;
  };
  CostAndDerivs ComputeCosts(const double k, const double k_min,
                             const double k_max, bool eval_opt) const {
    CostAndDerivs ret_val{};
#define PRINT_VAR_MAT(x) std::cout << #x " = " << x << ";\n";
    const double c_x = c_x_;
    const double c_y = c_y_;
    const double c_h = c_h_;
    const double c_k = c_k_;
    const double g_x = g_x_;
    const double g_y = g_y_;
    const double g_h = g_h_;
    const double g_k = g_k_;
    const double gx2 = Square(g_x);
    const double gy2 = Square(g_y);
    const double gh2 = Square(g_h);
    const double gk2 = Square(g_k);
    const double x_des = x_des_;
    const double y_des = y_des_;
    const double h_des = h_des_;
    const double dx = x_des - c_x + (g_x * (c_k - k)) / g_k;
    const double dy = y_des - c_y + (g_y * (c_k - k)) / g_k;
    const double dh = h_des - c_h + (g_h * (c_k - k)) / g_k;
    const double dx2 = Square(dx);
    const double dy2 = Square(dy);
    const double dh2 = Square(dh);
    const double lambda = (k - c_k) / g_k;

    eval_opt = false;
    const bool print_vars = eval_opt;

    constexpr bool kUseDeltaXyhNorm = true;
    if constexpr (kUseDeltaXyhNorm) {
      ret_val.cost_k_ = std::sqrt(dx2 + dy2 + dh2);
      // ret_val.jac_k_  = (dh*g_h + dx*g_x + dy*g_y)/(g_k*std::sqrt(dh2 + dx2 +
      // dy2));
      ret_val.jac_k_ =
          -((2.0 * g_h * (h_des - c_h + (g_h * (c_k - k)) / g_k)) / g_k +
            (2.0 * g_x * (x_des - c_x + (g_x * (c_k - k)) / g_k)) / g_k +
            (2.0 * g_y * (y_des - c_y + (g_y * (c_k - k)) / g_k)) / g_k) /
          (2.0 * std::sqrt(Square(h_des - c_h + (g_h * (c_k - k)) / g_k) +
                           Square(x_des - c_x + (g_x * (c_k - k)) / g_k) +
                           Square(y_des - c_y + (g_y * (c_k - k)) / g_k)));

      ret_val.hess_k_ =
          ((2.0 * gh2) / gk2 + (2.0 * gx2) / gk2 + (2.0 * gy2) / gk2) /
              (2.0 * std::sqrt(Square(h_des - c_h + (g_h * (c_k - k)) / g_k) +
                               Square(x_des - c_x + (g_x * (c_k - k)) / g_k) +
                               Square(y_des - c_y + (g_y * (c_k - k)) / g_k))) -
          Square((2.0 * g_h * (h_des - c_h + (g_h * (c_k - k)) / g_k)) / g_k +
                 (2.0 * g_x * (x_des - c_x + (g_x * (c_k - k)) / g_k)) / g_k +
                 (2.0 * g_y * (y_des - c_y + (g_y * (c_k - k)) / g_k)) / g_k) /
              (4 *
               ThreeHalvesPower(Square(h_des - c_h + (g_h * (c_k - k)) / g_k) +
                                Square(x_des - c_x + (g_x * (c_k - k)) / g_k) +
                                Square(y_des - c_y + (g_y * (c_k - k)) / g_k)));
      // ret_val.hess_k_ = (gh2 + gx2 + gy2)/(gk2*std::sqrt(dh2 + dx2 + dy2)) -
      // Square(dh*g_h + dx*g_x + dy*g_y)/(gk2*ThreeHalvesPower(dh2 + dx2 +
      // dy2));
    } else {
      ret_val.cost_k_ =
          (3.0 * std::sqrt(Square(h_des - c_h + (g_h * (c_k - k)) / g_k))) /
              20.0 +
          std::sqrt(Square(x_des - c_x + (g_x * (c_k - k)) / g_k) +
                    Square(y_des - c_y + (g_y * (c_k - k)) / g_k)) /
              2.0;
      ret_val.jac_k_ =
          -((2.0 * g_x * (x_des - c_x + (g_x * (c_k - k)) / g_k)) / g_k +
            (2.0 * g_y * (y_des - c_y + (g_y * (c_k - k)) / g_k)) / g_k) /
              (4.0 * std::sqrt(Square(x_des - c_x + (g_x * (c_k - k)) / g_k) +
                               Square(y_des - c_y + (g_y * (c_k - k)) / g_k))) -
          (3.0 * g_h * (h_des - c_h + (g_h * (c_k - k)) / g_k)) /
              (20.0 * g_k *
               std::sqrt(Square(h_des - c_h + (g_h * (c_k - k)) / g_k)));
      ret_val.hess_k_ =
          ((2.0 * gx2) / gk2 + (2.0 * gy2) / gk2) /
              (4.0 * std::sqrt(Square(x_des - c_x + (g_x * (c_k - k)) / g_k) +
                               Square(y_des - c_y + (g_y * (c_k - k)) / g_k))) -
          Square((2.0 * g_x * (x_des - c_x + (g_x * (c_k - k)) / g_k)) / g_k +
                 (2.0 * g_y * (y_des - c_y + (g_y * (c_k - k)) / g_k)) / g_k) /
              (8.0 * ThreeHalvesPower(
                         Square(x_des - c_x + (g_x * (c_k - k)) / g_k) +
                         Square(y_des - c_y + (g_y * (c_k - k)) / g_k))) +
          (3.0 * gh2) /
              (20.0 * gk2 *
               std::sqrt(Square(h_des - c_h + (g_h * (c_k - k)) / g_k))) -
          (3.0 * gh2 * Square(h_des - c_h + (g_h * (c_k - k)) / g_k)) /
              (20.0 * gk2 *
               ThreeHalvesPower(Square(h_des - c_h + (g_h * (c_k - k)) / g_k)));
    }

    // We can have discontinuous derivatives, just set jacobian to zero
    // at the discontinouous points.
    const auto handle_discontinuity = [](double d) {
      if (not std::isfinite(d)) {
        return 0.0;
      }
      return d;
    };
    ret_val.jac_k_ = handle_discontinuity(ret_val.jac_k_);
    ret_val.hess_k_ = handle_discontinuity(ret_val.hess_k_);

    if (eval_opt) {
      double jac_0 = 0.0;
      if constexpr (kUseDeltaXyhNorm) {
        jac_0 = (c_k * gh2 + c_k * gx2 + c_k * gy2 - c_h * g_h * g_k -
                 c_x * g_k * g_x - c_y * g_k * g_y + g_h * g_k * h_des +
                 g_k * g_x * x_des + g_k * g_y * y_des) /
                (gh2 + gx2 + gy2);
      } else {
        const double common_first =
            c_k - (g_k * (c_x * g_x + c_y * g_y - g_x * x_des - g_y * y_des)) /
                      (gx2 + gy2);
        const double common_second =
            (3.0 * g_h * g_k *
             std::sqrt(1 / (-9.0 * gh2 + 100.0 * gx2 + 100.0 * gy2)) *
             (c_x * g_y - c_y * g_x - g_y * x_des + g_x * y_des)) /
            (gx2 + gy2);
        const double sln_k_0 = common_first - common_second;
        const double sln_k_1 = common_first + common_second;
        const double jac_sln_0 =
            ComputeCosts(sln_k_0, k_min, k_max, false).jac_k_;
        const double jac_sln_1 =
            ComputeCosts(sln_k_1, k_min, k_max, false).jac_k_;
        jac_0 = (jac_sln_1 < jac_sln_0) ? sln_k_1 : sln_k_0;
      }
      const auto optimal_evals = ComputeCosts(jac_0, k_min, k_max, false);
      const double optimal_cost = optimal_evals.cost_k_;
      const double optimal_jac = optimal_evals.jac_k_;
      const double optimal_hess = optimal_evals.hess_k_;
      const double delta_optimal_cost =
          ComputeCosts(jac_0 + 0.05, k_min, k_max, false).cost_k_;
      const bool concave_up = optimal_cost < delta_optimal_cost;
      const double dist_to_min = std::abs(jac_0 - k_min);
      const double dist_to_max = std::abs(jac_0 - k_max);
      const bool jac_0_in_thresh = InClosedInterval(jac_0, k_min, k_max);
      if (concave_up) {
        if (jac_0_in_thresh) {
          ret_val.optimal_k_in_rng_ = jac_0;
        } else {
          ret_val.optimal_k_in_rng_ =
              (dist_to_max > dist_to_min) ? k_max : k_min;
        }
      } else {
        ret_val.optimal_k_in_rng_ = (dist_to_max > dist_to_min) ? k_min : k_max;
      }
      const double optimal_cost_in_rng =
          ComputeCosts(ret_val.optimal_k_in_rng_, k_min, k_max, false).cost_k_;
      ret_val.optimal_cost_in_rng_ = optimal_cost_in_rng;
      if (print_vars) {
        std::cout << "K:     " << k << " [Cost: " << ret_val.cost_k_
                  << "] [Jac: " << ret_val.jac_k_
                  << "] [Hess: " << ret_val.hess_k_ << "]" << std::endl;
        std::cout << "Opt:   " << jac_0 << " [Cost: " << optimal_cost
                  << "] [Jac: " << optimal_jac << "] [Hess: " << optimal_hess
                  << "]" << std::endl;
        std::cout << "OptIr: " << ret_val.optimal_k_in_rng_
                  << " [Cost: " << optimal_cost_in_rng << "]" << std::endl;
        std::cout << "Jac:   " << ret_val.jac_k_ << std::endl;
        std::cout << "Hess:  " << ret_val.hess_k_ << std::endl;
      }
    }
    return ret_val;
  }
};
CostFcnInfo GenCostFcn(const Vehrs& vehrs, const IndexT desired_idx,
                       const double state_u, const double state_v,
                       const double state_r, const double x_des,
                       const double y_des, const double h_des) {
  CostFcnInfo cost_fcn_info;
  const auto& slc =
      vehrs.slc_vals_.at(desired_idx).Slice(state_u, state_v, state_r);
  const double c_x = vehrs.xy_centers_[desired_idx * 2 + 0] + slc.x_sliced_sum_;
  const double c_y = vehrs.xy_centers_[desired_idx * 2 + 1] + slc.y_sliced_sum_;
  const double c_h = vehrs.h_centers_[desired_idx * 2 + 1] + slc.h_sliced_sum_;
  cost_fcn_info.c_x_ = c_x;
  cost_fcn_info.c_y_ = c_y;
  cost_fcn_info.c_h_ = ToAngle(c_h);
  cost_fcn_info.c_k_ = slc.center_slc_val_;
  cost_fcn_info.g_x_ = slc.slc_x_;
  cost_fcn_info.g_y_ = slc.slc_y_;
  cost_fcn_info.g_h_ = slc.slc_h_;
  cost_fcn_info.g_k_ = slc.slc_val_;
  cost_fcn_info.x_des_ = x_des;
  cost_fcn_info.y_des_ = y_des;
  cost_fcn_info.h_des_ = h_des;
  return cost_fcn_info;
}
struct ObsTestInfo {
  IndexT num_obs_;
  std::vector<double> obs_arr_;
  std::vector<double> obs_centers_;
  ObsTestInfo() : num_obs_{0}, obs_arr_(), obs_centers_() {}
};

struct ZonoTestInfo {
  IndexT num_zonos_;
  IndexT cumsum_num_out_zono_gens_;
  std::unique_ptr<IndexT[]> num_out_zono_gens_arr_;
  std::unique_ptr<IndexT[]> cum_size_arr_;
  std::unique_ptr<double[]> zono_arr_;
  ZonoTestInfo()
      : num_zonos_{0},
        cumsum_num_out_zono_gens_{0},
        num_out_zono_gens_arr_(),
        cum_size_arr_(),
        zono_arr_() {}
};

PointXYH WorldToLocalDesired(const PointXYH& desired_pt_world,
                             const double state_x, const double state_y,
                             const double state_h, const bool mirror) {
  const double mirror_mult = mirror ? -1.0 : 1.0;
  const double cos_h = std::cos(state_h);
  const double sin_h = std::sin(state_h);
  const double trans_x = desired_pt_world.x_ - state_x;
  const double trans_y = desired_pt_world.y_ - state_y;
  const double trans_rot_x = cos_h * trans_x + sin_h * trans_y;
  const double trans_rot_y = -sin_h * trans_x + cos_h * trans_y;
  return PointXYH{
      trans_rot_x, mirror_mult * trans_rot_y,
      ToAngle(mirror_mult * AngleDiff(desired_pt_world.h_, state_h))};
}

ObsTestInfo ConvertObsToZonoWithHeading(
    const Vehrs& vehrs,
    const jsk_recognition_msgs::PolygonArray& latest_read_obs,
    const double agent_x, const double agent_y, const double curr_heading,
    const bool mirror) {
  // Convert obstacles into appropriate zonotopes and rotate to align with
  // current rover heading

  ObsTestInfo obs_info;
  const double mirror_mult = mirror ? -1.0 : 1.0;
  auto& obs_centers = obs_info.obs_centers_;
  auto& obs_generators = obs_info.obs_arr_;
  // Clear out previous obstacles
  obs_centers.clear();
  obs_generators.clear();

  // Store current heading to use later
  const double cos_h = std::cos(mirror_mult * curr_heading);
  const double sin_h = std::sin(mirror_mult * curr_heading);

  const auto pt_to_local = [agent_x, agent_y, cos_h, sin_h, mirror_mult](
                               const geometry_msgs::Point32& pt) -> PointXY {
    // Translated origin
    const double x_trans = pt.x - agent_x;
    const double y_trans = mirror_mult * (pt.y - agent_y);
    // Multiply by negative of current heading to get local frame coords
    const double x_loc = cos_h * x_trans + sin_h * y_trans;
    const double y_loc = -sin_h * x_trans + cos_h * y_trans;
    return {x_loc, y_loc};
  };

  // Note: this is a geometry_msgs/PolygonStamped[]
  const auto polygons = latest_read_obs.polygons;

  for (const geometry_msgs::PolygonStamped& polygon_stamped : polygons) {
    const auto& pts_un = polygon_stamped.polygon.points;
    if (pts_un.size() < 4) {
      // TODO should we warn?
      continue;
    }

    const auto [x_loc0, y_loc0] = pt_to_local(pts_un[0]);
    const auto [x_loc1, y_loc1] = pt_to_local(pts_un[1]);
    const auto [x_loc2, y_loc2] = pt_to_local(pts_un[2]);
    const auto [x_loc3, y_loc3] = pt_to_local(pts_un[3]);

    constexpr double kMinX = -0.5;
    constexpr double kMaxX = 8.0;
    constexpr double kMaxY = 2.7;
    constexpr double kMinY = -kMaxY;
    const std::vector<::roahm::PointXY> obs_pts_local_unsorted = {
        {x_loc0, y_loc0}, {x_loc1, y_loc1}, {x_loc2, y_loc2}, {x_loc3, y_loc3}};
    const auto obs_pts_local = CounterClockwise(obs_pts_local_unsorted);
    std::vector<::roahm::PointXY> obs_pts_local_cycl = obs_pts_local;
    obs_pts_local_cycl.push_back(obs_pts_local_cycl.front());
    std::reverse(obs_pts_local_cycl.begin(), obs_pts_local_cycl.end());
    const bool inside_hull = CouldIntersect(vehrs.hull_, obs_pts_local_cycl);
    const bool inside_bounding_box =
        not(AllGreaterThan({x_loc0, x_loc1, x_loc2, x_loc3}, kMaxX) or
            AllGreaterThan({y_loc0, y_loc1, y_loc2, y_loc3}, kMaxY) or
            AllLessThan({x_loc0, x_loc1, x_loc2, x_loc3}, kMinX) or
            AllLessThan({y_loc0, y_loc1, y_loc2, y_loc3}, kMinY));
    if (!inside_bounding_box) {
      continue;
    }
    // Skip if it's not inside the hull and we've got at least one added
    if (not inside_hull) {
      // std::cout << "NOT INSIDE HULL" << std::endl;
      if ((not inside_hull) and (obs_info.num_obs_ > 0)) {
        // std::cout << "Skipping!" << std::endl;
        continue;
      } else {
        // std::cout << "OBS CURRENTLY: " << obs_info.num_obs_ << std::endl;
      }
    }
    /*
    if (!inside_hull) {
      //if (agent_x < -6.2 and agent_x > -6.6 and agent_y > 39.4) {
        if ( AllGreaterThan({pts_un[0].y, pts_un[1].y, pts_un[2].y,
    pts_un[3].y}, 39.3) ) { std::string out_str = "\n"; out_str += "STATE: " +
    std::to_string(agent_x) + ", " + std::to_string(agent_y) + "\n"; out_str +=
    "obs_pts = [\n"; for (const auto &pt : obs_pts_local_cycl) { out_str +=
    std::to_string(pt.x_) + ", " + std::to_string(pt.y_) + "\n";
          }
          out_str += "]';\nhull_pts = [\n";
          for (const auto &pt : vehrs.hull_) {
            out_str += std::to_string(pt.x_) + ", " + std::to_string(pt.y_) +
    "\n";
          }
          out_str += "]';\ncheck_convhull_intersect(hull_pts, obs_pts)\n";
          out_str += "GLOBAL\n";
          for (const auto &pt: pts_un) {
            out_str += std::to_string(pt.x) + ", " + std::to_string(pt.y) +
    "\n";
          }
          std::cout << out_str << std::endl;
        }
      //}
      //continue;
    }
    */

    std::array<PointXY, 4> pts;
    pts[0] =
        PointXY{pts_un[0].x - agent_x, mirror_mult * (pts_un[0].y - agent_y)};
    pts[1] =
        PointXY{pts_un[1].x - agent_x, mirror_mult * (pts_un[1].y - agent_y)};
    pts[2] =
        PointXY{pts_un[2].x - agent_x, mirror_mult * (pts_un[2].y - agent_y)};
    pts[3] =
        PointXY{pts_un[3].x - agent_x, mirror_mult * (pts_un[3].y - agent_y)};

    std::array<PointXY, 4> points;
    // Choose an origin point and offset all points to that origin
    // Also translate to start_point and flip y if mirrored
    PointXY pt_init{pts[0].x_, pts[0].y_};
    points[0] = PointXY{pts[0].x_ - pt_init.x_, pts[0].y_ - pt_init.y_};
    points[1] = PointXY{pts[1].x_ - pt_init.x_, pts[1].y_ - pt_init.y_};
    points[2] = PointXY{pts[2].x_ - pt_init.x_, pts[2].y_ - pt_init.y_};
    points[3] = PointXY{pts[3].x_ - pt_init.x_, pts[3].y_ - pt_init.y_};

    // Find a pair of non-origin points that are ~90 deg apart from each
    // other w.r.t. the origin (diagonals). We will align these to the
    // xy axes.
    const double theta_1 = std::atan2(points[1].y_, points[1].x_);
    const double theta_2 = std::atan2(points[2].y_, points[2].x_);
    const double theta_3 = std::atan2(points[3].y_, points[3].x_);
    auto closeness_to_90 = [](double theta_a, double theta_b) {
      return std::abs(
          AngleDiff(std::abs(AngleDiff(theta_a, theta_b)), M_PI / 2.0));
    };
    const double delta2_3 = closeness_to_90(theta_2, theta_3);
    constexpr double kThetaEps = 1.0e-3;
    const double rz_theta = (delta2_3 < kThetaEps) ? -theta_2 : -theta_1;
    const double cos_rz_theta = std::cos(rz_theta);
    const double sin_rz_theta = std::sin(rz_theta);

    // Rotate values to align points on xy plane
    // rz = [cos -sin; sin cos]
    std::array<double, 4> x_vals;
    std::array<double, 4> y_vals;
    const double buffer_dist_each_side = 0.10;
    const double buffer_dist_total = buffer_dist_each_side * 2.0;
    for (int i = 0; i < 4; ++i) {
      x_vals[i] = cos_rz_theta * points[i].x_ - sin_rz_theta * points[i].y_ +
                  buffer_dist_total;
      y_vals[i] = sin_rz_theta * points[i].x_ + cos_rz_theta * points[i].y_ +
                  buffer_dist_total;
    }

    // Compute center point and generators
    const auto minmax_x = std::minmax_element(x_vals.begin(), x_vals.end());
    const auto minmax_y = std::minmax_element(y_vals.begin(), y_vals.end());
    const double min_x = *minmax_x.first;
    const double max_x = *minmax_x.second;
    const double min_y = *minmax_y.first;
    const double max_y = *minmax_y.second;
    const double range_x = max_x - min_x;
    const double range_y = max_y - min_y;

    // Un-rotate the generators and center, un-offset the center
    // [cos sin; -sin cos]
    const double gen1_x = cos_rz_theta * range_x / 2.0;
    const double gen1_y = -sin_rz_theta * range_x / 2.0;
    const double gen2_x = sin_rz_theta * range_y / 2.0;
    const double gen2_y = cos_rz_theta * range_y / 2.0;
    const double c_x_init = range_x / 2.0 + min_x;
    const double c_y_init = range_y / 2.0 + min_y;
    const double c_x =
        (cos_rz_theta * c_x_init + sin_rz_theta * c_y_init) + pt_init.x_;
    const double c_y =
        (-sin_rz_theta * c_x_init + cos_rz_theta * c_y_init) + pt_init.y_;

    // Rotate by negative (transpose rot mat) of current robot heading
    const double gen1_x_with_heading = cos_h * gen1_x + sin_h * gen1_y;
    const double gen1_y_with_heading = -sin_h * gen1_x + cos_h * gen1_y;
    const double gen2_x_with_heading = cos_h * gen2_x + sin_h * gen2_y;
    const double gen2_y_with_heading = -sin_h * gen2_x + cos_h * gen2_y;
    const double c_x_with_heading = cos_h * c_x + sin_h * c_y;
    const double c_y_with_heading = -sin_h * c_x + cos_h * c_y;

    // Write to output arrays
    obs_centers.push_back(c_x_with_heading);
    obs_centers.push_back(c_y_with_heading);
    obs_generators.push_back(gen1_x_with_heading);
    obs_generators.push_back(gen1_y_with_heading);
    obs_generators.push_back(gen2_x_with_heading);
    obs_generators.push_back(gen2_y_with_heading);
    ++obs_info.num_obs_;
  }
  return obs_info;
}

ZonoTestInfo FillZonosFromVehrs(const Vehrs& vehrs) {
  ZonoTestInfo ret;
  // Number of total input zonotopes (i.e. the "size" of the FRS)
  IndexT num_zonos = vehrs.GetNumZonos();
  ret.num_zonos_ = num_zonos;

  // Number of output constraints for each input zonotope, per obstacle
  ret.num_out_zono_gens_arr_ = std::make_unique<IndexT[]>(num_zonos);
  auto& num_out_zono_gens_arr = ret.num_out_zono_gens_arr_;
  ret.cum_size_arr_ = std::make_unique<IndexT[]>(num_zonos);
  auto& cum_size_arr = ret.cum_size_arr_;

  for (IndexT i = 0; i < num_zonos; ++i) {
    num_out_zono_gens_arr[i] = vehrs.zono_sizes_.at(i) + 2;
  }

  // Compute total sizes
  IndexT cumsum_num_out_zono_gens = 0;
  for (IndexT i = 0; i < num_zonos; ++i) {
    cum_size_arr[i] = cumsum_num_out_zono_gens;
    cumsum_num_out_zono_gens += num_out_zono_gens_arr[i];
  }
  ret.cumsum_num_out_zono_gens_ = cumsum_num_out_zono_gens;

  // Each generator has x, y so mult by 2 for size
  IndexT zono_arr_size = 2 * cumsum_num_out_zono_gens;

  // Create arrays
  ret.zono_arr_ = std::make_unique<double[]>(zono_arr_size);
  auto& zono_arr = ret.zono_arr_;

  for (IndexT zono_idx = 0; zono_idx < num_zonos; ++zono_idx) {
    const IndexT out_start_idx = 2 * cum_size_arr[zono_idx];
    const IndexT num_out_gens = num_out_zono_gens_arr[zono_idx];
    const IndexT num_out_elts = num_out_gens * 2;
    const IndexT num_in_gens = num_out_gens - 2;
    const IndexT num_in_elts = num_in_gens * 2;
    const IndexT in_start_idx = out_start_idx - 4 * zono_idx;
    for (IndexT i = 0; i < num_in_elts; ++i) {
      zono_arr[out_start_idx + i] = vehrs.zono_xy_.at(in_start_idx + i);
    }
    for (IndexT i = num_in_elts; i < num_out_elts; ++i) {
      zono_arr[out_start_idx + i] = 0;
    }
  }
  return ret;
}
inline std::pair<double, double> NormPair(double a, double b) {
  const double norm_recip = 1.0 / std::sqrt(a * a + b * b);
  return std::make_pair(a * norm_recip, b * norm_recip);
}

struct Constraints {
  IndexT num_a_elts_;
  IndexT num_b_elts_;
  std::shared_ptr<double[]> a_con_arr_;
  std::shared_ptr<double[]> b_con_arr_;
  std::vector<IndexT> zono_startpoints_;
  std::vector<IndexT> zono_obs_sizes_;
  Constraints() = default;
};

Constraints GenerateConstraints(const Vehrs& vehrs, const ObsTestInfo& obs_info,
                                const double state_u, const double state_v,
                                const double state_r) {
  const IndexT num_obs_gens = 2;
  const auto filled_test_zonos = FillZonosFromVehrs(vehrs);
  const auto num_obs = obs_info.num_obs_;
  const auto& obs_arr = obs_info.obs_arr_;
  const auto& obs_centers = obs_info.obs_centers_;

  const auto num_zonos = filled_test_zonos.num_zonos_;
  const auto& cumsum_num_out_zono_gens =
      filled_test_zonos.cumsum_num_out_zono_gens_;
  const auto& num_out_zono_gens_arr = filled_test_zonos.num_out_zono_gens_arr_;
  const auto& cum_size_arr = filled_test_zonos.cum_size_arr_;
  const auto& zono_arr = filled_test_zonos.zono_arr_;
  if (num_obs < 1 || num_zonos < 1) {
    return Constraints{};
  }

  const IndexT total_d_size = cumsum_num_out_zono_gens;
  const IndexT total_c_size = 2 * cumsum_num_out_zono_gens;
  double* const delta_d_arr = new double[total_d_size];
  double* const c_arr = new double[total_c_size];
  for (IndexT i = 0; i < total_d_size; ++i) {
    delta_d_arr[i] = 0;
  }

  // Compute initial values.
  for (IndexT zono_idx = 0; zono_idx < num_zonos; ++zono_idx) {
    // Number of output generators for current zonotope
    const IndexT curr_n = num_out_zono_gens_arr[zono_idx];

    // Offsets for aggregated array output
    const IndexT n_start_offset = cum_size_arr[zono_idx];
    const IndexT d_start_idx = n_start_offset;
    const IndexT zono_start_idx = 2 * n_start_offset;
    const IndexT c_start_idx = 2 * n_start_offset;

    double* const curr_delta_d_arr = delta_d_arr + d_start_idx;
    double* const curr_c_arr = c_arr + c_start_idx;
    double* const curr_zono_arr = zono_arr.get() + zono_start_idx;

    const IndexT max_r_without_obs = curr_n - num_obs_gens;
    // delta_d(r, c) = abs(C(r,:) * G(:,c))
    for (IndexT r = 0; r < max_r_without_obs; ++r) {
      curr_delta_d_arr[r] = 0;

      // C(r,:) = Normalize([-G(1, r); G(2, r)]), one-indexed
      const auto [c_r0, c_r1] =
          NormPair(-curr_zono_arr[r * 2 + 1], curr_zono_arr[r * 2]);

      // Save to C array
      curr_c_arr[r * 2] = c_r0;
      curr_c_arr[r * 2 + 1] = c_r1;

      for (IndexT c = 0; c < curr_n - num_obs_gens; ++c) {
        const double g_c0 = curr_zono_arr[c * 2];
        const double g_c1 = curr_zono_arr[c * 2 + 1];
        curr_delta_d_arr[r] += std::abs((c_r0 * g_c0) + (c_r1 * g_c1));
      }
    }
  }

  // TODO allow for multiple slice
  const IndexT pa_size_per_obs = (2 * total_c_size);  // PA = [-C; C]
  // Pb = [d + deltaD; -d + deltaD]
  const IndexT pb_size_per_obs = (2 * total_d_size);
  const IndexT total_pa_size = pa_size_per_obs * num_obs;
  const IndexT total_pb_size = pb_size_per_obs * num_obs;

  constexpr int kNumSlcGens = 1;
  static_assert(kNumSlcGens == 1);  // Not implemented for other values yet
  const IndexT total_a_con_size = total_pb_size * kNumSlcGens;
  double* const d_arr = new double[total_d_size];
  double* const pa_arr = new double[total_pa_size];
  Constraints ret;
  ret.zono_startpoints_.push_back(0);
  for (IndexT zono_idx = 0; zono_idx < num_zonos; ++zono_idx) {
    for (IndexT obs_idx = 0; obs_idx < num_obs; ++obs_idx) {
      const auto prev_start = ret.zono_startpoints_.back();
      const auto additional_gens = 2 * num_out_zono_gens_arr[zono_idx];
      if (!(zono_idx == num_zonos - 1 && obs_idx == num_obs - 1)) {
        ret.zono_startpoints_.push_back(prev_start + additional_gens);
      }
      ret.zono_obs_sizes_.push_back(additional_gens);
    }
  }
  assert(ret.zono_obs_sizes_.size() == num_zonos * num_obs);
  assert(ret.zono_startpoints_.size() == num_zonos * num_obs);
  ret.num_a_elts_ = total_a_con_size;
  ret.num_b_elts_ = total_pb_size;
  ret.a_con_arr_ = std::make_unique<double[]>(total_a_con_size);
  ret.b_con_arr_ = std::make_unique<double[]>(total_pb_size);
  double* const pb_arr = ret.b_con_arr_.get();
  double* const a_con_arr = ret.a_con_arr_.get();

  // Initalize elements to zero
  for (IndexT i = 0; i < total_pa_size; ++i) {
    pa_arr[i] = 0;
  }
  for (IndexT i = 0; i < total_pb_size; ++i) {
    pb_arr[i] = 0;
  }

  for (IndexT zono_idx = 0; zono_idx < num_zonos; ++zono_idx) {
    const IndexT curr_n = num_out_zono_gens_arr[zono_idx];
    const IndexT n_start_offset = cum_size_arr[zono_idx];
    const IndexT c_start_idx = 2 * n_start_offset;
    const IndexT zono_start_idx = 2 * n_start_offset;
    const IndexT d_start_idx = n_start_offset;
    const IndexT max_r_without_obs = curr_n - num_obs_gens;
    double* const curr_c_arr = c_arr + c_start_idx;
    double* const curr_zono_arr = zono_arr.get() + zono_start_idx;
    double* const curr_delta_d_arr = delta_d_arr + d_start_idx;

    const auto sliced_info =
        vehrs.slc_vals_.at(zono_idx).Slice(state_u, state_v, state_r);
    const double slc_x = sliced_info.slc_x_;
    const double slc_y = sliced_info.slc_y_;
    // assert(sliced_info.lambda_ok_);
    // assert(sliced_info.slc_xy_set_);
    const double zono_center_x =
        sliced_info.x_sliced_sum_ + vehrs.xy_centers_.at(zono_idx * 2);
    const double zono_center_y =
        sliced_info.y_sliced_sum_ + vehrs.xy_centers_.at(zono_idx * 2 + 1);

    for (IndexT obs_idx = 0; obs_idx < num_obs; ++obs_idx) {
      const IndexT curr_row_out_start =
          (cum_size_arr[zono_idx] * num_obs) + (curr_n * obs_idx);

      // 2 for plus/minus components, 2 for obs dimensionality
      const IndexT pa_start_idx = 2 * 2 * curr_row_out_start;
      const IndexT pb_start_idx = 2 * curr_row_out_start;
      const double obs_center_x = obs_centers[obs_idx * 2];
      const double obs_center_y = obs_centers[obs_idx * 2 + 1];

      double* const curr_pa_arr = pa_arr + pa_start_idx;
      double* const curr_pb_arr = pb_arr + pb_start_idx;

      // Copy the non-obstacle C portion
      for (IndexT r = 0; r < max_r_without_obs; ++r) {
        curr_pa_arr[r * 2] = curr_c_arr[r * 2];
        curr_pa_arr[r * 2 + 1] = curr_c_arr[r * 2 + 1];
        curr_pa_arr[(r + curr_n) * 2] = -curr_c_arr[r * 2];
        curr_pa_arr[(r + curr_n) * 2 + 1] = -curr_c_arr[r * 2 + 1];
      }

      // Copy the precomputed delta_d values
      for (IndexT r = 0; r < curr_n; ++r) {
        const double delta_d_val = curr_delta_d_arr[r];
        curr_pb_arr[r] = delta_d_val;
        curr_pb_arr[r + curr_n] = delta_d_val;
      }

      for (IndexT obs_gen_idx = 0; obs_gen_idx < num_obs_gens; ++obs_gen_idx) {
        const IndexT obs_gen_start_idx =
            (obs_idx * num_obs_gens * 2) + obs_gen_idx * 2;
        const double obs_x = obs_arr[obs_gen_start_idx + 0];
        const double obs_y = obs_arr[obs_gen_start_idx + 1];
        const auto [c0_val, c1_val] = NormPair(-obs_y, obs_x);

        // Copy the normalized obstacle into C
        const IndexT pa_obs_start = (max_r_without_obs + obs_gen_idx) * 2;
        curr_pa_arr[pa_obs_start + 0] = c0_val;
        curr_pa_arr[pa_obs_start + 1] = c1_val;

        // Set the -C portion of PA
        curr_pa_arr[pa_obs_start + (curr_n * 2) + 0] = -c0_val;
        curr_pa_arr[pa_obs_start + (curr_n * 2) + 1] = -c1_val;
      }

      for (IndexT r = 0; r < curr_n - num_obs_gens; ++r) {
        for (IndexT obs_gen_idx = 0; obs_gen_idx < num_obs_gens;
             ++obs_gen_idx) {
          // In this, obs_gen_idx corresponds to the last num_obs_gens columns
          const IndexT obs_gen_start_idx =
              (obs_idx * num_obs_gens * 2) + obs_gen_idx * 2;
          const double obs_x = obs_arr[obs_gen_start_idx + 0];
          const double obs_y = obs_arr[obs_gen_start_idx + 1];
          const double pa_val_x = curr_pa_arr[r * 2 + 0];
          const double pa_val_y = curr_pa_arr[r * 2 + 1];
          const double abs_cg_val =
              std::abs(pa_val_x * obs_x + pa_val_y * obs_y);
          curr_pb_arr[r] += abs_cg_val;
          curr_pb_arr[r + curr_n] += abs_cg_val;
        }
      }

      // Sum from the bottom num_obs_gen x num_obs_gen corner of abs(CG) matrix
      for (IndexT c_obs_gen_idx = 0; c_obs_gen_idx < num_obs_gens;
           ++c_obs_gen_idx) {
        const IndexT r = curr_n - (num_obs_gens - c_obs_gen_idx);
        const IndexT c_obs_gen_start_idx =
            (obs_idx * num_obs_gens * 2) + c_obs_gen_idx * 2;
        const auto [c_obs_x, c_obs_y] = NormPair(
            obs_arr[c_obs_gen_start_idx], obs_arr[c_obs_gen_start_idx + 1]);
        for (IndexT g_obs_gen_idx = 0; g_obs_gen_idx < num_obs_gens;
             ++g_obs_gen_idx) {
          const IndexT g_obs_gen_start_idx =
              (obs_idx * num_obs_gens * 2) + g_obs_gen_idx * 2;
          const double g_obs_x = obs_arr[g_obs_gen_start_idx + 0];
          const double g_obs_y = obs_arr[g_obs_gen_start_idx + 1];
          curr_pb_arr[r] += std::abs(-c_obs_y * g_obs_x + c_obs_x * g_obs_y);
        }
      }

      for (IndexT c_obs_gen_idx = 0; c_obs_gen_idx < num_obs_gens;
           ++c_obs_gen_idx) {
        const IndexT r = curr_n - (num_obs_gens - c_obs_gen_idx);
        const IndexT c_obs_gen_start_idx =
            (obs_idx * num_obs_gens * 2) + c_obs_gen_idx * 2;
        const auto [c_obs_x, c_obs_y] = NormPair(
            obs_arr[c_obs_gen_start_idx], obs_arr[c_obs_gen_start_idx + 1]);
        for (IndexT c = 0; c < curr_n - num_obs_gens; ++c) {
          const double g_x = curr_zono_arr[c * 2 + 0];
          const double g_y = curr_zono_arr[c * 2 + 1];
          curr_pb_arr[r] += std::abs(-c_obs_y * g_x + c_obs_x * g_y);
        }
      }

      for (IndexT r = 0; r < curr_n; ++r) {
        curr_pb_arr[r + curr_n] = curr_pb_arr[r];
      }

      const double center_delta_x = obs_center_x - zono_center_x;
      const double center_delta_y = obs_center_y - zono_center_y;

      for (IndexT r = 0; r < curr_n; ++r) {
        const double d_val = curr_pa_arr[r * 2 + 0] * center_delta_x +
                             curr_pa_arr[r * 2 + 1] * center_delta_y;
        curr_pb_arr[r] += d_val;
        curr_pb_arr[r + curr_n] -= d_val;
        const double a_con_val =
            (curr_pa_arr[r * 2 + 0] * slc_x) + (curr_pa_arr[r * 2 + 1] * slc_y);
        a_con_arr[pb_start_idx + r] = a_con_val;
        a_con_arr[pb_start_idx + r + curr_n] = -a_con_val;
      }
    }
  }

  // Cleanup
  delete[] c_arr;
  delete[] delta_d_arr;

  // Cleanup [obstacle portion specific]
  delete[] d_arr;
  delete[] pa_arr;

  return ret;
}
void PrintConstraints(const Constraints& constraints, std::ostream& o_stream) {
  const auto& a_con_arr = constraints.a_con_arr_;
  const auto& pb_arr = constraints.b_con_arr_;
  o_stream << "bvals_in = [";
  IndexT endpoint = constraints.num_b_elts_;
  for (IndexT i = 0; i < endpoint; ++i) {
    // o_stream << "pb_arr[" << i << "]: " << pb_arr[i] << std::endl;
    o_stream << pb_arr[i];
    if (i + 1 != endpoint) {
      o_stream << ", ";
    }
  }
  o_stream << "];\n";

  o_stream << "avals_in = [";
  endpoint = constraints.num_a_elts_;
  for (IndexT i = 0; i < endpoint; ++i) {
    o_stream << a_con_arr[i];
    if (i + 1 != endpoint) {
      o_stream << ", ";
    }
  }
  o_stream << "];\n";
}

void WriteObsTestInfo(const ObsTestInfo& obs_info, std::string fname) {
  std::ofstream f_out;
  f_out.open(fname, std::ofstream::out);

  f_out << "function [num_obs_in, obs_arr_in] = load_obs_info_latest()\n";
  f_out << "num_obs_in = " << obs_info.num_obs_ << ";\n";
  f_out << "obs_arr_in = {};\n";
  for (int i = 0; i < obs_info.num_obs_; ++i) {
    f_out << "obs_arr_in{end+1} = [" << obs_info.obs_centers_[2 * i + 0] << ", "
          << obs_info.obs_arr_[4 * i + 0] << ", "
          << obs_info.obs_arr_[4 * i + 2] << ";\n"
          << obs_info.obs_centers_[2 * i + 1] << ", "
          << obs_info.obs_arr_[4 * i + 1] << ", "
          << obs_info.obs_arr_[4 * i + 3] << "];\n";
  }
  /*
   * TODO CLEANUP
  f_out << "obs_arr_in = [";
  for (int i = 0; i < obs_info.obs_arr_.size(); ++i) {
    f_out << obs_info.obs_arr_.at(i);
    if (i +1 != obs_info.obs_arr_.size()) {
      f_out << ",";
    }
  }
  f_out << "];\n";
  f_out << "obs_centers_in = [";
  for (int i = 0; i < obs_info.obs_centers_.size(); ++i) {
    f_out << obs_info.obs_centers_.at(i);
    if (i +1 != obs_info.obs_centers_.size()) {
      f_out << ",";
    }
  }
  f_out << "];";
  */
  f_out << "\nend";
  f_out.close();
}
void WriteConstraints(const Constraints& constraints) {
  std::ofstream f_out;
  f_out.open("/home/TODO/constraint_vals.m", std::ofstream::out);
  PrintConstraints(constraints, f_out);
  f_out.close();
}

void PrintConstraints(const Constraints& constraints) {
  PrintConstraints(constraints, std::cout);
}
}  // namespace roahm

#endif  // ROAHM_GENCON_H_
