#include "gencon.hpp"

#include <algorithm>         // for max, minmax_element
#include <array>             // for array, array<>::value...
#include <cassert>           // for assert
#include <cmath>             // for atan2, cos, sin, sqrt
#include <cstdlib>           // for abs, size_t
#include <fstream>           // for ifstream, ofstream
#include <initializer_list>  // for initializer_list
#include <iostream>          // for operator<<, basic_ost...
#include <utility>           // for pair, make_pair

#include "geometry_msgs/Point32.h"              // for Point32, Point32_
#include "geometry_msgs/Polygon.h"              // for Polygon_, Polygon_<>:...
#include "geometry_msgs/PolygonStamped.h"       // for PolygonStamped, Polyg...
#include "jsk_recognition_msgs/PolygonArray.h"  // for PolygonArray, Polygon...
#include "point_xy.hpp"                         // for PointXY
#include "simple_util.hpp"                      // for ToAngle, Square, Angl...

namespace roahm {

namespace {

std::vector<PointXY> Clockwise(const std::vector<PointXY>& unsorted) {
  if (unsorted.empty()) {
    return {};
  }
  double c_x = 0.0;
  double c_y = 0.0;
  for (const auto& pt : unsorted) {
    c_x += pt.x_;
    c_y += pt.y_;
  }
  c_x /= static_cast<double>(unsorted.size());
  c_y /= static_cast<double>(unsorted.size());
  auto ret = unsorted;
  std::sort(ret.begin(), ret.end(),
            [c_x, c_y](const PointXY& a, const PointXY& b) {
              const double theta_a =
                  ToAngle(std::atan2(a.x_ - c_x, a.y_ - c_y) + M_PI);
              const double theta_b =
                  ToAngle(std::atan2(b.x_ - c_x, b.y_ - c_y) + M_PI);
              return theta_a < theta_b;
            });
  return ret;
}

std::vector<PointXY> CounterClockwise(const std::vector<PointXY>& unsorted) {
  auto ret = Clockwise(unsorted);
  std::reverse(ret.begin(), ret.end());
  return ret;
}

/// Checks whether each element of a set are strictly less than a value.
/// \param vals the values to check
/// \param min_v the value to compare against
/// \return true iff \f$ \forall v \in \textrm{vals}, v < \textrm{max_v} \f$,
/// false otherwise or if \p min_v is a NaN
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

/// Checks whether each element of a list are strictly greater than a value.
/// \param vals the values to check
/// \param max_v the value to compare against
/// \return true iff \f$ \forall v \in \textrm{vals}, v > \textrm{max_v} \f$,
/// false otherwise or if \p max_v is a NaN
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

/// Takes a pair \f$ (a, b) \f$ and returns a normalized pair,
/// \f$ (\frac{a}{\sqrt{a^2 + b^2}}, \frac{b}{\sqrt{a^2 + b^2}}) \f$
/// \param a the first number of the set
/// \param b the second number of the set
/// \return the inputs normalized
/// \f$ (\frac{a}{\sqrt{a^2 + b^2}}, \frac{b}{\sqrt{a^2 + b^2}}) \f$
inline std::pair<double, double> NormPair(double a, double b) {
  const double norm_recip = 1.0 / Norm(a, b);
  return std::make_pair(a * norm_recip, b * norm_recip);
}

// TODO make sure that this is necessary, compare with constraint generation
/// Fills arrays with zonotope information from FRSes, leaving room for obstacle
/// generators to be appended for constraint generation
/// \param vehrs the forward reachable set
/// \return filled arrays containing the reachable sets' zonotope information
/// and space to append obstacle generators for constraint generation
ZonoInfo FillZonosFromVehrs(const Vehrs& vehrs) {
  // part of gen constraints (maybe)

  // SUS

  ZonoInfo ret;
  // Number of total input zonotopes (i.e. the "size" of the FRS)
  IndexT num_zonos = vehrs.GetNumZonos();
  ret.num_zonos_ = num_zonos;

  // Number of output constraints for each input zonotope, per obstacle
  ret.num_out_zono_gens_arr_ = std::make_unique<IndexT[]>(num_zonos);
  auto& num_out_zono_gens_arr = ret.num_out_zono_gens_arr_;
  ret.cum_size_arr_ = std::make_unique<IndexT[]>(num_zonos);
  auto& cum_size_arr = ret.cum_size_arr_;

  for (IndexT i = 0; i < num_zonos; ++i) {
    // Since we always have two generators per obstacles, we leave room for them
    num_out_zono_gens_arr[i] = vehrs.zono_sizes_.at(i) + 2;
  }

  // Compute total sizes
  IndexT cumsum_num_out_zono_gens = 0;
  for (IndexT i = 0; i < num_zonos; ++i) {
    cum_size_arr[i] = cumsum_num_out_zono_gens;
    cumsum_num_out_zono_gens += num_out_zono_gens_arr[i];
  }
  ret.cumsum_num_out_zono_gens_ = cumsum_num_out_zono_gens;

  // Each generator has x, y so multiply by 2 for size
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

    // Fill the forward reachable set zonotopes
    for (IndexT i = 0; i < num_in_elts; ++i) {
      zono_arr[out_start_idx + i] = vehrs.zono_xy_.at(in_start_idx + i);
    }
    double x_sum = 0.0;
    double y_sum = 0.0;
    for (IndexT i = 0; i < (num_in_elts/2); ++i) {
      const double x = vehrs.zono_xy_.at(in_start_idx + (2*i) + 0);
      const double y = vehrs.zono_xy_.at(in_start_idx + (2*i) + 1);
      x_sum += x;
      y_sum += y;
    }

    // Leave the obstacle generators empty for now
    for (IndexT i = num_in_elts; i < num_out_elts; ++i) {
      zono_arr[out_start_idx + i] = 0;
    }
  }
  return ret;
}

}  // namespace

// TODO TODO TODO FIXME
ObsInfo ConvertObsToZonoWithHeading(
    const jsk_recognition_msgs::PolygonArray& latest_read_obs,
    const PointXYH& local_frame, const bool mirror) {
  const auto agent_x = local_frame.x_;
  const auto agent_y = local_frame.y_;
  const auto curr_heading = local_frame.h_;
  // Convert obstacles into appropriate zonotopes and rotate to align with
  // current rover heading

  ObsInfo obs_info{};
  const double mirror_mult = mirror ? -1.0 : 1.0;
  // TODO REMOVE auto& obs_centers = obs_info.obs_centers_;
  // TODO REMOVE auto& obs_generators = obs_info.obs_arr_;
  // Clear out previous obstacles
  // TODO REMOVE obs_centers.clear();
  // TODO REMOVE obs_generators.clear();
  obs_info.ClearAll();

  // Store current heading to use later
  const double cos_h = std::cos(mirror_mult * curr_heading);
  const double sin_h = std::sin(mirror_mult * curr_heading);

  const auto pt_to_local = [agent_x, agent_y, cos_h, sin_h,
                            mirror_mult](  // JL: weird logic, but correct...
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
      // TODO should we warn?  // JL: what does it mean? has to be a box?
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
    const auto obs_pts_local =
        CounterClockwise(obs_pts_local_unsorted);  // JL: checked
    std::vector<::roahm::PointXY> obs_pts_local_cycl = obs_pts_local;
    obs_pts_local_cycl.push_back(
        obs_pts_local_cycl.front());  // JL: make the box closed
    std::reverse(
        obs_pts_local_cycl.begin(),
        obs_pts_local_cycl
            .end());  // not sure why reverse againn, but fine...  1 -- 2
    //                                           |    |
    //                                           4 -- 3
    const bool inside_bounding_box =
        not(AllGreaterThan({x_loc0, x_loc1, x_loc2, x_loc3}, kMaxX) or
            AllGreaterThan({y_loc0, y_loc1, y_loc2, y_loc3}, kMaxY) or
            AllLessThan({x_loc0, x_loc1, x_loc2, x_loc3}, kMinX) or
            AllLessThan({y_loc0, y_loc1, y_loc2, y_loc3}, kMinY));
    if (true and !inside_bounding_box) {  // JL: filtering not triggered
      continue;
    }
    // JL: remove ghost obstacles that intersect with rover's footprint (lidar
    // issue). rover size: 56cm * 30cm.
    const bool outside_footprint =
        AllGreaterThan({x_loc0, x_loc1, x_loc2, x_loc3}, 0.28) or
        AllGreaterThan({y_loc0, y_loc1, y_loc2, y_loc3}, 0.15) or
        AllLessThan({x_loc0, x_loc1, x_loc2, x_loc3}, -0.28) or
        AllLessThan({y_loc0, y_loc1, y_loc2, y_loc3}, -0.15);
    if (!outside_footprint) {
      continue;
    }

    std::array<PointXY, 4>
        pts;  // JL: pts are stored in a CCW fashion, but pts[0] is not
    // guaranteed to begin from a specific location
    pts[0] =
        PointXY{pts_un[0].x - agent_x, mirror_mult * (pts_un[0].y - agent_y)};
    pts[1] =
        PointXY{pts_un[1].x - agent_x, mirror_mult * (pts_un[1].y - agent_y)};
    pts[2] =
        PointXY{pts_un[2].x - agent_x, mirror_mult * (pts_un[2].y - agent_y)};
    pts[3] =
        PointXY{pts_un[3].x - agent_x, mirror_mult * (pts_un[3].y - agent_y)};

#if not SIMULATION

    // The minimum distance at which we should be using max buffer
    constexpr double kDistForMaxBuffer = 4.0;
    // Maximum buffer to add to each side of an obstacle.
    constexpr double kMaxBufferEachSide = 0.2;
    constexpr double kMinBufferEachSide = 0.01;

    // Distance from agent to center point.
    // Since these are in local frame, mean coordinate is mean distance
    const double mean_dist_x =
        (pts[0].x_ + pts[1].x_ + pts[2].x_ + pts[3].x_) / 4.0;
    const double mean_dist_y =
        (pts[0].y_ + pts[1].y_ + pts[2].y_ + pts[3].y_) / 4.0;
    const double dist_to_mean =
        std::sqrt(Square(mean_dist_x) + Square(mean_dist_y));
    const double buffer_dist_each_side =
        Clamp(kMaxBufferEachSide * dist_to_mean / kDistForMaxBuffer,
              kMinBufferEachSide, kMaxBufferEachSide);

#endif
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
    // TODO FIX THIS AREA?
    // const double theta_2 = std::atan2(points[2].y_, points[2].x_);
    // const double theta_3 = std::atan2(points[3].y_, points[3].x_);
    // auto closeness_to_90 = [](double theta_a, double theta_b) {
    //   return std::abs(
    //       AngleDiff(std::abs(AngleDiff(theta_a, theta_b)), M_PI / 2.0));
    // };
    // const double delta2_3 = closeness_to_90(theta_2, theta_3);
    // constexpr double kThetaEps = 1.0e-3;
    const double rz_theta = -theta_1;  //(delta2_3 < kThetaEps) ? -theta_2 :
    //-theta_1;        // JL testing!!!!!
    const double cos_rz_theta = std::cos(rz_theta);
    const double sin_rz_theta = std::sin(rz_theta);

    // Rotate values to align points on xy plane
    // rz = [cos -sin; sin cos]
    std::array<double, 4> x_vals;
    std::array<double, 4> y_vals;
#if SIMULATION
    const double buffer_dist_each_side = 0.05;
#endif
    const double buffer_dist_total = buffer_dist_each_side * 2.0;

    const bool JL_debug = true;
    for (int i = 0; i < 4; ++i) {
      x_vals[i] = cos_rz_theta * points[i].x_ -
                  sin_rz_theta * points[i].y_ +  // JL: sus
                  (not JL_debug) * buffer_dist_total;
      y_vals[i] = sin_rz_theta * points[i].x_ + cos_rz_theta * points[i].y_ +
                  (not JL_debug) * buffer_dist_total;
    }

    // Compute center point and generators
    const auto minmax_x = std::minmax_element(x_vals.begin(), x_vals.end());
    const auto minmax_y = std::minmax_element(y_vals.begin(), y_vals.end());
    const double min_x = *minmax_x.first;
    const double max_x = *minmax_x.second;
    const double min_y = *minmax_y.first;
    const double max_y = *minmax_y.second;
    const double range_x = max_x - min_x + JL_debug * buffer_dist_total;
    const double range_y = max_y - min_y + JL_debug * buffer_dist_total;

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
    obs_info.PushObs(c_x_with_heading, c_y_with_heading, gen1_x_with_heading, gen1_y_with_heading, gen2_x_with_heading, gen2_y_with_heading);
    // TODO REMOVE obs_centers.push_back(c_x_with_heading);
    // TODO REMOVE obs_centers.push_back(c_y_with_heading);
    // TODO REMOVE obs_generators.push_back(gen1_x_with_heading);
    // TODO REMOVE obs_generators.push_back(gen1_y_with_heading);
    // TODO REMOVE obs_generators.push_back(gen2_x_with_heading);
    // TODO REMOVE obs_generators.push_back(gen2_y_with_heading);
    //++obs_info.num_obs_;
  }
  return obs_info;
}

Constraints GenerateConstraints(const Vehrs& vehrs, const ObsInfo& obs_info) {
  // SUS
  // TODO: get rid of shared and unique pointers and replace them with vectors

  const IndexT num_obs_gens = 2;
  const auto filled_test_zonos = FillZonosFromVehrs(vehrs);
  const auto num_obs = obs_info.GetNumObs();

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

      const auto g_r0 = curr_zono_arr[r * 2];
      const auto g_r1 = curr_zono_arr[r * 2 + 1];
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
  ret.a_con_arr_ = std::make_unique<double[]>(total_a_con_size);  // SUS
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
    assert(curr_n >= num_obs_gens);
    double* const curr_c_arr = c_arr + c_start_idx;
    double* const curr_zono_arr = zono_arr.get() + zono_start_idx;
    double* const curr_delta_d_arr = delta_d_arr + d_start_idx;

    const auto t_interval = vehrs.zono_time_intervals_.at(zono_idx);
    const double zono_center_x = vehrs.xy_centers_.at(zono_idx * 2);
    const double zono_center_y = vehrs.xy_centers_.at(zono_idx * 2 + 1);
    const double slc_x = vehrs.slc_x_.at(zono_idx);
    const double slc_y = vehrs.slc_y_.at(zono_idx);

    for (IndexT obs_idx = 0; obs_idx < num_obs; ++obs_idx) {
      const IndexT curr_row_out_start =
          (cum_size_arr[zono_idx] * num_obs) + (curr_n * obs_idx);

      // 2 for plus/minus components, 2 for obs dimensionality
      const IndexT pa_start_idx = 2 * 2 * curr_row_out_start;
      const IndexT pb_start_idx = 2 * curr_row_out_start;
      const auto [obs_center_x, obs_center_y] = obs_info.GetCenter(obs_idx, t_interval);
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
        const auto [obs_x, obs_y] = obs_info.GetSingleGenerator(obs_idx, obs_gen_idx, t_interval);
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
          // TODO REMOVE const IndexT obs_gen_start_idx =
          // TODO REMOVE     (obs_idx * num_obs_gens * 2) + obs_gen_idx * 2;
          // TODO REMOVE const double obs_x = obs_arr.at(obs_gen_start_idx + 0);
          // TODO REMOVE const double obs_y = obs_arr.at(obs_gen_start_idx + 1);
          const auto [obs_x, obs_y] = obs_info.GetSingleGenerator(obs_idx, obs_gen_idx, t_interval);
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
        const auto [c_obs_x_unnorm, c_obs_y_unnorm] = obs_info.GetSingleGenerator(obs_idx, c_obs_gen_idx, t_interval);
        const auto [c_obs_x, c_obs_y] = NormPair(c_obs_x_unnorm, c_obs_y_unnorm);
        // TODO REMOVE const auto [c_obs_x, c_obs_y] = NormPair(
        // TODO REMOVE     obs_arr.at(c_obs_gen_start_idx), obs_arr.at(c_obs_gen_start_idx + 1));
        for (IndexT g_obs_gen_idx = 0; g_obs_gen_idx < num_obs_gens;
             ++g_obs_gen_idx) {
          // TODO REMOVE const IndexT g_obs_gen_start_idx =
          // TODO REMOVE     (obs_idx * num_obs_gens * 2) + g_obs_gen_idx * 2;
          const auto [g_obs_x, g_obs_y] = obs_info.GetSingleGenerator(obs_idx, g_obs_gen_idx, t_interval);
          // TODO REMOVE const double g_obs_x = obs_arr.at(g_obs_gen_start_idx + 0);
          // TODO REMOVE const double g_obs_y = obs_arr.at(g_obs_gen_start_idx + 1);
          curr_pb_arr[r] += std::abs(-c_obs_y * g_obs_x + c_obs_x * g_obs_y);
        }
      }

      for (IndexT c_obs_gen_idx = 0; c_obs_gen_idx < num_obs_gens;
           ++c_obs_gen_idx) {
        const auto [c_obs_x_unnorm, c_obs_y_unnorm] = obs_info.GetSingleGenerator(obs_idx, c_obs_gen_idx, t_interval);
        const auto [c_obs_x, c_obs_y] = NormPair(c_obs_x_unnorm, c_obs_y_unnorm);
        const IndexT r = curr_n - (num_obs_gens - c_obs_gen_idx);
        //const IndexT c_obs_gen_start_idx =
        //    (obs_idx * num_obs_gens * 2) + c_obs_gen_idx * 2;
        // TODO REMOVE const auto [c_obs_x, c_obs_y] = NormPair(
        // TODO REMOVE     obs_arr.at(c_obs_gen_start_idx), obs_arr.at(c_obs_gen_start_idx + 1));
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
        /// TODO make sure the offset by curr_n matches with constraint eval
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

/*
void WriteObsTestInfo(const ObsInfo& obs_info, std::string fname) {
  std::ofstream f_out;
  f_out.open(fname, std::ofstream::out);

  f_out << "function [num_obs_in, obs_arr_in] = load_obs_info_latest()\n";
  f_out << "num_obs_in = " << obs_info.num_obs_ << ";\n";
  f_out << "obs_arr_in = {};\n";
  for (std::size_t i = 0; i < obs_info.num_obs_; ++i) {
    f_out << "obs_arr_in{end+1} = [" << obs_info.obs_centers_[2 * i + 0] << ", "
          << obs_info.obs_arr_[4 * i + 0] << ", "
          << obs_info.obs_arr_[4 * i + 2] << ";\n"
          << obs_info.obs_centers_[2 * i + 1] << ", "
          << obs_info.obs_arr_[4 * i + 1] << ", "
          << obs_info.obs_arr_[4 * i + 3] << "];\n";
  }
*/
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
/*
  f_out << "\nend";
  f_out.close();
}
*/

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
