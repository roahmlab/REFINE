#ifndef ROAHM_GET_WAYPOINT_FROM_PATH_HPP_
#define ROAHM_GET_WAYPOINT_FROM_PATH_HPP_
#include <algorithm>  // for max
#include <array>      // for array
#include <cstddef>    // for size_t
#include <vector>     // for vector

#include "point_xyh.hpp"               // for PointXYH
#include "rover_state.hpp"             // for RoverState
#include "waypoint_interp_params.hpp"  // for WaypointInterpParams

/// @file get_waypoint_from_path.hpp Contains utilities to get a next desired
/// waypoint from the high-level planner (HLP) path.

namespace roahm {
/// TODO
using PathT = std::vector<PointXYH>;

/// TODO
struct DistPoint {
  /// TODO
  double d_;
  /// TODO
  std::vector<double> d_along_;
  /// TODO
  std::size_t p_idx_;
};

/// TODO
/// \param state_x TODO
/// \param state_y TODO
/// \param hlp_path TODO
/// \return TODO
DistPoint DistPointOnPolylineSimon(double state_x, double state_y,
                                   const PathT& hlp_path);

std::array<PointXYH, 3> GetWaypointFromPath(
    const RoverState& state, const PathT& hlp_path,
    const WaypointInterpParams& wp_params);

}  // namespace roahm
#endif
