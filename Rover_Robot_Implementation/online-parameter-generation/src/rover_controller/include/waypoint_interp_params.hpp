#ifndef ROAHM_WAYPOINT_INTERP_PARAMS_HPP_
#define ROAHM_WAYPOINT_INTERP_PARAMS_HPP_
#include <map>
#include <string>

#include "simple_util.hpp"

/// @file waypoint_interp_params.hpp Contains a simple structure and function
/// to load waypoint interpolation parameters from ROS parameters.

namespace roahm {
/// TODO
struct WaypointInterpParams {
  /// Peak velocity to use as baseline for lookahead
  double peak_velocity_;
  /// Waypoints per meter
  double points_per_meter_;
  /// Multiplier on peak velocity to calculate xy lookahead during low curvature
  /// regions
  double low_curvature_lookahead_mult_;
  /// Multiplier on peak velocity to calculate xy lookahead during high
  /// curvature regions
  double high_curvature_lookahead_mult_;
  /// Amount to add to calculated lookahead for heading lookahead during low
  /// curvature regions
  double low_curvature_heading_plus_;
  /// Amount to add to calculated lookahead for heading lookahead during high
  /// curvature regions
  double high_curvature_heading_plus_;
  /// The minimum cutoff to be considered "high" curvature
  /// k > {this value} ==> "high curvature"
  double high_curvature_cutoff_min_;
  /// How many meters to look forward. Multiplied by points_per_meter to get
  /// number of points to look at.
  double reduce_curve_lookahead_;
  /// TODO should we remove?
  /// JL's idea of doubling lookahead sampling points
  double use_JL_idea_;

  /// Default Constructor, sets all parameters to zeros
  WaypointInterpParams()
      : WaypointInterpParams(0, 0, 0, 0, 0, 0, 0, 0, false) {}
  /// Constructor
  /// \param kv a map to populate with parameter names and value pairs, filled
  /// with default values if they are not found via ROS parameter lookup
  explicit WaypointInterpParams(std::map<std::string, double>& kv)
      : WaypointInterpParams(
            GetWithDefaultWarn(kv, "peak_velocity", 1.5),
            GetWithDefaultWarn(kv, "points_per_meter", 20.0),
            GetWithDefaultWarn(kv, "low_curvature_lookahead_mult", 2.25),
            GetWithDefaultWarn(kv, "high_curvature_lookahead_mult", 1.75),
            GetWithDefaultWarn(kv, "low_curvature_heading_plus", 0.5),
            GetWithDefaultWarn(kv, "high_curvature_heading_plus", 10.8),
            GetWithDefaultWarn(kv, "high_curvature_cutoff_min", 0.20),
            GetWithDefaultWarn(kv, "reduce_curve_lookahead", 7.0),
            GetWithDefaultWarn(kv, "use_JL_idea", 0.0)) {}
  /// Constructor.
  /// \param peak_velocity_  Peak velocity to use as baseline for lookahead
  /// \param points_per_meter_  Waypoints per meter
  /// \param low_curvature_lookahead_mult_  Multiplier on peak velocity to
  /// calculate xy lookahead during low curvature regions
  /// \param high_curvature_lookahead_mult_  Multiplier on peak velocity to
  /// calculate xy lookahead during high curvature regions
  /// \param low_curvature_heading_plus_  Amount to add to calculated lookahead
  /// for heading lookahead during low curvature regions
  /// \param high_curvature_heading_plus_  Amount to add to calculated lookahead
  /// for heading lookahead during high curvature regions
  /// \param high_curvature_cutoff_min_  The minimum cutoff to be considered
  /// "high" curvature
  /// \param reduce_curve_lookahead_ How many meters to look forward. Multiplied
  /// by points_per_meter to get number of points to look at.
  /// \param use_JL_idea_  JL's idea of doubling lookahead sampling points
  WaypointInterpParams(double peak_velocity, double points_per_meter,
                       double low_curvature_lookahead_mult,
                       double high_curvature_lookahead_mult,
                       double low_curvature_heading_plus,
                       double high_curvature_heading_plus,
                       double high_curvature_cutoff_min,
                       double reduce_curve_lookahead, double use_JL_idea)
      : peak_velocity_{peak_velocity},
        points_per_meter_{points_per_meter},
        low_curvature_lookahead_mult_{low_curvature_lookahead_mult},
        high_curvature_lookahead_mult_{high_curvature_lookahead_mult},
        low_curvature_heading_plus_{low_curvature_heading_plus},
        high_curvature_heading_plus_{high_curvature_heading_plus},
        high_curvature_cutoff_min_{high_curvature_cutoff_min},
        reduce_curve_lookahead_{reduce_curve_lookahead},
        use_JL_idea_{use_JL_idea} {}
};
/// Loads waypoint interpolation parameters from ROS parameters
/// \return the loaded waypoint interpolation parameters
WaypointInterpParams GetWaypointInterpParams();
}  // namespace roahm
#endif  // ROAHM_WAYPOINT_INTERP_PARAMS_HPP_
