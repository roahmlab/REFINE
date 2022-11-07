#ifndef WAYPOINT_INTERP_PARAMS_
#define WAYPOINT_INTERP_PARAMS_
#include <map>
#include <string>

#include "simple_util.h"
namespace roahm {
struct WaypointInterpParams {
  double max_lookahead_;
  double peak_velocity_;
  double points_per_meter_;
  double low_curvature_lookahead_mult_;
  double high_curvature_lookahead_mult_;
  double low_curvature_heading_plus_;
  double high_curvature_heading_plus_;
  double high_curvature_cutoff_min_;
  double reduce_curve_lookahead_;
  WaypointInterpParams() : WaypointInterpParams(0, 0, 0, 0, 0, 0, 0, 0, 0) {}
  WaypointInterpParams(std::map<std::string, double>& kv)
      : WaypointInterpParams(
            GetWithDefaultWarn(kv, "max_lookahead", 15.0),
            GetWithDefaultWarn(kv, "peak_velocity", 1.5),
            GetWithDefaultWarn(kv, "points_per_meter", 20.0),
            GetWithDefaultWarn(kv, "low_curvature_lookahead_mult", 2.25),
            GetWithDefaultWarn(kv, "high_curvature_lookahead_mult", 1.75),
            GetWithDefaultWarn(kv, "low_curvature_heading_plus", 0.5),
            GetWithDefaultWarn(kv, "high_curvature_heading_plus", 10.8),
            GetWithDefaultWarn(kv, "high_curvature_cutoff_min", 0.20),
            GetWithDefaultWarn(kv, "reduce_curve_lookahead", 7.0)) {}
  WaypointInterpParams(
      double max_lookahead, double peak_velocity, double points_per_meter,
      double low_curvature_lookahead_mult, double high_curvature_lookahead_mult,
      double low_curvature_heading_plus, double high_curvature_heading_plus,
      double high_curvature_cutoff_min, double reduce_curve_lookahead)
      : max_lookahead_{max_lookahead},
        peak_velocity_{peak_velocity},
        points_per_meter_{points_per_meter},
        low_curvature_lookahead_mult_{low_curvature_lookahead_mult},
        high_curvature_lookahead_mult_{high_curvature_lookahead_mult},
        low_curvature_heading_plus_{low_curvature_heading_plus},
        high_curvature_heading_plus_{high_curvature_heading_plus},
        high_curvature_cutoff_min_{high_curvature_cutoff_min},
        reduce_curve_lookahead_{reduce_curve_lookahead} {}
};
WaypointInterpParams GetWaypointInterpParams() {
  const std::string wp_prefix = "/parameter_generation/waypoint_interp_params";
  auto loaded_params =
      GetRosParam<std::map<std::string, double>>(wp_prefix, {});
  return WaypointInterpParams(loaded_params);
}
}  // namespace roahm
#endif  // WAYPOINT_INTERP_PARAMS_
