#include "waypoint_interp_params.hpp"
namespace roahm {

WaypointInterpParams GetWaypointInterpParams() {
  const std::string wp_prefix = "/parameter_generation/waypoint_interp_params";
  auto loaded_params =
      GetRosParam<std::map<std::string, double>>(wp_prefix, {});
  return WaypointInterpParams(loaded_params);
}

}  // namespace roahm