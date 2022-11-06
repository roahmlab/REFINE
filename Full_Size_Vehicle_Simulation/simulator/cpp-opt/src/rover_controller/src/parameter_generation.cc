#include <boost/smart_ptr/make_shared_object.hpp>  // for make_shared
#include <boost/type_index/type_index_facade.hpp>  // for operator==
#include <string>                                  // for string, alloc...

#include "jsk_recognition_msgs/PolygonArray.h"          // for PolygonArray
#include "nav_msgs/Path.h"                              // for Path
#include "param_gen.hpp"                                // for ParameterGene...
#include "ros/init.h"                                   // for init, spin
#include "ros/node_handle.h"                            // for NodeHandle
#include "ros/subscriber.h"                             // for Subscriber
#include "rover_control_msgs/RoverDebugStateStamped.h"  // for RoverDebugSta...
#include "simple_util.hpp"                              // for GetRosParam

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "parameter_generator");
  ros::NodeHandle nh;

  // Load filename from launch file (ROS parameters)
  const std::string no_fname = "NO_FILENAME_LOADED";
  const std::string frs_filename = ::roahm::GetRosParam<std::string>(
      "/parameter_generation/frs_filename", no_fname);
  const std::string frs_low_filename = ::roahm::GetRosParam<std::string>(
      "/parameter_generation/frs_low_filename", no_fname);
  const std::string predict_coeff_filename = ::roahm::GetRosParam<std::string>(
      "/parameter_generation/poly_predict_coeffs_filename", no_fname);

  // Initialize main parameter generation class
  roahm::ParameterGenerator param_generator(nh, frs_filename, frs_low_filename,
                                            predict_coeff_filename);

  // Setup subscribers
  ros::Subscriber sub_hlp_path =
      nh.subscribe("/move_base/GlobalPlanner/plan", 1,
                   &roahm::ParameterGenerator::CallbackHLP, &param_generator);
  ros::Subscriber sub_obs =
      nh.subscribe("/zonotope_visualization", 1,
                   &roahm::ParameterGenerator::CallbackObs, &param_generator);
  ros::Subscriber sub_gen_param =
      nh.subscribe("/fp_req", 1, &roahm::ParameterGenerator::CallbackFpReq,
                   &param_generator);

  // Keep ROS alive
  ros::spin();
}

#undef ROS_INFO_STR
