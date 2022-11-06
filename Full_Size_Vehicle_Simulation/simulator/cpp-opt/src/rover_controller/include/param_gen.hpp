#ifndef ROVER_RTD_SU21_PARAM_GEN_HPP
#define ROVER_RTD_SU21_PARAM_GEN_HPP

// Enable extra debugging when not collecting timing/running release mode
#define TIMING_RUNNING false

// These macros are all just for debug/convenience purposes,
// can be removed eventually
#define PRINT_VAR(x) WGUARD_ROS_INFO_STREAM(#x << x);
#define ROS_INFO_STR(x) WGUARD_ROS_INFO_STREAM(x);

#if !TIMING_RUNNING
#define TGUARD(x) x
#else
#define TGUARD(x)
#endif

#include "warning_guards.hpp"  // for WARN_DISABLE_...
#define WGUARD(x) WARN_DISABLE_BEGIN x WARN_DISABLE_END

#define WGUARD_ROS_ERROR(x) WGUARD(ROS_ERROR(x))
#define WGUARD_ROS_INFO(x) WGUARD(ROS_INFO(x))
#define WGUARD_ROS_INFO_STR(x) WGUARD(ROS_INFO_STR(x))
#define WGUARD_ROS_INFO_STREAM(x) WGUARD(ROS_INFO_STREAM(x))

#define TGUARD_ROS_INFO_STREAM(x) TGUARD(WGUARD_ROS_INFO_STREAM(x))
#define TGUARD_ROS_INFO_STR(x) TGUARD(WGUARD_ROS_INFO_STR(x))
#define TGUARD_ROS_INFO(x) TGUARD(WGUARD_ROS_INFO(x))

#include <string>  // for allocator
#include <vector>  // for vector

#include "frs_loader.hpp"              // for LoadFrs, FrsT...
#include "manu_type.hpp"               // for ManuType, Man...
#include "point_xyh.hpp"               // for PointXYH
#include "poly_predictor.hpp"          // for PolyPredictor
#include "rover_state.hpp"             // for RoverState
#include "waypoint_interp_params.hpp"  // for WaypointInter...

WARN_DISABLE_BEGIN
#include <jsk_recognition_msgs/PolygonArray.h>          // for PolygonArray
#include <nav_msgs/Path.h>                              // for Path
#include <rover_control_msgs/GenParamInfo.h>            // for GenParamInfo
#include <rover_control_msgs/MatlabPlotInfo.h>          // for MatlabPlotInfo
#include <rover_control_msgs/OnlineDebugMsg.h>          // for OnlineDebugMsg
#include <rover_control_msgs/RoverDebugStateStamped.h>  // for RoverDebugSta...
#include <std_msgs/Bool.h>                              // for Bool
#include <std_msgs/Float64MultiArray.h>                 // for Float64MultiA...

#include "geometry_msgs/PoseStamped.h"  // for PoseStamped
#include "ros/console.h"                // for ROS_INFO_STREAM
#include "ros/node_handle.h"            // for NodeHandle
#include "ros/publisher.h"              // for Publisher
WARN_DISABLE_END

namespace roahm {

struct ParamEvalResult {
  double sln_k_;
  bool successful_;
  int manu_type_;

  ParamEvalResult() : sln_k_(0), successful_(false) {}
};

struct ParamRet {
  rover_control_msgs::OnlineDebugMsg online_debug_ret_;
  rover_control_msgs::MatlabPlotInfo matlab_plot_info_;
};

ParamRet GenerateParameter(
    const FrsTotal& frs,
    const jsk_recognition_msgs::PolygonArray& latest_read_obs,
    const PointXYH& x_des_normal, const PointXYH& x_des_mirror,
    const PointXYH& x_des_global, RoverState fp_state,
    const RoverState& current_state, bool is_low,
    const nav_msgs::Path& full_path,
    const ros::Publisher& obs_center_publisher_);

class ParameterGenerator {
  struct FpReqSimInfo {  // forward predict request
    bool was_successful_;
    double au_;
    double ay_;
    ManuType manu_type_;
    static constexpr FpReqSimInfo Fail() {
      return {false, 0.0, 0.0, ManuType::kNone};
    }
  };

 public:
  jsk_recognition_msgs::PolygonArray latest_read_obs_;
  bool obs_have_been_appended_;
  RoverState forward_predicted_state_;
  ros::Publisher gen_param_publisher_;
  ros::Publisher obs_publisher_;
  ros::Publisher obs_center_publisher_;
  ros::Publisher online_debug_msg_pub_;
  ros::Publisher matlab_plot_info_pub_;
  ros::Publisher init_pos_pub_;
  rover_control_msgs::GenParamInfo chosen_parameters_;
  PointXYH x_des_;
  FrsTotal frs_;
  FrsTotal frs_low_;
  PolyPredictor poly_predictor_;
  ParamEvalResult latest_param_;
  nav_msgs::Path hlp_path_input_;
  std::vector<PointXYH> hlp_path_xyh_;
  PredictInfo predict_info_;
  WaypointInterpParams waypoint_interp_params_;

 public:
  /// TODO
  /// \param nh TODO
  /// \param frs_fname TODO
  /// \param frs_low_fname TODO
  /// \param poly_predict_fname TODO
  ParameterGenerator(ros::NodeHandle& nh, const std::string& frs_fname,
                     const std::string& frs_low_fname,
                     const std::string& poly_predict_fname)
      : latest_read_obs_{},
        obs_have_been_appended_{false},
        forward_predicted_state_{},
        gen_param_publisher_{nh.advertise<rover_control_msgs::GenParamInfo>(
            "/gen_param_out", 1)},
        obs_publisher_{nh.advertise<jsk_recognition_msgs::PolygonArray>(
            "/global_obstacles", 1)},
        obs_center_publisher_{nh.advertise<std_msgs::Float64MultiArray>(
            "/local_obstacles_center", 1)},
        online_debug_msg_pub_{nh.advertise<rover_control_msgs::OnlineDebugMsg>(
            "/online_debug_msg", 1)},
        matlab_plot_info_pub_{nh.advertise<rover_control_msgs::MatlabPlotInfo>(
            "/matlab_plot_info", 20)},
        init_pos_pub_{nh.advertise<rover_control_msgs::RoverDebugStateStamped>(
            "/init_conditions", 20)},
        chosen_parameters_{},
        x_des_{},
        frs_{LoadFrs(frs_fname)},
        frs_low_{LoadFrs(frs_low_fname)},
        poly_predictor_{poly_predict_fname},
        latest_param_{},
        hlp_path_input_{},
        hlp_path_xyh_{},
        predict_info_{},
        waypoint_interp_params_{} {};

  /// TODO
  /// \return TODO
  bool SuccessfullyLoaded() const {
    return frs_.successful_ and frs_low_.successful_ and
           poly_predictor_.SuccessfullyLoaded();
  };

  /// TODO
  /// \param latest_param TODO
  void PublishPathAndWaypointDebug(
      rover_control_msgs::OnlineDebugMsg latest_param) const;

  FpReqSimInfo ChooseParameters();

  void ForwardPredict() {
    forward_predicted_state_ = poly_predictor_.Predict(predict_info_);
  }

  void ChooseDesiredWaypoint();

  void CallbackHLP(const nav_msgs::Path& hlp_path);
  void CallbackRunSimulation(std_msgs::Bool run);

  //
  // ROS Callbacks
  //

  FpReqSimInfo CallbackFpReqInternal(
      const rover_control_msgs::RoverDebugStateStamped& msg);

  void CallbackFpReq(const rover_control_msgs::RoverDebugStateStamped& msg);

  void CallbackObs(const jsk_recognition_msgs::PolygonArray& msg);

  void RunSimulation();
};
}  // namespace roahm

#endif  // ROVER_RTD_SU21_PARAM_GEN_HPP
