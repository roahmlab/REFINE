// Enable extra debugging when not collecting timing/running release mode
#define TIMING_RUNNING false

// These macros are all just for debug/convenience purposes,
// can be removed eventually
#define PRINT_VAR(x) ROS_INFO_STREAM(#x << x);
#define ROS_INFO_STR(x) ROS_INFO_STREAM(x);

#if !TIMING_RUNNING
#define TGUARD(x) x
#else
#define TGUARD(x)
#endif

#define TGUARD_ROS_INFO_STREAM(x) TGUARD(ROS_INFO_STREAM(x))
#define TGUARD_ROS_INFO_STR(x) TGUARD(ROS_INFO_STR(x))
#define TGUARD_ROS_INFO(x) TGUARD(ROS_INFO(x))

#include <fmt/format.h>                                 // for format
#include <jsk_recognition_msgs/PolygonArray.h>          // for PolygonArray
#include <math.h>                                       // for cos, sin
#include <nav_msgs/Path.h>                              // for Path, Path_<>...
#include <ros/time.h>                                   // for Time
#include <rover_control_msgs/GenParamInfo.h>            // for GenParamInfo
#include <rover_control_msgs/MatlabPlotInfo.h>          // for MatlabPlotInfo
#include <rover_control_msgs/OnlineDebugMsg.h>          // for OnlineDebugMsg
#include <rover_control_msgs/RoverDebugStateStamped.h>  // for RoverDebugSta...
#include <std_msgs/Bool.h>                              // for Bool
#include <std_msgs/Header.h>                            // for Header_, Head...
#include <stdint.h>                                     // for uint8_t

#include <algorithm>                               // for min_element, max
#include <array>                                   // for array
#include <boost/smart_ptr/make_shared_object.hpp>  // for make_shared
#include <boost/type_index/type_index_facade.hpp>  // for operator==
#include <chrono>                                  // for time_point
#include <ext/alloc_traits.h>                      // for __alloc_trait...
#include <iostream>                                // for operator<<
#include <iterator>                                // for distance
#include <memory>                                  // for allocator
#include <string>                                  // for string, char_...
#include <vector>                                  // for vector, vecto...

#include "IpIpoptApplication.hpp"       // for IpoptApplication
#include "IpReturnCodes_inc.h"          // for Solve_Succeeded
#include "IpSmartPtr.hpp"               // for GetRawPtr
#include "IpSolveStatistics.hpp"        // for SolveStatistics
#include "IpTNLP.hpp"                   // for TNLP
#include "IpTypes.hpp"                  // for Number, Index
#include "MyNLP.hpp"                    // for MyNLP
#include "bench_helper.h"               // for Tick, GetDeltaS
#include "common.h"                     // for IndexT
#include "complex_heading_state.h"      // for ComplexHeadin...
#include "frs_loader.h"                 // for FrsSelectInfo
#include "gencon.h"                     // for Constraints
#include "gencon_samples.h"             // for Obs11_polarr
#include "geometry_msgs/Point.h"        // for Point_
#include "geometry_msgs/Pose.h"         // for Pose_
#include "geometry_msgs/PoseStamped.h"  // for PoseStamped
#include "get_waypoint_from_path.h"     // for GetWaypointFr...
#include "online_utils.h"               // for PrintMMMSSecT...
#include "point_xyh.h"                  // for PointXYH
#include "poly_predictor.h"             // for PredictInfo
#include "rmanu_type.h"                 // for ManuToUint8
#include "ros/console.h"                // for LogLocation
#include "ros/init.h"                   // for init, spin
#include "ros/node_handle.h"            // for NodeHandle
#include "ros/publisher.h"              // for Publisher
#include "ros/subscriber.h"             // for Subscriber
#include "simple_util.h"                // for GetRosParam
#include "waypoint_interp_params.h"     // for GetWaypointIn...

namespace roahm {

struct ParamEvalResult {
  double sln_k_;
  bool successful_;
  int manu_type_;

  ParamEvalResult() : sln_k_(0), successful_(false) {}
};

struct BinTimes {
  std::vector<double> eval_f_times_;
  std::vector<double> eval_grad_f_times_;
  std::vector<double> eval_g_times_;
  std::vector<double> eval_jac_g_times_;
  std::vector<double> eval_h_times_;
  double eval_jac_g_init_time_;
  double eval_h_init_time_;
  double constraint_gen_time_;
  double loop_time_;
  double ipopt_solve_time_;
  double obs_proc_time_;
  long num_ipopt_iters_;
  long num_constraints_;
  long num_ab_rows_;
  bool successful_;
};

struct ParamRet {
  rover_control_msgs::OnlineDebugMsg online_debug_ret_;
  rover_control_msgs::MatlabPlotInfo matlab_plot_info_;
};

ParamRet GenerateParameter(
    const FrsTotal& frs,
    const jsk_recognition_msgs::PolygonArray& latest_read_obs,
    const PointXYH& x_des_normal, const PointXYH& x_des_mirror,
    const PointXYH& x_des_global, const double state_x, const double state_y,
    const double state_h, double state_u, double state_v, double state_r,
    const ComplexHeadingState& current_state, const bool is_low,
    const nav_msgs::Path& full_path) {
  // DisplayObs(latest_read_obs);
  const auto gen_param_start_time = Tick();
  ROS_INFO_STREAM("IS_LOW: " << is_low);
  const auto frses_to_search =
      GetParamsToSearchOver(frs, state_u, state_v, state_r);

  // Store a copy of v, r inverted for mirror
  const double mirror_v = -state_v;
  const double mirror_r = -state_r;
  const int iters_to_run = frses_to_search.size();
  const auto tick_preloop = Tick();
  int successful_runs = 0;
  int failure_runs = 0;
  const int num_search = frses_to_search.size();
  const int num_apps = 6;

  ROS_INFO("APP INIT");
  // Initialize and set options on IPOPT
  const auto app_vec = GetIpApps(num_apps);

  // Number of outer loops = ceil(num_search / num_apps)
  const int num_outer = (num_search + (num_apps - 1)) / num_apps;

  const int num_opt_problems = num_search;

  std::vector<double> cost_vals(num_opt_problems);
  std::vector<double> param_vals_mirrored(num_opt_problems);

  // DO NOT USE vector<bool>: that implementation is **NOT** thread safe
  std::vector<int> success_vals(num_opt_problems);
  std::vector<BinTimes> bin_times(num_opt_problems);

  for (int outer_idx = 0; outer_idx < num_outer; ++outer_idx) {
    // Run each ipopt application in parallel
#pragma omp parallel for
    for (int app_idx = 0; app_idx < num_apps; ++app_idx) {
      const auto loop_start_tick = Tick();
      Ipopt::ApplicationReturnStatus status;
      const int search_idx_pre = outer_idx * num_apps + app_idx;
      // Can't break early with parallel-for, so repeat a few problems if
      // necessary
      const int search_idx = (search_idx_pre + 1) >= num_search
                                 ? (num_search - 1)
                                 : search_idx_pre;

      if (search_idx != search_idx_pre) {
        continue;
      }
      auto& app = app_vec.at(app_idx);
      status = app->Initialize();
      if (status != Ipopt::Solve_Succeeded) {
        ROS_ERROR("\n\n*** Error during IPOPT initialization!");
      }
      const auto& frs_select_info = frses_to_search.at(search_idx);
      const bool mirror = frs_select_info.mirror_;
      const double mirror_mult = mirror ? -1.0 : 1.0;
      const auto& frs_to_use = frs.GetVehrs(frs_select_info);
      TGUARD_ROS_INFO_STREAM("\n\nRUNNING NEW:\n\n");
      TGUARD_ROS_INFO_STREAM(
          "ManuType: " << ToString(frs_select_info.manu_type_));
      TGUARD_ROS_INFO_STREAM("Mirror:   " << mirror);
      TGUARD_ROS_INFO_STREAM("idxu0: " << frs_select_info.idxu0_);
      TGUARD_ROS_INFO_STREAM("idx0:  " << frs_select_info.idx0_);
      TGUARD_ROS_INFO_STREAM("idx1:  " << frs_select_info.idx1_);

      // Load these in depending on whether mirroring or not
      TGUARD(const auto pre_obs_proc_tick = Tick());
      const auto obs_info_to_use = ConvertObsToZonoWithHeading(
          frs_to_use, latest_read_obs, state_x, state_y, state_h, mirror);
      TGUARD(const auto post_obs_proc_tick = Tick());
      PRINT_VAR(obs_info_to_use.num_obs_)
      const auto& x_des_to_use = mirror ? x_des_mirror : x_des_normal;
      const auto v_to_use = mirror ? mirror_v : state_v;
      const auto r_to_use = mirror ? mirror_r : state_r;

      const IndexT zono_desired_idx = frs_to_use.t_eval_idx_;
      const auto cost_fcn_info =
          GenCostFcn(frs_to_use, zono_desired_idx, state_u, v_to_use, r_to_use,
                     x_des_to_use.x_, x_des_to_use.y_, x_des_to_use.h_);
      const auto pre_cons_gen_tick = Tick();
      const auto cons = GenerateConstraints(frs_to_use, obs_info_to_use,
                                            state_u, v_to_use, r_to_use);
      // WriteConstraints(cons);
      const auto post_cons_gen_tick = Tick();

      const int num_ab_rows = cons.num_b_elts_;
      std::shared_ptr<Ipopt::Number[]> a_mat = cons.a_con_arr_;
      std::shared_ptr<Ipopt::Number[]> b_mat = cons.b_con_arr_;

      const auto tick_prenlp = Tick();
      const double k_rng = frs_to_use.k_rng_;

      // Create an instance of your nlp...
      SmartPtr<TNLP> mynlp =
          new MyNLP(num_ab_rows, a_mat, b_mat, cost_fcn_info,
                    cons.zono_startpoints_, cons.zono_obs_sizes_, k_rng);

      const auto tick_postnlp = Tick();
      status = app->OptimizeTNLP(mynlp);
      const auto tick_postsolve = Tick();
      const int total_idx = outer_idx * num_apps + app_idx;
      const Index iter_count = app->Statistics()->IterationCount();

#define PRINT_IPOPT_RET_CODE(x) ROS_INFO_STREAM("IPOPT RETURN CODE: " << x)
      switch (status) {
        case Ipopt::Solve_Succeeded:
          PRINT_IPOPT_RET_CODE("Solve_Succeeded");
          break;
        case Ipopt::Solved_To_Acceptable_Level:
          PRINT_IPOPT_RET_CODE("Solved_To_Acceptable_Level");
          break;
        case Ipopt::Feasible_Point_Found:
          PRINT_IPOPT_RET_CODE("Feasible_Point_Found");
          break;
        case Ipopt::Infeasible_Problem_Detected:
          PRINT_IPOPT_RET_CODE("Infeasible_Problem_Detected");
          break;
        case Ipopt::Search_Direction_Becomes_Too_Small:
          PRINT_IPOPT_RET_CODE("Search_Direction_Becomes_Too_Small");
          break;
        case Ipopt::Diverging_Iterates:
          PRINT_IPOPT_RET_CODE("Diverging_Iterates");
          break;
        case Ipopt::User_Requested_Stop:
          PRINT_IPOPT_RET_CODE("User_Requested_Stop");
          break;
        case Ipopt::Maximum_Iterations_Exceeded:
          PRINT_IPOPT_RET_CODE("Maximum_Iterations_Exceeded");
          break;
        case Ipopt::Maximum_WallTime_Exceeded:
          PRINT_IPOPT_RET_CODE("Maximum_WallTime_Exceeded");
          break;
        case Ipopt::Maximum_CpuTime_Exceeded:
          PRINT_IPOPT_RET_CODE("Maximum_CpuTime_Exceeded");
          break;
        case Ipopt::Restoration_Failed:
          PRINT_IPOPT_RET_CODE("Restoration_Failed");
          break;
        case Ipopt::Error_In_Step_Computation:
          PRINT_IPOPT_RET_CODE("Error_In_Step_Computation");
          break;
        case Ipopt::Invalid_Option:
          PRINT_IPOPT_RET_CODE("Invalid_Option");
          break;
        case Ipopt::Not_Enough_Degrees_Of_Freedom:
          PRINT_IPOPT_RET_CODE("Not_Enough_Degrees_Of_Freedom");
          break;
        case Ipopt::Invalid_Problem_Definition:
          PRINT_IPOPT_RET_CODE("Invalid_Problem_Definition");
          break;
        case Ipopt::Unrecoverable_Exception:
          PRINT_IPOPT_RET_CODE("Unrecoverable_Exception");
          break;
        case Ipopt::NonIpopt_Exception_Thrown:
          PRINT_IPOPT_RET_CODE("NonIpopt_Exception_Thrown");
          break;
        case Ipopt::Insufficient_Memory:
          PRINT_IPOPT_RET_CODE("Insufficient_Memory");
          break;
        case Ipopt::Internal_Error:
          PRINT_IPOPT_RET_CODE("Internal_Error");
          break;
        default:
          PRINT_IPOPT_RET_CODE("UNKNOWN IPOPT RETURN CODE?");
          break;
      }

      if (status == Ipopt::Solve_Succeeded or
          status == Ipopt::Solved_To_Acceptable_Level) {
        // Retrieve some statistics about the solve
        ++successful_runs;
        const Number final_obj = app->Statistics()->FinalObjective();
        const auto sln_k_ = static_cast<MyNLP*>(GetRawPtr(mynlp))->sln_k_;
        success_vals.at(total_idx) = true;
        cost_vals.at(total_idx) = final_obj;
        param_vals_mirrored.at(total_idx) = mirror_mult * sln_k_;

        // TODO REVERT
        const std::string output_str = fmt::format(
            "IPOPT: SUCCESS\n"
            "  Found feasible parameter in {} iterations.\n"
            "  Final Cost: {:.2f}",
            iter_count, final_obj);
        TGUARD_ROS_INFO_STR(output_str);
      } else {
        const auto* mnlp = static_cast<MyNLP*>(GetRawPtr(mynlp));
        const bool was_really_feasible = mnlp->FoundFeasible();
        ROS_INFO_STREAM("IPOPT failed to find feasible parameter in "
                        << iter_count << " iterations."
                        << " [really: " << was_really_feasible << "]");
        if (was_really_feasible) {
          const double cost_val = mnlp->GetFeasibleCost();
          const double param_val = mnlp->GetFeasibleParam();
          ROS_ERROR_STREAM("IPOPT LIED, FEASIBLE SOLUTION FOUND [K: "
                           << param_val << "] [Cost: " << cost_val << "]");
          cost_vals.at(total_idx) = cost_val;
          success_vals.at(total_idx) = true;
          param_vals_mirrored.at(total_idx) = mirror_mult * param_val;
          ++successful_runs;
        } else {
          cost_vals.at(total_idx) = 1.0e+10;
          success_vals.at(total_idx) = false;
          param_vals_mirrored.at(total_idx) = 0.0;
          ++failure_runs;
        }
      }
#if !TIMING_RUNNING
      // Store some solve statistics.
      MyNLP* mnlp = static_cast<MyNLP*>(GetRawPtr(mynlp));
      BinTimes bin_time;
      bin_time.eval_f_times_ = mnlp->total_eval_f_times_;
      bin_time.eval_grad_f_times_ = mnlp->total_eval_grad_f_times_;
      bin_time.eval_g_times_ = mnlp->total_eval_g_times_;
      bin_time.eval_jac_g_times_ = mnlp->total_eval_jac_g_times_;
      bin_time.eval_h_times_ = mnlp->total_eval_h_times_;
      bin_time.eval_jac_g_init_time_ = mnlp->total_eval_jac_g_init_time_;
      bin_time.eval_h_init_time_ = mnlp->total_eval_h_init_time_;
      bin_time.constraint_gen_time_ =
          GetDeltaS(post_cons_gen_tick, pre_cons_gen_tick);
      bin_time.obs_proc_time_ =
          GetDeltaS(post_obs_proc_tick, pre_obs_proc_tick);
      bin_time.loop_time_ = GetDeltaS(tick_postsolve, loop_start_tick);  // TODO
      bin_time.num_ipopt_iters_ = iter_count;
      bin_time.num_constraints_ = cons.zono_startpoints_.size();
      bin_time.num_ab_rows_ = num_ab_rows;
      bin_time.successful_ = success_vals.at(total_idx);
      bin_time.ipopt_solve_time_ = GetDeltaS(tick_postsolve, tick_postnlp);
      bin_times.at(total_idx) = bin_time;
      std::string f_timing_str =
          fmt::format("\n{:16s} {:10s}  {:15s} {:15s} {:15s} {:15s}\n", "Name",
                      "Runs", "Mean", "Min", "Max", "Sum") +
          PrintMMMSSecToMicro(bin_time.eval_f_times_, "eval_f     ") +
          PrintMMMSSecToMicro(bin_time.eval_grad_f_times_, "eval_grad_f") +
          PrintMMMSSecToMicro(bin_time.eval_g_times_, "eval_g     ") +
          PrintMMMSSecToMicro(bin_time.eval_jac_g_times_, "eval_jac_g_") +
          PrintMMMSSecToMicro(bin_time.eval_h_times_, "eval_h     ");
      ROS_INFO_STR(f_timing_str);
      const std::string timing_str = fmt::format(
          "IPOPT Iters:        {}\n"
          "Success:            {}\n"
          "Num Rows:           {}\n"
          "Num Constraints:    {}\n"
          "IPOPT Time:         {}s\n"
          "Gencon Time:        {}s\n"
          "OBS Process Time:   {}s\n",
          bin_time.num_ipopt_iters_, bin_time.successful_,
          bin_time.num_ab_rows_, bin_time.num_constraints_,
          bin_time.ipopt_solve_time_, bin_time.constraint_gen_time_,
          bin_time.obs_proc_time_);
      ROS_INFO_STR(timing_str);
#endif
    }
  }
  const auto gen_param_end_time = Tick();
#if !TIMING_RUNNING
  ROS_INFO_STREAM("Successes: " << successful_runs);
  ROS_INFO_STREAM("Failures:  " << failure_runs);
  ROS_INFO_STREAM("TOTAL GEN PARAM TIME: "
                  << GetDeltaS(gen_param_end_time, gen_param_start_time)
                  << "s");
  ROS_INFO_STREAM("TOTAL LOOP TIME: "
                  << GetDeltaS(gen_param_end_time, tick_preloop) << "s");
  ROS_INFO_STREAM("NUMBER OF VEHRS's LOOKED AT: " << frses_to_search.size());
  ROS_INFO_STREAM("PER LOOP TIME: "
                  << GetDeltaS(gen_param_end_time, gen_param_start_time) /
                         static_cast<double>(iters_to_run)
                  << "s");
#endif
  rover_control_msgs::OnlineDebugMsg ret_val;
  rover_control_msgs::MatlabPlotInfo mat_ret;
  mat_ret.was_successful = false;

  if (frses_to_search.size() < 1) {
    return {ret_val, mat_ret};
  }
  const auto min_cost_it = std::min_element(cost_vals.begin(), cost_vals.end());
  auto min_cost_idx = std::distance(cost_vals.begin(), min_cost_it);
  if (min_cost_idx > num_search) {
    std::cerr << "\n\n\nMIN COST OUT OF SEARCH BOUNDS\n\n\n";
    min_cost_idx = num_search - 1;
  }
  const auto frs_min_info = frses_to_search.at(min_cost_idx);
  const int t0_idx_c_style = frs_min_info.idx1_;
  const bool was_successful = successful_runs > 0;
  const auto final_manu_type = frs_min_info.manu_type_;
  const double final_param = param_vals_mirrored.at(min_cost_idx);
  const bool final_param_is_au = IsSpd(final_manu_type);
  const double au_out = final_param_is_au ? final_param : state_u;
  const double ay_out = (not final_param_is_au) ? final_param : 0.0;

  ret_val.manu_type = ToString(final_manu_type);
  ret_val.idxu0 = frs_min_info.idxu0_;
  ret_val.idx0 = frs_min_info.idx0_;  // Corresponds to the param bin index
  ret_val.idx1 = t0_idx_c_style;      // Corresponds to the t0 index
  ret_val.mirror = frs_min_info.mirror_;
  ret_val.final_cost = cost_vals.at(min_cost_idx);
  ret_val.final_param = param_vals_mirrored.at(min_cost_idx);
  ret_val.time_to_compute_s =
      GetDeltaS(gen_param_end_time, gen_param_start_time);
  ret_val.num_successful = successful_runs;
  ret_val.num_failed = failure_runs;
  ret_val.x0 = state_x;
  ret_val.y0 = state_y;
  ret_val.h0 = state_h;
  ret_val.u0 = state_u;
  ret_val.w0 = state_u;  // TODO
  ret_val.v0 = state_v;
  ret_val.r0 = state_r;
  ret_val.r_err_sum0 = 0.0;  // TODO
  ret_val.h_err_sum0 = 0.0;  // TODO
  ret_val.x_des_local_x = x_des_normal.x_;
  ret_val.x_des_local_y = x_des_normal.y_;
  ret_val.x_des_local_h = x_des_normal.h_;
  ret_val.opt_search_successes = {};
  ret_val.opt_search_costs = {};
  ret_val.opt_search_parameters = {};
  ret_val.opt_search_manu_types = {};
  ret_val.opt_search_u0_idxs = {};
  ret_val.opt_search_traj_idx0s = {};
  ret_val.opt_search_traj_idx1s = {};
  ret_val.opt_search_should_mirror = {};
  ret_val.times = {};  // TODO

  for (auto success_val : success_vals) {
    ret_val.opt_search_successes.push_back(success_val);
  }
  for (auto cost_val : cost_vals) {
    ret_val.opt_search_costs.push_back(cost_val);
  }
  for (auto k_val : param_vals_mirrored) {
    ret_val.opt_search_parameters.push_back(k_val);
  }
  for (auto frs_info : frses_to_search) {
    ret_val.opt_search_manu_types.push_back(ToString(frs_info.manu_type_));
    ret_val.opt_search_u0_idxs.push_back(frs_info.idxu0_);
    ret_val.opt_search_traj_idx0s.push_back(frs_info.idx0_);
    ret_val.opt_search_traj_idx1s.push_back(frs_info.idx1_);
    ret_val.opt_search_should_mirror.push_back(frs_info.mirror_);
  }

  const int u0_idx_1_indexed = frs_min_info.idxu0_ + 1;
  const int idx0_1_indexed = frs_min_info.idx0_ + 1;
  const int idx1_1_indexed = t0_idx_c_style + 1;
  const std::uint8_t manu_type_uint8_char = ManuToUint8(final_manu_type);

  mat_ret.was_successful = was_successful;
  mat_ret.is_low = is_low;
  // mat_ret.full_rob_path = full_path;
  // Current state at time of previous trajectory start
  mat_ret.x0 = current_state.x_;
  mat_ret.y0 = current_state.y_;
  mat_ret.h0 = current_state.GetHeading();
  mat_ret.u0 = current_state.u_;
  mat_ret.v0 = current_state.v_;
  mat_ret.r0 = current_state.r_;
  // The predicted starting state of our trajectory
  mat_ret.pred_x = state_x;
  mat_ret.pred_y = state_y;
  mat_ret.pred_h = state_h;
  mat_ret.pred_u = state_u;
  mat_ret.pred_v = state_v;
  mat_ret.pred_r = state_r;
  // Waypoint
  mat_ret.wp_x = x_des_global.x_;
  mat_ret.wp_y = x_des_global.y_;
  mat_ret.wp_h = x_des_global.h_;
  // Chosen Values
  mat_ret.u0_idx = u0_idx_1_indexed;
  mat_ret.idx0 = idx0_1_indexed;
  mat_ret.idx1 = idx1_1_indexed;
  mat_ret.manu_type = manu_type_uint8_char;
  mat_ret.k_param = final_param;

  // Obstacles
  mat_ret.obstacles = latest_read_obs;

  return {ret_val, mat_ret};
}

class ParameterGenerator {
  struct FpReqSimInfo {
    bool was_successful_;
    double au_;
    double ay_;
    RManuType manu_type_;
    static constexpr FpReqSimInfo Fail() {
      return {false, 0.0, 0.0, RManuType::kNone};
    }
  };

 public:
  jsk_recognition_msgs::PolygonArray latest_read_obs_;
  bool obs_have_been_appended_;
  ComplexHeadingState forward_predicted_state_;
  ros::Publisher gen_param_publisher_;
  ros::Publisher obs_publisher_;
  ros::Publisher online_debug_msg_pub_;
  ros::Publisher matlab_plot_info_pub_;
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
  ParameterGenerator(ros::NodeHandle& nh, std::string frs_fname,
                     std::string frs_low_fname, std::string poly_predict_fname)
      : latest_read_obs_{},
        obs_have_been_appended_{false},
        forward_predicted_state_{},
        gen_param_publisher_{nh.advertise<rover_control_msgs::GenParamInfo>(
            "/gen_param_out", 1)},
        obs_publisher_{
            nh.advertise<jsk_recognition_msgs::PolygonArray>("/obs_out", 1)},
        online_debug_msg_pub_{nh.advertise<rover_control_msgs::OnlineDebugMsg>(
            "/online_debug_msg", 1)},
        matlab_plot_info_pub_{nh.advertise<rover_control_msgs::MatlabPlotInfo>(
            "/matlab_plot_info", 20)},
        chosen_parameters_{},
        x_des_{},
        frs_{LoadFrs(frs_fname)},
        frs_low_{LoadFrs(frs_low_fname)},
        poly_predictor_{poly_predict_fname},
        latest_param_{},
        hlp_path_input_{},
        hlp_path_xyh_{},
        predict_info_{},
        waypoint_interp_params_{} {}

  bool SuccessfullyLoaded() const {
    return frs_.successful_ and frs_low_.successful_ and
           poly_predictor_.SuccessfullyLoaded();
  };

  void PublishPathAndWaypointDebug(
      rover_control_msgs::OnlineDebugMsg latest_param) const {
    latest_param.x_des_global_x = x_des_.x_;
    latest_param.x_des_global_y = x_des_.y_;
    latest_param.x_des_global_h = x_des_.h_;

    latest_param.path_x = {};
    latest_param.path_y = {};
    latest_param.path_h = {};
    for (const auto& pxyh : hlp_path_xyh_) {
      latest_param.path_x.push_back(pxyh.x_);
      latest_param.path_y.push_back(pxyh.y_);
      latest_param.path_h.push_back(pxyh.h_);
    }
    online_debug_msg_pub_.publish(latest_param);
  }

  FpReqSimInfo ChooseParameters() {
    if (!SuccessfullyLoaded()) {
      ROS_ERROR("Not choosing parameters. FRS was not successfully loaded.\n");
      return FpReqSimInfo::Fail();
    }
    ROS_INFO("Choosing Parameters...");
    // forward_predicted_state_.u_ = 0.460655;

    const double state_x = forward_predicted_state_.x_;
    const double state_y = forward_predicted_state_.y_;
    const double state_h = forward_predicted_state_.GetHeading();
    const double state_u = forward_predicted_state_.u_;
    const double state_v = forward_predicted_state_.v_;
    const double state_r = forward_predicted_state_.r_;
    ROS_INFO_STREAM("FPS: " << forward_predicted_state_.ToStringXYHUVR());
    ChooseDesiredWaypoint();

    const auto x_des_normal_ =
        WorldToLocalDesired(x_des_, state_x, state_y, state_h, false);
    const auto x_des_mirror_ =
        WorldToLocalDesired(x_des_, state_x, state_y, state_h, true);
    TGUARD(ROS_INFO_STREAM("Generating parameter..."));
    const bool is_low = state_u < frs_.MinU0();
    ROS_INFO_STREAM("IS_LOW: " << is_low << " (speed: " << state_u
                               << ", max: " << frs_.MinU0() << ")");
    const auto& frs_file_to_use = is_low ? frs_low_ : frs_;
    const ComplexHeadingState current_state{
        predict_info_.x0_, predict_info_.y0_, predict_info_.h0_,
        predict_info_.u0_, predict_info_.v0_, predict_info_.r0_,
        predict_info_.u0_};
    const auto t0gp = Tick();
    const auto [latest_param, plot_info_param] = GenerateParameter(
        frs_file_to_use, latest_read_obs_, x_des_normal_, x_des_mirror_, x_des_,
        state_x, state_y, state_h, state_u, state_v, state_r, current_state,
        is_low, hlp_path_input_);
    const auto t1gp = Tick();
    ROS_INFO_STREAM("GP Took: " << GetDeltaS(t1gp, t0gp) << "s");
    PublishPathAndWaypointDebug(latest_param);
    matlab_plot_info_pub_.publish(plot_info_param);

    if (latest_param.num_successful > 0) {
      const std::string manu_str = latest_param.manu_type;
      const double final_param = latest_param.final_param;
      ROS_INFO_STREAM("PUBLISHING PARAMETER " << final_param << "(" << manu_str
                                              << ")");
      const RManuType manu_type = FromString(manu_str);
      const double au = GetGenParamAu(manu_type, state_u, final_param);
      const double ay = GetGenParamAy(manu_type, state_u, final_param);
      const int t0_idx = latest_param.idx1;
      const double t0_offset = t0_idx * 1.5;
      rover_control_msgs::GenParamInfo gen_param_out_msg;
      gen_param_out_msg.au = au;
      gen_param_out_msg.ay = ay;
      gen_param_out_msg.t0_offset = t0_offset;
      gen_param_out_msg.header.stamp = ros::Time::now();
      // TODO make sure this meshes with controller okay
      gen_param_out_msg.manu_type = ManuToUint8(manu_type);
      gen_param_publisher_.publish(gen_param_out_msg);
      return {true, au, ay, manu_type};
    } else {
      ROS_INFO("NOT PUBLISHING PARAMETER");
      ROS_INFO_STREAM("Successes: " << latest_param.num_successful);
      ROS_INFO_STREAM("Failures:  " << latest_param.num_failed);
    }
    return FpReqSimInfo::Fail();
  }

  void ForwardPredict() {
    forward_predicted_state_ = poly_predictor_.Predict(predict_info_);
  }

  void ChooseDesiredWaypoint() {
    ROS_INFO("ChooseDesiredWaypoint");
    if (hlp_path_input_.poses.size() > 0) {
      ROS_INFO("Nonempty HLP Path");
      TGUARD_ROS_INFO(
          "Using hlp_path_input_ from ROS to generate desired waypoint.");
      hlp_path_xyh_.clear();
      for (const geometry_msgs::PoseStamped& pose_stamp :
           hlp_path_input_.poses) {
        const auto& pos = pose_stamp.pose.position;
        const double yaw = GetOrientationYaw(pose_stamp.pose.orientation);
        const double x = pos.x;
        const double y = pos.y;
        hlp_path_xyh_.emplace_back(x, y, yaw);
      }
      waypoint_interp_params_ = GetWaypointInterpParams();
      auto x_desireds = GetWaypointFromPath(
          forward_predicted_state_, hlp_path_xyh_, waypoint_interp_params_);
      x_des_ = x_desireds.back();
      ROS_INFO("ChooseDesiredWaypoint COMPLETE");
    } else {
      ROS_ERROR("NO PATH PROVIDED");
      const double lookahead_val = 1.0;
      const double h = forward_predicted_state_.GetHeading();
      const double x = forward_predicted_state_.x_;
      const double y = forward_predicted_state_.y_;
      x_des_ = {x + std::cos(h), y + std::sin(h), h};
    }
  }

  void CallbackHLP(const nav_msgs::Path& hlp_path) {
    ROS_INFO("Callback HLP");
    hlp_path_input_ = hlp_path;
  }
  void CallbackRunSimulation(const std_msgs::Bool run) {
    ROS_INFO("CallbackRunSimulation\n");
    if (run.data) {
      RunSimulation();
    }
  }

  //
  // ROS Callbacks
  //

  FpReqSimInfo CallbackFpReqInternal(
      const rover_control_msgs::RoverDebugStateStamped& msg) {
    ROS_INFO("Callback state");

    // Start timing
    const auto t0 = Tick();
    predict_info_ = PredictInfoFromControllerMessage(msg);

    // Generate predicted state and then trajectory parameters
    ForwardPredict();
    auto ret = ChooseParameters();

    // Print time
    const auto t1 = Tick();
    ROS_INFO_STREAM("Callback FP Took: " << GetDeltaS(t1, t0) << "s");
    return ret;
  }
  void CallbackFpReq(const rover_control_msgs::RoverDebugStateStamped& msg) {
    CallbackFpReqInternal(msg);
  }

  void CallbackObs(const jsk_recognition_msgs::PolygonArray& msg) {
    ROS_INFO("Callback obs");
    obs_have_been_appended_ = false;
    latest_read_obs_ = msg;
    ROS_INFO_STREAM("NUM OBS: " << latest_read_obs_.polygons.size());
  }

  void RunSimulation() {
    ROS_INFO("RunSimulation\n");
    // CallbackFpReq (false running auto, 0 state)
    rover_control_msgs::RoverDebugStateStamped init_msg{};
    init_msg.x = 0.0;
    init_msg.y = 0.0;
    init_msg.h = 0.0;
    init_msg.u = 1.0;
    bool is_stuck = false;
    for (int i = 0; (i < 16) and (not is_stuck); ++i) {
      ROS_INFO_STREAM("SIM ITERATION BEG: " << i);
      const auto [found_param, au, ay, manu_type] =
          CallbackFpReqInternal(init_msg);
      is_stuck = not found_param;
      init_msg.running_auto = found_param;
      init_msg.au = au;
      init_msg.ay = ay;
      init_msg.manu_type = ManuToUint8(manu_type);
      init_msg.x = forward_predicted_state_.x_;
      init_msg.y = forward_predicted_state_.y_;
      init_msg.h = forward_predicted_state_.GetHeading();
      init_msg.u = forward_predicted_state_.u_;
      init_msg.v = forward_predicted_state_.v_;
      init_msg.r = forward_predicted_state_.r_;
      ROS_INFO_STREAM("SIM ITERATION END: " << i << " [stuck: " << is_stuck
                                            << "]");
    }
  }
};
}  // namespace roahm
int main(int argc, char** argv) {
  //::roahm::DisplayObs(::roahm::Obs12_polarr());
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
  ros::Subscriber sub_run_sim = nh.subscribe(
      "/run_online_sim", 1, &roahm::ParameterGenerator::CallbackRunSimulation,
      &param_generator);
  auto obs_publisher_ = nh.advertise<jsk_recognition_msgs::PolygonArray>(
      "/zonotope_visualization", 1);
  obs_publisher_.publish(::roahm::Obs11_polarr());

  // Keep ROS alive
  ros::spin();
}

#undef ROS_INFO_STR
