#include "param_gen.hpp"

#include <algorithm>  // for max, min, min...
#include <array>      // for array
#include <chrono>     // for operator-
#include <cmath>      // for cos, sin, M_PI
#include <cstddef>    // for size_t
#include <cstdint>    // for uint8_t
#include <iostream>   // for operator<<
#include <iterator>   // for distance
#include <memory>     // for allocator

#include "IpIpoptApplication.hpp"                       // for IpoptApplication
#include "IpReturnCodes_inc.h"                          // for Solve_Succeeded
#include "IpSmartPtr.hpp"                               // for SmartPtr, Get...
#include "IpSolveStatistics.hpp"                        // for SolveStatistics
#include "IpTNLP.hpp"                                   // for TNLP
#include "IpTypes.hpp"                                  // for Number
#include "common.hpp"                                   // for IndexT
#include "cost_fcn.hpp"                                 // for GenCostFcn
#include "fl_zono_ipopt_problem.hpp"                    // for FlZonoIpoptPr...
#include "fmt/core.h"                                   // for format
#include "gencon.hpp"                                   // for WorldToLocalD...
#include "geometry_msgs/Point.h"                        // for Point, Point_
#include "geometry_msgs/Pose.h"                         // for Pose, Pose_
#include "geometry_msgs/Quaternion.h"                   // for Quaternion
#include "get_waypoint_from_path.hpp"                   // for DistPointOnPo...
#include "jsk_recognition_msgs/PolygonArray.h"          // for PolygonArray
#include "nav_msgs/Path.h"                              // for Path, Path_
#include "ros/time.h"                                   // for operator<<, Time
#include "rover_control_msgs/GenParamInfo.h"            // for GenParamInfo
#include "rover_control_msgs/MatlabPlotInfo.h"          // for MatlabPlotInfo
#include "rover_control_msgs/OnlineDebugMsg.h"          // for OnlineDebugMsg
#include "rover_control_msgs/RoverDebugStateStamped.h"  // for RoverDebugSta...
#include "simple_util.hpp"                              // for Clamp, ClampW...
#include "std_msgs/Bool.h"                              // for Bool
#include "std_msgs/Float64MultiArray.h"                 // for Float64MultiA...
#include "std_msgs/Header.h"                            // for Header, Header_
#include "tf2/LinearMath/Quaternion.h"                  // for Quaternion

namespace roahm {

namespace {

/// Gets the current time, intended to help add benchmarks.
/// \return the current time as a ::std::chrono::time_point
inline auto Tick() { return std::chrono::high_resolution_clock::now(); }

/// Subtracts two time instances and returns the duration in seconds.
/// \tparam T the type of the first parameter, \p t1, most likely a
/// ::std::chrono::time_point
/// \tparam S the type of the second parameter, \p t0, most likely a
/// ::std::chrono::time_point
/// \param t1 the "end" time
/// \param t0 the "start" time
/// \return the duration from \p t0 until \p t1 in seconds.
template <typename T, typename S>
double GetDeltaS(const T& t1, const S& t0) {
  return static_cast<double>(
             std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0)
                 .count()) /
         1.0e9;
}

PredictInfo PredictInfoFromControllerMessage(
    const rover_control_msgs::RoverDebugStateStamped& msg) {
  PredictInfo predict_info{};
  predict_info.x0_ = msg.x;
  predict_info.y0_ = msg.y;
  predict_info.h0_ = ToAngle(msg.h);
  predict_info.u0_ = msg.u;
  predict_info.v0_ = msg.v;
  predict_info.r0_ = msg.r;
  predict_info.should_predict_ = msg.running_auto;
  if (msg.running_auto) {
    ROS_INFO("Setting prediction initial state to incoming state msg");
    predict_info.poly_var0_ = msg.au;
    predict_info.manu_type_ = ManuFromUint8(msg.manu_type);
    if (IsSpd(predict_info.manu_type_)) {
      predict_info.poly_var1_ = msg.u;
    } else {
      predict_info.poly_var1_ = msg.ay;
    }
    predict_info.use_15_30_ = false;
  }
  return predict_info;
}

std::vector<FrsSelectInfo> GetParamsToSearchOver(const FrsTotal& frs,
                                                 double& state_u,
                                                 double& state_v,
                                                 double& state_r) {
  // Set of FRSes to search
  std::vector<FrsSelectInfo> frses_to_search;  // vector of struct

  // (En/Dis)able certain manu types for testing
  const bool search_au = true;
  const bool search_lan = true;
  const bool search_dir = true;
  const bool can_clamp_u = true;
  const auto u0_idx = frs.SelectU0Idx(state_u, can_clamp_u);
  std::cout << "u0_idx is: " << u0_idx << std::endl;
  ROS_INFO("SEARCH AU");
  auto add_frs = [&state_v, &state_r, &frses_to_search](
                     const long first_idx,
                     const long last_idx,  // [first, last)
                     const bool is_table_populated, const FrsMega& vehrs,
                     const ManuType& manu_type, std::size_t u0_idx,
                     bool is_outer, int other_idx, bool also_mirror) -> void {
    std::cout << "First_idx: " << first_idx << std::endl;
    std::cout << "Last_idx: " << last_idx << std::endl;

    for (int i = first_idx; i < last_idx; ++i) {
      const int idx1 = is_outer ? i : other_idx;
      const int idx2 = is_outer ? other_idx : i;
      if (is_table_populated) {
        const auto vr_minmax = vehrs.GetMinMax(manu_type, idx1, idx2);
        const auto [v0_min, v0_max, r0_min, r0_max] = vr_minmax;
        state_v = ClampWithWarn(state_v, v0_min, v0_max, "V");
        state_r = ClampWithWarn(state_r, r0_min, r0_max, "R");
      }
      frses_to_search.emplace_back(manu_type, u0_idx, idx1, idx2, false);
      if (also_mirror) {
        frses_to_search.emplace_back(manu_type, u0_idx, idx1, idx2, true);
      }
    }
  };
  if (search_au) {
    ROS_INFO("SEARCH SPD");
    const bool alternating_au = frs.AlternatingAu();
    const long num_megas_l = static_cast<long>(frs.megas_.size());
    const long au_u0_idx_min =
        alternating_au ? std::max(u0_idx - 1, 0l) : u0_idx;

    // au_u0_idx_max in [0, frs.megas_.size()]
    const long au_u0_idx_max =
        alternating_au ? std::min(u0_idx + 2, num_megas_l) : u0_idx + 1;

    // au_u0_idx in [au_u0_idx_min, au_u0_idx_max)
    for (int au_u0_idx = au_u0_idx_min; au_u0_idx < au_u0_idx_max;
         ++au_u0_idx) {
      const auto& mega_u = frs.megas_.at(au_u0_idx);
      if (mega_u.au_.empty()) {
        continue;
      }
      const long max_bin = static_cast<long>(mega_u.au_.size()) - 1;
      const bool is_table_populated =
          mega_u.IsTablePopulated(ManuType::kSpdChange);
      const auto manu_type = ManuType::kSpdChange;
      const long num_au_above = 2;
      const long num_au_below = num_au_above;
      const auto first_idx = Clamp(u0_idx - num_au_below, 0, max_bin + 1);
      const auto last_idx = Clamp(u0_idx + num_au_above, 0, max_bin + 1);
      add_frs(first_idx, last_idx, is_table_populated, mega_u, manu_type,
              au_u0_idx, true, 0, false);
    }
  }

  const auto& mega_u = frs.megas_.at(u0_idx);
  auto add_dir_lan_frs = [&mega_u, &add_frs,
                          &u0_idx](const ManuType manu_type) {
    const auto& manu_set = mega_u.GetDirLanSet(manu_type);
    const bool is_table_populated = mega_u.IsTablePopulated(manu_type);
    const int manu_set_size = static_cast<int>(manu_set.size());
    for (int i = 0; i < manu_set_size; ++i) {
      const int num_search = std::min(
          1, static_cast<int>(manu_set.at(i).size()));  
      add_frs(0, num_search, is_table_populated, mega_u, manu_type, u0_idx,
              false, i, true);
    }
  };
  if (search_lan) {
    ROS_INFO("SEARCH LAN");
    add_dir_lan_frs(ManuType::kLanChange);
  }

  if (search_dir) {
    ROS_INFO_STREAM("SEARCH DIR");
    add_dir_lan_frs(ManuType::kDirChange);
  }

  return frses_to_search;
}

std::vector<Ipopt::SmartPtr<Ipopt::IpoptApplication>> GetIpApps(int num_apps) {
  // param: number of ipopt you want to construct, equals to number of threads
  // returns vector of Ipopt solvers and init their params

  std::vector<Ipopt::SmartPtr<Ipopt::IpoptApplication>> app_vec;
  app_vec.reserve(num_apps);
  for (int i = 0; i < num_apps; ++i) {
    app_vec.push_back(IpoptApplicationFactory());
  }
  for (auto& app : app_vec) {
    auto opts = app->Options();
    opts->SetNumericValue("tol", 1.0e-3);
    opts->SetStringValue("hessian_constant", "yes");
    opts->SetStringValue("linear_solver", "ma57");
    opts->SetNumericValue("ma57_pre_alloc", 5.0);
    opts->SetIntegerValue("print_level", 0);
    opts->SetIntegerValue("max_iter", 15);
    opts->SetIntegerValue("acceptable_iter", 15);     // def 15
    opts->SetNumericValue("acceptable_tol", 1.0e-3);  // def 1.0e-6
  }
  return app_vec;
}

}  // namespace

ParamRet GenerateParameter(
    const FrsTotal& frs,
    const jsk_recognition_msgs::PolygonArray& latest_read_obs,
    const PointXYH& x_des_normal, const PointXYH& x_des_mirror,
    const PointXYH& x_des_global, RoverState fp_state,
    const RoverState& current_state, const bool is_low,
    const nav_msgs::Path& full_path,
    const ros::Publisher& obs_center_publisher_) {
  const auto gen_param_start_time = Tick();
  std::cout << "State to search: " << fp_state.ToStringXYHUVR() << std::endl;

  const auto latest_read_obs_global = latest_read_obs;

  // NOTE: fp_state u, v, r can be modified here.
  const auto frses_to_search =
      GetParamsToSearchOver(frs, fp_state.u_, fp_state.v_, fp_state.r_);

  WGUARD_ROS_INFO_STREAM(
      "number of frses to search: " << frses_to_search.size());

  // Store a copy of v, r inverted for mirror
  const double mirror_v = -fp_state.v_;
  const double mirror_r = -fp_state.r_;
  std::atomic<int> successful_runs = 0;
  std::atomic<int> failure_runs = 0;
  std::atomic<int> next_idx = 0;
  const int num_search = static_cast<int>(frses_to_search.size());
  const int num_apps = 6;

  WGUARD_ROS_INFO("APP INIT");
  // Initialize and set options on IPOPT
  const auto app_vec = GetIpApps(num_apps);

  // Number of outer loops = ceil(num_search / num_apps)
  const int num_outer = (num_search + (num_apps - 1)) / num_apps;

  std::vector<double> cost_vals(num_search);
  std::vector<double> param_vals_mirrored(num_search);

  // DO NOT USE vector<bool>: that implementation is **NOT** thread safe
  std::vector<int> success_vals(num_search);

  // TODO could just use a std::atomic<int> for the index.
  for (int outer_idx = 0; outer_idx < num_outer; ++outer_idx) {
    // Run each ipopt application in parallel
#pragma omp parallel for  
    for (int app_idx = 0; app_idx < num_apps; ++app_idx) {
      const int total_idx = next_idx++;
      if (total_idx >= num_search) {
        continue;
      }
      auto& app = app_vec.at(app_idx);
      if (app->Initialize() != Ipopt::Solve_Succeeded) {
        WGUARD_ROS_ERROR("\n\n*** Error during IPOPT initialization!");
      }
      const auto& frs_select_info = frses_to_search.at(total_idx);
      const bool mirror = frs_select_info.mirror_;
      const double mirror_mult = mirror ? -1.0 : 1.0;
      const auto& frs_to_use = frs.GetVehrs(frs_select_info);
      TGUARD_ROS_INFO_STREAM("\n\nRUNNING NEW:\n\n");
      TGUARD_ROS_INFO_STREAM(
          "ManuType: " << ToString(frs_select_info.manu_type_));
      TGUARD_ROS_INFO_STREAM("Mirror:   "
                             << mirror << " idxu0: " << frs_select_info.idxu0_
                             << " idx0:  " << frs_select_info.idx0_
                             << " idx1:  " << frs_select_info.idx1_
                             << " [app idx]: " << app_idx);  // t0 idx

      const auto obs_info_to_use = ConvertObsToZonoWithHeading(
          latest_read_obs_global, fp_state.GetXYH(), mirror);

      // DEBUG ONLY//////////////////////////////////////////
      std_msgs::Float64MultiArray array_msg;
      for (auto val : obs_info_to_use.obs_centers_) {
        array_msg.data.push_back(val);
      }
      array_msg.data.push_back(fp_state.GetHeading());
      if (not mirror) {
        obs_center_publisher_.publish(array_msg);
      }
      ///////////////////////////////////////////////////////

      PRINT_VAR(obs_info_to_use.num_obs_)
      const auto& x_des_to_use = mirror ? x_des_mirror : x_des_normal;
      const auto v_to_use = mirror ? mirror_v : fp_state.v_;
      const auto r_to_use = mirror ? mirror_r : fp_state.r_;
      TGUARD_ROS_INFO_STREAM("xdes_to_use: (" << x_des_to_use.x_ << ", "
                                              << x_des_to_use.y_ << ", "
                                              << x_des_to_use.h_ << ")"
                                              << " [app idx]: " << app_idx);
      TGUARD_ROS_INFO_STREAM("vr_to_use: " << v_to_use << ", " << r_to_use
                                           << " [app idx]: " << app_idx);

      const IndexT zono_desired_idx = frs_to_use.t_eval_idx_;
      const auto cost_fcn_info = GenCostFcn(
          frs_to_use, zono_desired_idx, fp_state.u_, v_to_use, r_to_use,
          x_des_to_use.x_, x_des_to_use.y_, x_des_to_use.h_);
      auto sliced = frs_to_use.SliceAt(fp_state.u_, v_to_use, r_to_use);
      const auto cons = GenerateConstraints(sliced, obs_info_to_use);

      std::shared_ptr<Ipopt::Number[]> a_mat = cons.a_con_arr_;
      std::shared_ptr<Ipopt::Number[]> b_mat = cons.b_con_arr_;

      const double k_rng = frs_to_use.k_rng_;

      // Create an instance of your nlp...
      Ipopt::SmartPtr<Ipopt::TNLP> mynlp =
          new fl_zono_ipopt_problem::FlZonoIpoptProblem(
              a_mat, b_mat, cost_fcn_info, cons.zono_startpoints_,
              cons.zono_obs_sizes_, k_rng);

      const Ipopt::ApplicationReturnStatus status = app->OptimizeTNLP(mynlp);
      const bool ipopt_success = status == Ipopt::Solve_Succeeded or
                                 status == Ipopt::Solved_To_Acceptable_Level;
      const auto* mnlp =
          dynamic_cast<fl_zono_ipopt_problem::FlZonoIpoptProblem*>(
              GetRawPtr(mynlp));
      const bool was_really_feasible = mnlp->FoundFeasible();

      if (ipopt_success or was_really_feasible) {
        // Retrieve some statistics about the solve
        ++successful_runs;
        const Ipopt::Number final_obj =
            ipopt_success ? (app->Statistics()->FinalObjective())
                          : mnlp->GetFeasibleCost();
        const auto sln_k_ =
            ipopt_success ? (mnlp->sln_k_) : (mnlp->GetFeasibleParam());
        success_vals.at(total_idx) = true;
        cost_vals.at(total_idx) = final_obj;
        param_vals_mirrored.at(total_idx) = mirror_mult * sln_k_;

        const std::string output_str = fmt::format(
            "IPOPT: SUCCESS [K: {:.2f} (Man: {:.2f})] [MANU: {}] [MIR: {}] "
            "[COST: {:.2f}] [IP_SUCCESS: {}] [REALLY: {}] [X: {:.4f}] [Y: "
            "{:.4f}] [H: {:.4f}] [U: {:.4f}] [V: {:.4f}] [R: {:.4f}] [app idx: "
            "{:d}]\n",
            param_vals_mirrored.at(total_idx), mnlp->GetFeasibleCost(),
            ToString(frs_select_info.manu_type_), mirror, final_obj,
            ipopt_success, was_really_feasible, fp_state.x_, fp_state.y_,
            fp_state.GetHeading(), fp_state.u_, fp_state.v_, fp_state.r_,
            app_idx);
        TGUARD_ROS_INFO_STR(output_str);
      } else {
        cost_vals.at(total_idx) = 1.0e+10;
        success_vals.at(total_idx) = false;
        param_vals_mirrored.at(total_idx) = 0.0;
        ++failure_runs;
      }
    }
  }
  const auto gen_param_end_time = Tick();
  rover_control_msgs::OnlineDebugMsg ret_val;
  rover_control_msgs::MatlabPlotInfo mat_ret;
  mat_ret.was_successful = false;

  if (frses_to_search.empty()) {
    std::cout << "early return" << std::endl;
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

  ret_val.manu_type = ToString(final_manu_type);
  ret_val.idxu0 = frs_min_info.idxu0_;
  ret_val.idx0 = frs_min_info.idx0_;  // Corresponds to the param bin index
  ret_val.idx1 = t0_idx_c_style;      // Corresponds to the t0 index
  ret_val.mirror = frs_min_info.mirror_;
  ret_val.final_cost = cost_vals.at(min_cost_idx);
  ret_val.final_param = param_vals_mirrored.at(min_cost_idx);
  const std::string output_str = fmt::format(
      "PUBLISHING [K: {:.2f}] [MANU: {}] [MIR: {}] [COST: {:.2f}]\n",
      ret_val.final_param, ret_val.manu_type, ret_val.mirror,
      ret_val.final_cost);
  WGUARD_ROS_INFO_STREAM(output_str);
  ret_val.time_to_compute_s =
      GetDeltaS(gen_param_end_time, gen_param_start_time);
  ret_val.num_successful = successful_runs;
  ret_val.num_failed = failure_runs;
  ret_val.x0 = fp_state.x_;
  ret_val.y0 = fp_state.y_;
  ret_val.h0 = fp_state.GetHeading();
  ret_val.u0 = fp_state.u_;
  ret_val.w0 = fp_state.u_;  
  ret_val.v0 = fp_state.v_;
  ret_val.r0 = fp_state.r_;
  ret_val.r_err_sum0 = 0.0;  
  ret_val.h_err_sum0 = 0.0; 
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
  ret_val.times = {};  

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
#if SIMULATION
  mat_ret.full_rob_path = full_path;
#else
  // mat_ret.full_rob_path = full_path;
#endif
  std::cout << "mat_ret.full_rob_path" << mat_ret.full_rob_path.poses.size()
            << std::endl;
  // Current state at time of previous trajectory start
  mat_ret.x0 = current_state.x_;
  mat_ret.y0 = current_state.y_;
  mat_ret.h0 = current_state.GetHeading();
  mat_ret.u0 = current_state.u_;
  mat_ret.v0 = current_state.v_;
  mat_ret.r0 = current_state.r_;
  // The predicted starting state of our trajectory
  mat_ret.pred_x = fp_state.x_;
  mat_ret.pred_y = fp_state.y_;
  mat_ret.pred_h = fp_state.GetHeading();
  mat_ret.pred_u = fp_state.u_;
  mat_ret.pred_v = fp_state.v_;
  mat_ret.pred_r = fp_state.r_;
  // Waypoint
  mat_ret.wp_x = x_des_global.x_;
  mat_ret.wp_y = x_des_global.y_;
  mat_ret.wp_h = x_des_global.h_;

  std::cout << "mat_ret.wp_x is: " << mat_ret.wp_x << std::endl;
  std::cout << "mat_ret.wp_y is: " << mat_ret.wp_y << std::endl;

  // Chosen Values
  mat_ret.u0_idx = u0_idx_1_indexed;
  mat_ret.idx0 = idx0_1_indexed;
  mat_ret.idx1 = idx1_1_indexed;
  mat_ret.manu_type = manu_type_uint8_char;
  mat_ret.k_param = final_param;

  // Obstacles
#if SIMULATION
  mat_ret.obstacles = latest_read_obs;
#else
  // mat_ret.obstacles = latest_read_obs;
#endif

  return {ret_val, mat_ret};
}

void ParameterGenerator::PublishPathAndWaypointDebug(
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

ParameterGenerator::FpReqSimInfo ParameterGenerator::ChooseParameters() {
  if (not SuccessfullyLoaded()) {
    WGUARD_ROS_ERROR(
        "Not choosing parameters. FRS was not successfully loaded.\n");
    return FpReqSimInfo::Fail();
  }
  WGUARD_ROS_INFO("Choosing Parameters...");
  // forward_predicted_state_.u_ = 0.460655;

  const double state_x = forward_predicted_state_.x_;
  const double state_y = forward_predicted_state_.y_;
  const double state_h = forward_predicted_state_.GetHeading();
  const double state_u = forward_predicted_state_.u_;
  // FPS: Forward Predicted State
  WGUARD_ROS_INFO_STREAM("FPS: " << forward_predicted_state_.ToStringXYHUVR());
  ChooseDesiredWaypoint();

  const PointXYH state_xyh = forward_predicted_state_.GetXYH();
  const auto x_des_normal_ = x_des_.ToLocalFrame(state_xyh);
  const auto x_des_mirror_ = x_des_normal_.Mirror();

  const bool is_low = state_u < frs_.MinU0();
  const auto& frs_file_to_use = is_low ? frs_low_ : frs_;
  const auto t0gp = Tick();

  // Structured binding
  const auto [latest_param, plot_info_param] = GenerateParameter(
      frs_file_to_use, latest_read_obs_, x_des_normal_, x_des_mirror_, x_des_,
      forward_predicted_state_, predict_info_.GetInitRoverState(), is_low,
      hlp_path_input_, obs_center_publisher_);

  const auto t1gp = Tick();
  WGUARD_ROS_INFO_STREAM("GP Took: " << GetDeltaS(t1gp, t0gp) << "s");
  PublishPathAndWaypointDebug(latest_param);
  matlab_plot_info_pub_.publish(plot_info_param);

  if (latest_param.num_successful > 0) {
    const std::string manu_str = latest_param.manu_type;
    const double final_param = latest_param.final_param;
    const ManuType manu_type = FromString(manu_str);
    const double au = GetGenParamAu(manu_type, state_u, final_param);
    const double ay = GetGenParamAy(manu_type, state_u, final_param);
    const int t0_idx = latest_param.idx1;
    const double t0_offset = t0_idx * 1.5;
    rover_control_msgs::GenParamInfo gen_param_out_msg;
    gen_param_out_msg.au = au;
    gen_param_out_msg.ay = ay;
    gen_param_out_msg.t0_offset = t0_offset;
    gen_param_out_msg.header.stamp = ros::Time::now();
    gen_param_out_msg.manu_type = ManuToUint8(manu_type);
    gen_param_publisher_.publish(gen_param_out_msg);
    return {true, au, ay, manu_type};
  } else {
    ROS_INFO("NOT PUBLISHING PARAMETER");
  }
  return FpReqSimInfo::Fail();
}

void ParameterGenerator::ChooseDesiredWaypoint() {
  if (not hlp_path_input_.poses.empty()) {
    hlp_path_xyh_.clear();
    for (const geometry_msgs::PoseStamped& pose_stamp : hlp_path_input_.poses) {
      const auto& pos = pose_stamp.pose.position;
      const double yaw = GetOrientationYaw(pose_stamp.pose.orientation);
      const double x = pos.x;
      const double y = pos.y;
      hlp_path_xyh_.emplace_back(x, y,
                                 yaw);  // push new poses in to the path vector
    }
    waypoint_interp_params_ = GetWaypointInterpParams();
    auto x_desireds = GetWaypointFromPath(
        forward_predicted_state_, hlp_path_xyh_, waypoint_interp_params_);
    x_des_ = x_desireds.back();
  } else {
    ROS_ERROR("NO PATH PROVIDED");
    x_des_ = {};
  }
}

void ParameterGenerator::CallbackHLP(const nav_msgs::Path& hlp_path) {
  hlp_path_input_ = hlp_path;
}

void ParameterGenerator::CallbackRunSimulation(const std_msgs::Bool run) {
  WGUARD_ROS_INFO("CallbackRunSimulation\n");
  if (run.data) {
    RunSimulation();
  }
}

ParameterGenerator::FpReqSimInfo ParameterGenerator::CallbackFpReqInternal(
    const rover_control_msgs::RoverDebugStateStamped& msg) {
  // Start timing
  const auto t0 = Tick();
  WGUARD_ROS_INFO_STREAM("MSG INPUT: " << msg.x << ", " << msg.y << ", "
                                       << msg.h);
  predict_info_ = PredictInfoFromControllerMessage(msg);

  // Generate predicted state and then trajectory parameters
  ForwardPredict();
  auto ret = ChooseParameters();

  // Print time
  const auto t1 = Tick();
  WGUARD_ROS_INFO_STREAM("Callback FP Took: " << GetDeltaS(t1, t0) << "s");
  return ret;
}

void ParameterGenerator::CallbackFpReq(
    const rover_control_msgs::RoverDebugStateStamped& msg) {
  CallbackFpReqInternal(msg);
}

void ParameterGenerator::CallbackObs(
    const jsk_recognition_msgs::PolygonArray& msg) {
  obs_have_been_appended_ = false;
  latest_read_obs_ = msg;
}

void ParameterGenerator::RunSimulation() {
  WGUARD_ROS_INFO("RunSimulation\n");


  rover_control_msgs::RoverDebugStateStamped init_msg{};

  init_msg.x = -2.0;
  init_msg.y = 0.0;
  init_msg.h = 0.0;
  init_msg.u = 0.2;
  init_msg.v = 0.0;
  init_msg.r = 0.0;

  init_pos_pub_.publish(init_msg);

  bool is_stuck = false;
#if SIMULATION
  for (int i = 0; (i < 10) and (not is_stuck); ++i) {
#else
  for (int i = 0; (i < 4) and (not is_stuck); ++i) {
#endif
    WGUARD_ROS_INFO_STREAM("SIM ITERATION BEG: " << i);
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
    WGUARD_ROS_INFO_STREAM("SIM ITERATION END: " << i << " [stuck: " << is_stuck
                                                 << "]");
    WGUARD_ROS_INFO(
        "===================================================================="
        "==============");
  }
}

}  // namespace roahm
