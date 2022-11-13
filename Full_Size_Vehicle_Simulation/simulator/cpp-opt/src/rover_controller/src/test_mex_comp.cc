#include <boost/smart_ptr/make_shared_object.hpp>  // for make_shared
#include <boost/type_index/type_index_facade.hpp>  // for operator==
#include <chrono>
#include <random>
#include <string>  // for string, alloc...

#include "IpIpoptApplication.hpp"  // for IpoptApplication
#include "IpReturnCodes_inc.h"     // for Solve_Succeeded
#include "IpSmartPtr.hpp"          // for SmartPtr, Get...
#include "IpSolveStatistics.hpp"   // for SolveStatistics
#include "IpTNLP.hpp"              // for TNLP
#include "IpTypes.hpp"             // for Number
#include "fl_zono_ipopt_problem.hpp"
#include "frs_loader.hpp"
#include "gencon.hpp"
#include "mex.hpp"
#include "mexAdapter.hpp"
#include "rover_state.hpp"


namespace roahm {
namespace {
template <typename T, typename S>
double GetDeltaS(const T& t1, const S& t0) {
  return static_cast<double>(
             std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0)
                 .count()) /
         1.0e9;
}
std::vector<FrsSelectInfo> GetParamsToSearchOver(const FrsTotal& frs,
                                                 double& state_u,
                                                 double& state_v,
                                                 double& state_r) {
  std::cout << "===== GET PARAMS TO SEARCH =====" << std::endl;
  // Set of FRSes to search
  std::vector<FrsSelectInfo> frses_to_search;  // vector of struct

  // (En/Dis)able certain manu types
  const bool search_au = true;
  const bool search_lan = true;
  const bool search_dir = true;
  const bool can_clamp_u = true;
  const auto u0_idx = frs.SelectU0Idx(state_u, can_clamp_u);
	const double min_spd_au = 0.0;
	const double max_spd_au = state_u + 4.0;
  std::cout << "u0_idx is: " << u0_idx << std::endl;
  std::cout << "SEARCH AU" << std::endl;
  auto add_frs = [&state_v, &state_r, &frses_to_search, &min_spd_au, &max_spd_au](
                     const long first_idx,
                     const long last_idx,  // [first, last)
                     const bool is_table_populated, const FrsMega& vehrs,
                     const ManuType& manu_type, std::size_t u0_idx,
                     bool is_outer, int other_idx, bool also_mirror) -> void {
    //DEL std::cout << "First_idx: " << first_idx << std::endl;
    //DEL std::cout << "Last_idx: " << last_idx << std::endl;

    for (int i = first_idx; i < last_idx; ++i) {
      const int idx1 = is_outer ? i : other_idx;
      const int idx2 = is_outer ? other_idx : i;
      if (is_table_populated) {
        const auto vr_minmax = vehrs.GetMinMax(manu_type, idx1, idx2);
        const auto [v0_min, v0_max, r0_min, r0_max] = vr_minmax;
        state_v = ClampWithWarn(state_v, v0_min, v0_max, "V");
        state_r = ClampWithWarn(state_r, r0_min, r0_max, "R");
      }
      //std::cout << "Adding Nor FRS " << ToString(manu_type) << std::endl;
      frses_to_search.emplace_back(manu_type, u0_idx, idx1, idx2, false);
      if (also_mirror) {
        //std::cout << "Adding Mir FRS " << ToString(manu_type) << std::endl;
        frses_to_search.emplace_back(manu_type, u0_idx, idx1, idx2, true);
      }
    }
  };
  if (search_au) {
    //DEL ROS_INFO("SEARCH SPD");
    const bool alternating_au = frs.AlternatingAu();
    //TODO DEL const long num_megas_l = static_cast<long>(frs.megas_.size());
    //TODO DEL const long au_u0_idx_min =
    //TODO DEL     alternating_au ? std::max(u0_idx - 1, 0l) : u0_idx;

    //TODO DEL // au_u0_idx_max in [0, frs.megas_.size()] 
    //TODO DEL // NOTE: closed upper bound since the lookup will be [min, max)
    //TODO DEL const long au_u0_idx_max =
    //TODO DEL     alternating_au ? std::min(u0_idx + 2, num_megas_l) : u0_idx + 1;
    const long au_u0_idx_min = u0_idx;
    const long au_u0_idx_max = au_u0_idx_min + 1;

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
			for (int i = 0; i < mega_u.au_.size(); ++i) {
				const double center_k = mega_u.au_.at(i).GetCenterK();
				if ((center_k >= min_spd_au) and (center_k <= max_spd_au)) {
					std::cout << "Center K: " << center_k << "[idx: " << i << "]" << std::endl;
        	add_frs(i, i+1, is_table_populated, mega_u, manu_type, au_u0_idx, true, 0, false);
          std::cout << "Adding SPEED CHANGE [TOTAL SIZE: " << frses_to_search.size() << "] [NO SPD: " << mega_u.au_.size() << "]" << std::endl;
				}
			}
      //del TODO FIXME const long num_au_above = 2;
      //del TODO FIXME const long num_au_below = num_au_above;
      //del TODO FIXME const auto first_idx = Clamp(u0_idx - num_au_below, 0, max_bin + 1);
      //del TODO FIXME const auto last_idx = Clamp(u0_idx + num_au_above, 0, max_bin + 1);
      //del TODO FIXME const auto first_idx = 0;
      //del TODO FIXME const auto last_idx = max_bin + 1;
      //del add_frs(first_idx, last_idx, is_table_populated, mega_u, manu_type,
      //del         au_u0_idx, true, 0, false);
      //const auto continuous_rng_begin = Clamp(u0_idx - 2, 0, max_bin + 1);
      //const auto continuous_rng_end = Clamp(u0_idx, 0, max_bin + 1);
      //const int spd_gap = 1;
      //for (int i = 0; i < continuous_rng_begin; i += spd_gap) {
      //  add_frs(i, i+1, is_table_populated, mega_u, manu_type, au_u0_idx, true, 0, false);
      //}
      //for (int i = continuous_rng_begin; i < continuous_rng_end; ++i) {
      //  add_frs(i, i+1, is_table_populated, mega_u, manu_type, au_u0_idx, true, 0, false);
      //}
    }
  }

  const auto& mega_u = frs.megas_.at(u0_idx);
  auto add_dir_lan_frs = [&mega_u, &add_frs,
                          &u0_idx, &frses_to_search](const ManuType manu_type) {
    const auto& manu_set = mega_u.GetDirLanSet(manu_type);
    const bool is_table_populated = mega_u.IsTablePopulated(manu_type);
    const int manu_set_size = static_cast<int>(manu_set.size());
    for (int i = 0; i < manu_set_size; ++i) {
      const int num_search =
          static_cast<int>(manu_set.at(i).size());  // TODO Remove Min
      // const int num_search = std::min(
      //    1, static_cast<int>(manu_set.at(i).size()));  // TODO Remove Min
      add_frs(0, num_search, is_table_populated, mega_u, manu_type, u0_idx,
              false, i, true);
      std::cout << "Adding D/LAN CHANGE [TOTAL SIZE: " << frses_to_search.size() << "]" << std::endl;
    }
  };
  if (search_lan) {
    //DEL TODO REMOVE ROS_INFO("SEARCH LAN");
    //DEL std::cout << "SEARCH LAN" << std::endl;
    add_dir_lan_frs(ManuType::kLanChange);
  }

  if (search_dir) {
    //DEL TODO REMOVE ROS_INFO_STREAM("SEARCH DIR");
    //DEL std::cout << "SEARCH DIR" << std::endl;
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
    opts->SetStringValue("hessian_constant", "no");
    opts->SetStringValue("linear_solver", "ma57");
    opts->SetNumericValue("ma57_pre_alloc", 5.0);
    opts->SetIntegerValue("print_level", 3);
    opts->SetIntegerValue("max_iter", 15);
    opts->SetIntegerValue("acceptable_iter", 15);     // def 15
    opts->SetNumericValue("acceptable_tol", 1.0e-3);  // def 1.0e-6
    opts->SetStringValue("print_timing_statistics", "yes");
  }
  return app_vec;
}
inline auto Tick() { return std::chrono::high_resolution_clock::now(); }
}  // namespace

struct SimParam {
  bool successful_;
  double res_;
  ::roahm::ManuType manu_type_;
  int idx0_;
  int idx1_;
};

SimParam GenerateSimParameter(
    const FrsTotal& frs,
    const ObsInfo global_obs_info,
    const PointXYH& x_des_local,  // TODO make sure this is actually local
    ::roahm::RoverState fp_state) {
  const auto x_des_mirror = x_des_local.Mirror();
  const auto gen_param_start_time = Tick();
  std::cout << "State to search: " << fp_state.ToStringXYHUVR() << std::endl;

  // NOTE: fp_state u, v, r can be modified here.
  const auto frses_to_search =
      GetParamsToSearchOver(frs, fp_state.u_, fp_state.v_, fp_state.r_);

  std::cout << "number of frses to search: " << frses_to_search.size() << std::endl;
  //DEL WGUARD_ROS_INFO_STREAM(
  //DEL     "number of frses to search: " << frses_to_search.size());

  // Store a copy of v, r inverted for mirror
  const double mirror_v = -fp_state.v_;
  const double mirror_r = -fp_state.r_;
  std::atomic<int> successful_runs = 0;
  std::atomic<int> failure_runs = 0;
  std::atomic<int> next_idx = 0;
  const int num_search = static_cast<int>(frses_to_search.size());
  const int num_apps = 13;

  //DEL WGUARD_ROS_INFO("APP INIT");
  std::cout << "APP INIT" << std::endl;
  // Initialize and set options on IPOPT
  const auto app_vec = GetIpApps(num_apps);

  // Number of outer loops = ceil(num_search / num_apps)
  const int num_outer = (num_search + (num_apps - 1)) / num_apps;

  std::vector<double> cost_vals(num_search);
  std::vector<double> param_vals_mirrored(num_search);
  std::vector<double> delta_y_vals(num_search);
  std::vector<double> delta_h_vals(num_search);

  // DO NOT USE vector<bool>: that implementation is **NOT** thread safe
  std::vector<int> success_vals(num_search);
	constexpr double kBigCost = 1.0e+10;

  //DEL std::cout << "NUM SEARCH: " << num_search << std::endl;
  // TODO could just use a std::atomic<int> for the index.
  for (int outer_idx = 0; outer_idx < num_outer; ++outer_idx) {
    // Run each ipopt application in parallel
#pragma omp parallel for  // JL: serial for debugging
    for (int app_idx = 0; app_idx < num_apps; ++app_idx) {
      const int total_idx = next_idx++;
      if (total_idx >= num_search) {
        continue;
      }
      auto& app = app_vec.at(app_idx);
      if (app->Initialize() != Ipopt::Solve_Succeeded) {
        std::cout << "\n\n*** Error during IPOPT initialization!" << std::endl;
      }
      const auto& frs_select_info = frses_to_search.at(total_idx);
			const auto curr_manu_type = frs_select_info.manu_type_;
      const bool mirror = frs_select_info.mirror_;
      const double mirror_mult = mirror ? -1.0 : 1.0;
      const auto& frs_to_use = frs.GetVehrs(frs_select_info);
      //DEL std::cout << "\n\nRUNNING NEW:\n\n" << std::endl;
      //DEL std::cout << "ManuType: " << ToString(frs_select_info.manu_type_)
      //DEL           << std::endl;
      //DEL std::cout << "Mirror:   " << mirror
      //DEL           << " idxu0: " << frs_select_info.idxu0_
      //DEL           << " idx0:  " << frs_select_info.idx0_
      //DEL           << " idx1:  " << frs_select_info.idx1_
      //DEL           << " [app idx]: " << app_idx << std::endl;  // t0 idx

      // TODO seems like we should just add a function to go state->PointXYH
      //DEL std::cout << "Relative To" << std::endl;
      const auto obs_info_to_use = global_obs_info.RelativeTo(fp_state.GetXYH(), mirror);
      //const auto obs_info_to_use = ConvertObsToZonoWithHeading(
      //    latest_read_obs_global, fp_state.GetXYH(), mirror);

      auto x_des_to_use = mirror ? x_des_mirror : x_des_local;
			if (IsLan(curr_manu_type)) {
				x_des_to_use.x_ = fp_state.u_ * 6.0 + 4.0 * 6.0;
				x_des_to_use.y_ *= 1.15;
			}
      if (IsDir(curr_manu_type) and (std::abs(fp_state.GetHeading())<=0.1)) {
        x_des_to_use.x_ = fp_state.u_ * 3.0 + 4.0 * 3.0;
				x_des_to_use.y_ *= 0.7;
			}
      if (IsSpd(curr_manu_type)) {
				x_des_to_use.x_ = 1.5 * ( std::min( x_des_to_use.x_ / 3.0, fp_state.u_ + 4) + fp_state.u_);
			}
      const auto v_to_use = mirror ? mirror_v : fp_state.v_;
      const auto r_to_use = mirror ? mirror_r : fp_state.r_;
      const IndexT zono_desired_idx = frs_to_use.t_eval_idx_;
			if (IsLan(curr_manu_type)) {
				std::cout << "[LAN] TIME: " << frs_to_use.zono_time_intervals_.at(zono_desired_idx).Min() << std::endl;
			}

      const auto JL_t1 = Tick();
      const auto cost_fcn_info = GenCostFcn(
          frs_to_use, zono_desired_idx, fp_state.u_, v_to_use, r_to_use,
          x_des_to_use.x_, x_des_to_use.y_, x_des_to_use.h_);
      const auto JL_t2 = Tick();
      std::cout << "Cost Generation Time: " << GetDeltaS(JL_t2, JL_t1) << std::endl;
      auto sliced = frs_to_use.SliceAt(fp_state.u_, v_to_use, r_to_use);
      const auto cons = GenerateConstraints(sliced, obs_info_to_use);
      const auto JL_t3 = Tick();
      std::cout << "Constraint Generation Time: " << GetDeltaS(JL_t3, JL_t2) << std::endl;
      std::shared_ptr<Ipopt::Number[]> a_mat = cons.a_con_arr_;
      std::shared_ptr<Ipopt::Number[]> b_mat = cons.b_con_arr_;

      const double k_rng = frs_to_use.k_rng_;

      // Create an instance of your nlp...
      Ipopt::SmartPtr<Ipopt::TNLP> mynlp =
          new fl_zono_ipopt_problem::FlZonoIpoptProblem(
              a_mat, b_mat, cost_fcn_info, cons.zono_startpoints_,
              cons.zono_obs_sizes_, k_rng, curr_manu_type);

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
        // TODO this is ok?
				// TODO remove underscore
        const auto sln_k_ =
            ipopt_success ? (mnlp->sln_k_) : (mnlp->GetFeasibleParam());
        success_vals.at(total_idx) = true;
        cost_vals.at(total_idx) = final_obj;
        param_vals_mirrored.at(total_idx) = mirror_mult * sln_k_;
				const double unmir_frs_dy = mnlp->ComputeDeltaY(sln_k_);
				delta_y_vals.at(total_idx) = std::abs(x_des_to_use.y_ - unmir_frs_dy);
				delta_h_vals.at(total_idx) = std::abs(fp_state.GetHeading() + mnlp->ComputeDeltaH(sln_k_));
	

        // TODO REVERT
        const double k = param_vals_mirrored.at(total_idx);
        const double cost = mnlp->GetFeasibleCost();
				const double dy = delta_y_vals.at(total_idx);
        const std::string str_manu = ToString(frs_select_info.manu_type_);
        const bool really_feas = was_really_feasible;
        std::cout << "IPOPT SUCCESS [K: " << k << "] [Cost: " << cost << "] [Manu: " << str_manu << "] [RF: " << really_feas << "] [DY: " << dy << "] [X_DES_Y: " << x_des_to_use.y_ << "] [MIRROR: " << mirror_mult << "] [UNMIR FRS DY: " << unmir_frs_dy << "] [IPOPT_ACTUAL: " << ipopt_success << "] [FEAS: " << was_really_feasible << "]" << std::endl;
        std::cout << "Agent State: " << fp_state.x_ << ", " << fp_state.y_ << ", " << fp_state.GetHeading() << std::endl;
        std::cout << "xyh (end): " << *(frs_to_use.xy_centers_.end()-2) << ", " << *(frs_to_use.xy_centers_.end()-1) << ", " << frs_to_use.h_centers_.back() << std::endl;
        
        //const std::string output_str = fmt::format(
        //   "IPOPT: SUCCESS [K: {:.2f} (Man: {:.2f})] [MANU: {}] [MIR: {}] "
        //   "[COST: {:.2f}] [IP_SUCCESS: {}] [REALLY: {}] [X: {:.4f}] [Y: "
        //   "{:.4f}] [H: {:.4f}] [U: {:.4f}] [V: {:.4f}] [R: {:.4f}] [app idx: "
        //   "{:d}]\n",
        //   param_vals_mirrored.at(total_idx), mnlp->GetFeasibleCost(),
        //   ToString(frs_select_info.manu_type_), mirror, final_obj,
        //   ipopt_success, was_really_feasible, fp_state.x_, fp_state.y_,
        //   fp_state.GetHeading(), fp_state.u_, fp_state.v_, fp_state.r_,
        //   app_idx);
        //std::cout << output_str;
        //TGUARD_ROS_INFO_STR(output_str);
      } else {
        cost_vals.at(total_idx) = kBigCost;
        success_vals.at(total_idx) = false;
        param_vals_mirrored.at(total_idx) = 0.0;
        delta_y_vals.at(total_idx) = 0.0;
        delta_h_vals.at(total_idx) = 0.0;
        ++failure_runs;
      }
    }
  }
  const auto gen_param_end_time = Tick();
  std::cout << "Time: " << GetDeltaS(gen_param_end_time, gen_param_start_time)
            << std::endl;
  std::cout << "Successful: " << successful_runs << std::endl;
  std::cout << "Failed:     " << failure_runs << std::endl;

  // JL add to debug
  std::cout << "cost value \n";
  for (int JL_idx = 0; JL_idx < cost_vals.size(); JL_idx++) {
	  std::cout << "cost = " << cost_vals.at(JL_idx) << " (manu:" << IsSpd(frses_to_search.at(JL_idx).manu_type_) << IsDir(frses_to_search.at(JL_idx).manu_type_) << IsLan(frses_to_search.at(JL_idx).manu_type_) << " )" << param_vals_mirrored.at(JL_idx) << "\n"; 
  }


  if (frses_to_search.empty()) {
    std::cout << "early return" << std::endl;
    return {};
  }
  const auto min_cost_it = std::min_element(cost_vals.begin(), cost_vals.end());
  auto min_cost_idx = std::distance(cost_vals.begin(), min_cost_it);
  if (min_cost_idx > num_search) {
    std::cerr << "\n\n\nMIN COST OUT OF SEARCH BOUNDS\n\n\n";
    min_cost_idx = num_search - 1;
  }
	{
	const auto init_manu_type = frses_to_search.at(min_cost_idx).manu_type_;
  bool min_cost_is_lan = IsLan(init_manu_type);
  bool min_cost_is_dir = IsDir(init_manu_type);
	double curr_min_cost = *min_cost_it;
	constexpr double kMinGoodHeading = 10.0 * (M_PI / 360.0);
	const bool has_okay_heading = std::abs(fp_state.GetHeading()) <= kMinGoodHeading;
	const double init_delta_y = std::abs(delta_y_vals.at(min_cost_idx));
	const bool initial_min_cost_close_enough = init_delta_y < 0.9;
  const bool init_speed_high_enough_for_lan = fp_state.u_ >= 20.0;
  const bool wpt_is_close_n_diff_lane = (x_des_local.x_ < 90) and (std::abs(x_des_local.y_) > 0.8);
  const bool ini_min_cost_close_but_hi_spd_dir = initial_min_cost_close_enough and init_speed_high_enough_for_lan and min_cost_is_dir;
	//std::cout << "[DBG] Initial Min Cost Dy: " << init_delta_y << std::endl;
	//std::cout << "[DBG] Min Cost Is Lan: " << min_cost_is_lan << std::endl;
	//std::cout << "[DBG] Has Okay Heading: " << has_okay_heading << std::endl;
  //std::cout << "[JL] cond "<< init_delta_y  << "   "<<(init_speed_high_enough_for_lan or wpt_is_close_n_diff_lane) << std::endl;
	if (not has_okay_heading) {
		for (int i = 0; i < cost_vals.size(); ++i) {
		  const auto manu_info = frses_to_search.at(i);
			const double cost = cost_vals.at(i);
			const bool was_successful = cost < kBigCost;
			const bool manu_is_dir = IsDir(manu_info.manu_type_);
			if (not was_successful or not manu_is_dir) {
				continue;
			}
			const double dh = delta_h_vals.at(i);
			if ((not min_cost_is_dir) or (dh < curr_min_cost)) {
			  min_cost_idx = i;
			  min_cost_is_dir = true;
				curr_min_cost = dh;
			}
		}
	} else if ((not min_cost_is_lan) and ((not initial_min_cost_close_enough) or ini_min_cost_close_but_hi_spd_dir) and (init_speed_high_enough_for_lan or wpt_is_close_n_diff_lane)) {
		std::cout << "[LAN] INITIAL VAL NOT CLOSE ENOUGH" << std::endl;
		for (int i = 0; i < cost_vals.size(); ++i) {
		  const auto manu_info = frses_to_search.at(i);
			const double cost = cost_vals.at(i);
			const bool was_successful = cost < kBigCost;
			const bool manu_is_lan = IsLan(manu_info.manu_type_);
			if ((not was_successful) or (not manu_is_lan)) {
				continue;
			}
			const double dy = delta_y_vals.at(i);
			std::cout << "[LAN] [DY]: " << dy << std::endl;
			if (std::abs(dy) < 7.0) {
			//if (true /* and std::abs(dy) > 1.5 */ ) {
				std::cout << "[LAN] [Choosing DY]: " << dy << std::endl;
				if ((not min_cost_is_lan) or (cost < curr_min_cost)) {
			    min_cost_idx = i;
				  min_cost_is_lan = true;
					curr_min_cost = cost;
				}
			}
		}
	}
	}

  const auto frs_min_info = frses_to_search.at(min_cost_idx);
  const int t0_idx_c_style = frs_min_info.idx1_;
  const bool was_successful = successful_runs > 0;
  const auto final_manu_type = frs_min_info.manu_type_;
  const double final_param = param_vals_mirrored.at(min_cost_idx);
  const int idx0 = frs_min_info.idx0_;
  const int idx1 = frs_min_info.idx1_;
  return {was_successful, final_param, final_manu_type, idx0, idx1};
}
}  // namespace roahm


class MexFunction : public matlab::mex::Function {
 private:
  ::roahm::FrsTotal frs_;
  ::roahm::FrsTotal frs_low_;

 public:
  MexFunction() : 
        frs_{::roahm::LoadFrs("/data/car_frs.txt")},
        frs_low_{::roahm::LoadFrs("/NONE")} {}
  int i = 0;
  void operator()(matlab::mex::ArgumentList outputs,
                  matlab::mex::ArgumentList inputs) {
    if (inputs.empty()) {
      std::cerr << "Error: no inputs specified!\n";
      return;
    }
    {
      constexpr std::size_t kExpectedNumInputs = 3;
      const auto in_sz = inputs.size();
      if (in_sz != kExpectedNumInputs) {
        std::cerr << "Error: " << in_sz << " inputs provided, instead of "
                  << kExpectedNumInputs << "\n";
      }
    }
    const auto& state_var = inputs[0];
    ::roahm::RoverState rover_state;
    if (state_var.getType() != matlab::data::ArrayType::DOUBLE) {
      std::cerr << "Error: state type is not DOUBLE!\n";
      return;
    }
    {
      constexpr std::size_t kExpectedNumElts = 6;
      const auto num_state_elts = state_var.getNumberOfElements();
      if (num_state_elts != kExpectedNumElts) {
        std::cerr << "Error: state has " << num_state_elts
                  << " components, instead of " << kExpectedNumElts << "\n";
        return;
      }

      const auto dims = state_var.getDimensions();
      if (dims.empty()) {
        std::cerr << "Error: state has empty dimension list!\n";
        return;
      }
      if (dims.at(0) != kExpectedNumElts) {
        std::cerr << "Error: state has " << dims.at(0)
                  << " dimensions, instead of " << kExpectedNumElts << "\n";
        return;
      }
      const double x = state_var[0];
      const double y = state_var[1];
      const double h = state_var[2];
      const double u = state_var[3];
      const double v = state_var[4];
      const double r = state_var[5];
      const double w = u;
      rover_state = ::roahm::RoverState{x, y, h, u, v, r, w};
    }
    const auto& wp_var = inputs[1];
    ::roahm::PointXYH x_des_global{};
    if (wp_var.getType() != matlab::data::ArrayType::DOUBLE) {
      std::cerr << "Error: waypoint type is not DOUBLE!\n";
      return;
    }
    {
      constexpr std::size_t kExpectedNumElts = 3;
      const auto num_wp_elts = wp_var.getNumberOfElements();
      if (num_wp_elts != kExpectedNumElts) {
        std::cerr << "Error: wp has " << num_wp_elts
                  << " components, instead of " << kExpectedNumElts << "\n";
        return;
      }

      const auto dims = wp_var.getDimensions();
      if (dims.empty()) {
        std::cerr << "Error: waypoint has empty dimension list!\n";
        return;
      }
      const auto num_dim_0 = dims.at(0);
      if (num_dim_0 != kExpectedNumElts) {
        std::cerr << "Error: waypoint has " << num_dim_0
                  << " dimensions, instead of " << kExpectedNumElts << "\n";
        return;
      }
      x_des_global = ::roahm::PointXYH{wp_var[0], wp_var[1], wp_var[2]};
    }

    const auto& obs_var = inputs[2];
    ::roahm::ObsInfo global_obs_info{};
    {
      const auto dims = obs_var.getDimensions();
      constexpr std::size_t kExpectedNumTimes = 150;
      if (dims.empty()) {
        std::cerr << "Error: no obstacle dimensions provided\n";
        return;
      }
      {
        const std::size_t kExpectedNumDims = 2;
        const auto num_dims = dims.size();
        if (num_dims != kExpectedNumDims) {
          std::cerr << "Error: obstacle set has " << num_dims
                    << " dimensions, instead of expected " << kExpectedNumDims
                    << "\n";
          return;
        }
      }

      constexpr std::size_t kExpectedDim0 =
          6;  // x, y, theta, velocity, length, width
      constexpr std::size_t kExpectedMinDim1 =
          1;  // should be at least one obstacle(TODO)
      const auto d0 = dims[0];
      const auto d1 = dims[1];
      if (d0 != kExpectedDim0) {
        std::cerr << "Error: obstacle dimension 0 (0-idx), " << d0
                  << " != " << kExpectedDim0 << "\n";
        return;
      }
      if (d1 < kExpectedMinDim1) {
        std::cerr << "Error: obstacle dimension 1 (0-idx), " << d1 << " < "
                  << kExpectedMinDim1 << "\n";
        return;
      }
			std::cout << "[DBGOBS] Num Obs: " << d1 << std::endl;
      for (int obs_idx = 0; obs_idx < d1; ++obs_idx) {
        const double x0 = obs_var[0][obs_idx];
        const double y0 = obs_var[1][obs_idx];
        const double h0 = obs_var[2][obs_idx];
        const double vel = obs_var[3][obs_idx];
        const double len = obs_var[4][obs_idx];
        const double wid = obs_var[5][obs_idx];
        ::roahm::DynObs dyn_obs{x0, y0, h0, vel, len, wid};
        global_obs_info.PushObs(dyn_obs);
        std::cout << dyn_obs << std::endl;
      }
    }
    {
      constexpr std::size_t kExpectedNumOutputs = 1;
      if (outputs.empty()) {
        std::cerr << "Error: no outputs specified!\n";
        return;
      }

      const auto num_outputs = outputs.size();
      if (num_outputs != kExpectedNumOutputs) {
        std::cerr << "Error: " << num_outputs << " outputs specified, but "
                  << kExpectedNumOutputs << "expected!\n";
        return;
      }
    }

    auto& param_output = outputs[0];
    std::cout << "Successfully passed I/O check" << std::endl;
    std::cout << "FRS: " << frs_.megas_.size() << std::endl;
    std::cout << "global_obs: " << global_obs_info.GetNumObs() << std::endl;
    auto x_des_local = x_des_global.ToLocalFrame(::roahm::PointXYH{rover_state.x_, rover_state.y_, rover_state.GetHeading()});;
    auto ret = GenerateSimParameter(
      frs_,
      global_obs_info,
      x_des_local,  // TODO make sure this is actually local
      rover_state);
    matlab::data::ArrayFactory factory;
    
    const int matlab_manu_type = ret.successful_ ? (IsSpd(ret.manu_type_) ? 1 : (IsDir(ret.manu_type_) ? 2 : 3)) : -1;
    const double param_val = ret.res_;
    const int t0_idx = 1;
    const double au_val = IsSpd(ret.manu_type_) ? param_val : rover_state.u_;
    const double ay_val = IsSpd(ret.manu_type_) ? 0.0 : param_val;

    std::cout << "Was Successful: " << ret.successful_ << std::endl;
    std::cout << "Au Val:         " << au_val << std::endl;
    std::cout << "Ay Val:         " << ay_val << std::endl;
    std::cout << "Param Val:      " << param_val << std::endl;
    std::cout << "Manu Type: " << matlab_manu_type << std::endl;
    std::cout << "| Speed: " << IsSpd(ret.manu_type_) << std::endl;
    std::cout << "| Dir:   " << IsDir(ret.manu_type_) << std::endl;
    std::cout << "| Lan:   " << IsLan(ret.manu_type_) << std::endl;


    param_output = factory.createArray<double>({6, 1}, {au_val, ay_val, t0_idx, matlab_manu_type, ret.idx0_, ret.idx1_});
    std::cout << "Ran Simulation" << std::endl;
  }
};

// int main(int argc, char** argv) {
//}

#undef ROS_INFO_STR
