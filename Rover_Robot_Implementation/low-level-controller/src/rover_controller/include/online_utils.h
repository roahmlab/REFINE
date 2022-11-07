#ifndef ROAHM_ONLINE_UTILS_H_
#define ROAHM_ONLINE_UTILS_H_
#include <fmt/format.h>
#include <ros/ros.h>
#include <rover_control_msgs/RoverDebugStateStamped.h>

#include <algorithm>
#include <numeric>
#include <string>
#include <vector>

#include "IpIpoptApplication.hpp"
#include "IpSmartPtr.hpp"
#include "frs_loader.h"
#include "poly_predictor.h"
#include "rmanu_type.h"
#include "simple_util.h"

namespace roahm {

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
    // TODO
    predict_info.t0_idx_c_style_ = 0;
    // TODO
    predict_info.use_15_30_ = false;
  }
  return predict_info;
}

std::vector<FrsSelectInfo> GetParamsToSearchOver(const FrsTotal& frs,
                                                 double& state_u,
                                                 double& state_v,
                                                 double& state_r) {
  // Set of FRSes to search
  std::vector<FrsSelectInfo> frses_to_search;

  // (En/Dis)able certain manu types
  const bool search_au = true;
  const bool search_lan = true;
  const bool search_dir = true;
  const bool can_clamp_u = true;
  const auto u0_idx = frs.SelectU0Idx(state_u, can_clamp_u);

  ROS_INFO("SEARCH AU");
  auto add_frs = [&state_v, &state_r, &frses_to_search](
                     const long first_idx,
                     const long last_idx,  // [first, last)
                     const bool is_table_populated, const FrsMega& vehrs,
                     const RManuType& manu_type, std::size_t u0_idx,
                     bool is_outer, int other_idx, bool also_mirror) -> void {
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
      const long max_bin = static_cast<long>(mega_u.au_.size()) - 1;
      if (max_bin < 0) {
        continue;
      }
      const bool is_table_populated = IsPopulatedMultidim(mega_u.autb_);
      const auto manu_type = RManuType::kSpdChange;
      const long num_au_above = 2;
      const long num_au_below = num_au_above;
      const auto first_idx = Clamp(u0_idx - num_au_below, 0, max_bin + 1);
      const auto last_idx = Clamp(u0_idx + num_au_above, 0, max_bin + 1);
      // [first, last)
      add_frs(first_idx, last_idx, is_table_populated, mega_u, manu_type,
              au_u0_idx, true, 0, false);
    }
  }

  const auto& mega_u = frs.megas_.at(u0_idx);
  auto add_dir_lan_frs = [&mega_u, &add_frs,
                          &u0_idx](const RManuType manu_type) {
    const auto& manu_set = mega_u.GetDirLanSet(manu_type);
    const bool is_table_populated = mega_u.IsTablePopulated(manu_type);
    for (int i = 0; i < manu_set.size(); ++i) {
      const int num_search = Min(1, manu_set.at(i).size());  // TODO Remove Min
      add_frs(0, num_search, is_table_populated, mega_u, manu_type, u0_idx,
              false, i, true);
    }
  };
  if (search_lan) {
    ROS_INFO("SEARCH LAN");
    add_dir_lan_frs(RManuType::kLanChange);
  }
  if (search_dir) {
    ROS_INFO_STREAM("SEARCH DIR");
    add_dir_lan_frs(RManuType::kDirChange);
  }
  return frses_to_search;
}

std::vector<Ipopt::SmartPtr<Ipopt::IpoptApplication>> GetIpApps(int num_apps) {
  std::vector<Ipopt::SmartPtr<Ipopt::IpoptApplication>> app_vec;
  app_vec.reserve(num_apps);
  for (int i = 0; i < num_apps; ++i) {
    app_vec.push_back(IpoptApplicationFactory());
  }
  for (auto& app : app_vec) {
    app->Options()->SetNumericValue("tol", 1.0e-3);
    // app->Options()->SetStringValue("print_timing_statistics", "yes");
    app->Options()->SetStringValue("hessian_constant", "yes");
    app->Options()->SetStringValue("linear_solver", "ma57");
    app->Options()->SetNumericValue("ma57_pre_alloc", 5.0);
    app->Options()->SetIntegerValue("print_level", 0);
    app->Options()->SetIntegerValue("max_iter", 15);
    app->Options()->SetIntegerValue("acceptable_iter", 15);     // def 15
    app->Options()->SetNumericValue("acceptable_tol", 1.0e-3);  // def 1.0e-6
    // app->Options()->SetNumericValue("acceptable_dual_inf_tol", 1.0e+10); //
    // def 1.0e+10 app->Options()->SetNumericValue("acceptable_constr_viol_tol",
    // 0.01); // def 0.01
    // app->Options()->SetNumericValue("acceptable_compl_inf_tol", 0.01); // def
    // 0.01 app->Options()->SetNumericValue("acceptable_obj_tol", 0.1); // def
    // 0.01
  }
  return app_vec;
}

std::string PrintMMMS(const std::vector<double>& v, std::string name,
                      std::string unit, double multiplier) {
  const double min_v =
      v.empty() ? 0.0 : multiplier * (*std::min_element(v.begin(), v.end()));
  const double max_v =
      v.empty() ? 0.0 : multiplier * (*std::max_element(v.begin(), v.end()));
  const double sum_v =
      v.empty() ? 0.0 : multiplier * (std::accumulate(v.begin(), v.end(), 0.0));
  const auto sz_v = v.size();
  const double mean_v = v.empty() ? 0.0 : (sum_v / sz_v);
  return fmt::format(
      "{:10s} ({:3d} runs): {:15.5f} {:15.5f} {:15.5f} {:15.5f} {:5s}\n", name,
      sz_v, mean_v, min_v, max_v, sum_v, unit);
  // ROS_INFO_STREAM(name << "(" << sz_v << " runs) : Mean, Min, Max, Sum: " <<
  // mean_v
  //          << " " << min_v << " " << max_v << " " << sum_v << " (" << unit
  //          << ")");
}
std::string PrintMMMS(const std::vector<double>& v, std::string name,
                      std::string unit) {
  return PrintMMMS(v, name, unit, 1.0);
}

std::string PrintMMMSSecToMicro(const std::vector<double>& v,
                                std::string name) {
  return PrintMMMS(v, name, "us", 1.0e6);
}

}  // namespace roahm
#endif
