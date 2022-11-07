#ifndef ROAHM_TRAJECTORY_VALUES_HPP_
#define ROAHM_TRAJECTORY_VALUES_HPP_

#include <algorithm>
#include <vector>

#include "trajectory_sym.hpp"

/// @file trajectory_values.hpp Contains a function to generate a list of
/// trajectories to execute for system identification

// TODO could add functions for different sets of trajectories

namespace roahm {
namespace {
/// Fills a container with linearly spaced values
/// For containers with one element, the maximum value is used.
/// For containers with more than one element, they are filled as follows:
/// \f$
/// \frac{ \begin{bmatrix}0 & 1 & \cdots & N\end{bmatrix} }{N+1}
/// (\textrm{max} - \textrm{min}) + \textrm{min}
/// \f$
/// \tparam T the container type
/// \param container the container to fill
/// \param min_v the minimum value
/// \param max_v the maximum value
template <typename T>
void LinspaceFill(T& container, double min_v, double max_v) {
  if (container.size() <= 0) {
    return;
  } else if (container.size() == 1) {
    container[0] = max_v;
    return;
  } else {
    const double rng = max_v - min_v;
    for (std::size_t i = 0; i < container.size(); ++i) {
      container[i] =
          ((static_cast<double>(i) / container.size()) * rng) + min_v;
    }
  }
}
}  // namespace
std::vector<Trajectory> GetDefaultTrajectories() {
  const auto lan_change = ManuType::kLanChange;
  const auto dir_change = ManuType::kDirChange;

  std::vector<Trajectory> trajectories;
  std::vector<double> ay_vals(10);  // [-0.6, 0.6, 0.1]
  std::vector<double> au_vals(5);   // [0.2, 1.2, 0.2]
  std::vector<double> u0_vals(5);   // [0.2, 1.2, 0.2]

  LinspaceFill(ay_vals, -0.8, 1.0);
  LinspaceFill(au_vals, 0.8, 2.3);
  LinspaceFill(u0_vals, 0.8, 2.3);
  std::reverse(au_vals.begin(), au_vals.end());
  std::vector<ManuType> manu_types{lan_change, dir_change};

  // double au_simon = 1.5;
  // double ay_simon = 1.0;

  // for (auto manu_type : manu_types) {
  //   trajectories.emplace_back(0.6, 0.5, 0.6, manu_type);
  //   trajectories.emplace_back(1.0, 0.7, 1.0, manu_type);
  //   trajectories.emplace_back(1.2, 0.8, 1.2, manu_type);
  //   trajectories.emplace_back(1.5, 1.1, 1.5, manu_type);
  //   trajectories.emplace_back(2.0, 1.4, 2.0, manu_type);
  // }

  // trajectories.emplace_back(1.0, 0.5, 0.8, lan_change);
  // for (auto au : au_vals) {
  //   for (auto ay : ay_vals) {
  //     trajectories.emplace_back(au, 0.0, 0.0, 0.0,
  //                                       dir_change);
  //     for (auto manu_type : manu_types) {
  //       trajectories.emplace_back(0.6, 0.5, 0.6, manu_type);
  //       trajectories.emplace_back(1.0, 0.7, 1.0, manu_type);
  //       trajectories.emplace_back(1.2, 0.8, 1.2, manu_type);
  //       trajectories.emplace_back(1.5, 1.1, 1.5, manu_type);
  //       trajectories.emplace_back(2.0, 1.4, 2.0, manu_type);
  //     }
  //   }
  // }

  // for (auto au : au_vals) {
  //   trajectories.emplace_back(au, 0.8, au, lan_change);
  //   trajectories.emplace_back(au, 0.8, au, dir_change);
  //   trajectories.emplace_back(au, 0.6, au, lan_change);
  //   trajectories.emplace_back(au, 0.6, au, dir_change);
  //   trajectories.emplace_back(au, 0.4, au, lan_change);
  //   trajectories.emplace_back(au, 0.4, au, dir_change);
  //   trajectories.emplace_back(au, 0.2, au, lan_change);
  //   trajectories.emplace_back(au, 0.2, au, dir_change);
  //   trajectories.emplace_back(au, 0.0, au, lan_change);
  //   trajectories.emplace_back(au, 0.0, au, dir_change);
  //   trajectories.emplace_back(au, -0.2, au, lan_change);
  //   trajectories.emplace_back(au, -0.2, au, dir_change);
  //   trajectories.emplace_back(au, -0.4, au, lan_change);
  //   trajectories.emplace_back(au, -0.4, au, dir_change);
  //   trajectories.emplace_back(au, -0.6, au, lan_change);
  //   trajectories.emplace_back(au, -0.6, au, dir_change);
  //   trajectories.emplace_back(au, -0.8, au, lan_change);
  //   trajectories.emplace_back(au, -0.8, au, dir_change);
  // }

  // for (auto au : au_vals) {
  //   for (auto u0 : u0_vals) {
  //     trajectories.emplace_back(au, 0.0, u0, dir_change);
  //   }
  // }

  // for (auto ay : ay_vals) {
  //   for (auto manu_type : manu_types) {
  //     const double u0 = 1.0;
  //     const double au = u0;
  //     trajectories.emplace_back(au, ay, u0, manu_type);
  //   }
  // }

  // for (auto manu_type : manu_types) {
  //   const double ay = 0.6;
  //   const double u0 = 1.0;
  //   const double au = u0;
  //   trajectories.emplace_back(au, ay, u0, manu_type);
  // }

  // const double u0 = 1.0;
  // const double au = u0;
  // trajectories.emplace_back(au, 0.6, u0, lan_change);
  // trajectories.emplace_back(au, -0.6, u0, dir_change);
  // trajectories.emplace_back(au, -0.6, u0, lan_change);
  // // trajectories.emplace_back(au, 0.6, u0,
  // //                                  dir_change);

  // const double u0 = 1.0;
  // const double au = u0;
  // trajectories.emplace_back(au, 0.6, u0, lan_change);
  // trajectories.emplace_back(au, 0.0, u0, dir_change);
  // trajectories.emplace_back(au, 0.0, u0, dir_change);
  // const double u0 = 1.0;
  // const double au = u0;
  // const double ay = 0.6;
  // trajectories.emplace_back(au, ay, u0, lan_change);
  // trajectories.emplace_back(au, -ay, u0, lan_change);
  // // for videos
  // const double u0 = 1.0;
  // const double au = u0;
  // const double ay0 = 0.6;
  // const double ay1 = 0.4;
  // trajectories.emplace_back(au, ay0, u0, dir_change);
  // trajectories.emplace_back(au, -ay0, u0, dir_change);
  // trajectories.emplace_back(au, -ay1, u0, dir_change);
  // trajectories.emplace_back(au, ay1, u0, dir_change);
  // // for videos
  // const double u0 = 2.0;
  // const double au = u0;
  // const double ay0 = 0.8;
  // const double ay1 = 0.8;
  // const double u0_low = 1.0;
  // trajectories.emplace_back(u0_low, ay0, u0_low, dir_change);
  // trajectories.emplace_back(u0_low, -ay0, u0_low, dir_change);

  // trajectories.emplace_back(au, ay0, u0, dir_change);
  // trajectories.emplace_back(au, -ay0, u0, dir_change);
  // trajectories.emplace_back(au, ay1, u0, dir_change);
  // trajectories.emplace_back(au, ay1, u0, lan_change);
  // trajectories.emplace_back(au, -ay1, u0, dir_change);
  // trajectories.emplace_back(au, -ay1, u0, lan_change);
  return trajectories;
};
}  // namespace roahm

#endif  // ROAHM_TRAJECTORY_VALUES_HPP_
