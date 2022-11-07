#ifndef ROAHM_TRAJECTORY_VALUES_
#define ROAHM_TRAJECTORY_VALUES_

//
// Add functions here for different sets of trajectories
// E.g. maybe a GetDemoTrajectories() function for running demo.
// These should be called from rover_controller.
//

#include <algorithm>
#include <vector>

#include "trajectory_sym.h"

namespace roahm {
std::vector<Trajectory> GetDefaultTrajectories() {
  const auto lan_change = RManuType::kLanChange;
  const auto dir_change = RManuType::kDirChange;

  std::vector<Trajectory> trajectories;
  std::vector<double> ay_vals(10);  // [-0.6, 0.6, 0.1]
  std::vector<double> au_vals(5);   // [0.2, 1.2, 0.2]
  std::vector<double> u0_vals(5);   // [0.2, 1.2, 0.2]

  //LinspaceFill(ay_vals, -0.8, 1.0);
  //LinspaceFill(au_vals, 0.8, 2.3);
  //LinspaceFill(u0_vals, 0.8, 2.3);
  std::reverse(au_vals.begin(), au_vals.end());
  std::vector<RManuType> manu_types{lan_change, dir_change};

  double au_simon = 1.5;
  double ay_simon = 1.0;

  double init_speed = 0.55;
  double goal_speed = 1.2;
  trajectories.emplace_back(1.5, 0.8, 1.5, lan_change);
   // trajectories.emplace_back(goal_speed, -0.4, init_speed, dir_change);
  // trajectories.emplace_back(goal_speed, -0.3, init_speed, dir_change);
  // trajectories.emplace_back(goal_speed, -0.2, init_speed, dir_change);
  // trajectories.emplace_back(goal_speed, -0.1, init_speed, dir_change);
  // trajectories.emplace_back(goal_speed, 0.0, init_speed, dir_change);
  // trajectories.emplace_back(goal_speed, 0.1, init_speed, dir_change);
  // trajectories.emplace_back(goal_speed, 0.2, init_speed, dir_change);
  // trajectories.emplace_back(goal_speed, 0.3, init_speed, dir_change);
  // trajectories.emplace_back(goal_speed, 0.4, init_speed, dir_change);






  // for (auto au : au_vals) {
  //   // trajectories.emplace_back(au, 0.8*au*2/3, au, lan_change);
  //   // trajectories.emplace_back(au, 0.6*au*2/3, au, lan_change);
  //   // trajectories.emplace_back(au, 0.4*au*2/3, au, lan_change);
  //   // trajectories.emplace_back(au, 0.2*au*2/3, au, lan_change);
  //   // trajectories.emplace_back(au, 0.0*au*2/3, au, lan_change);
  //   // trajectories.emplace_back(au, -0.2*au*2/3, au, lan_change);
  //   // trajectories.emplace_back(au, -0.4*au*2/3, au, lan_change);
  //   // trajectories.emplace_back(au, -0.6*au*2/3, au, lan_change);
  //   // trajectories.emplace_back(au, -0.8*au*2/3, au, lan_change);
  // }

  // trajectories.emplace_back(2.0, -1.2, 2.0, lan_change);
  // trajectories.emplace_back(2.0, -1.3, 2.0, lan_change);
  // trajectories.emplace_back(2.0, -1.4, 2.0, lan_change);
  // trajectories.emplace_back(2.0, -1.5, 2.0, lan_change);
  // trajectories.emplace_back(2.0, -1.6, 2.0, lan_change);

  // //6
  // trajectories.emplace_back(1.8, -1.2, 1.8, lan_change);
  // trajectories.emplace_back(1.8, -1.3, 1.8, lan_change);
  // trajectories.emplace_back(1.8, -1.4, 1.8, lan_change);
  // trajectories.emplace_back(1.8, -1.5, 1.8, lan_change);
  // trajectories.emplace_back(1.8, -1.6, 1.8, lan_change);
  
  // //11
  // trajectories.emplace_back(1.6, -1.2, 1.6, lan_change);
  // trajectories.emplace_back(1.6, -1.3, 1.6, lan_change);
  // trajectories.emplace_back(1.6, -1.4, 1.6, lan_change);
  // trajectories.emplace_back(1.6, -1.5, 1.6, lan_change);
  // trajectories.emplace_back(1.6, -1.6, 1.6, lan_change);

  // //16
  // trajectories.emplace_back(1.4, -1.1, 1.4, lan_change);
  // trajectories.emplace_back(1.4, -1.2, 1.4, lan_change);
  // trajectories.emplace_back(1.4, -1.3, 1.4, lan_change);
  // trajectories.emplace_back(1.4, -1.4, 1.4, lan_change);


  // trajectories.emplace_back(2.2,  1.6, 2.2, lan_change);
  // trajectories.emplace_back(2.2, -1.6, 2.2, lan_change);
  // trajectories.emplace_back(2.5,  1.7, 2.5, lan_change);
  // trajectories.emplace_back(2.5, -1.7, 2.5, lan_change);

  return trajectories;
};
}  // namespace roahm

#endif  // ROAHM_TRAJECTORY_VALUES_
