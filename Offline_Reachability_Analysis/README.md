# REFINE: Offline Reachability Analysis

**Note:** offline reachability analysis can also be achieved without the provided docker image being launched.

REFINE utilizes 3 families of desired trajectories that are observed during daily driving to achieve *speed change*, *direction change* and *lane change*. 
Each desired trajectory is parametrized by control parameter `p=[p_u,p_y]` where `p_u` denotes desired longitudinal speed and `p_y` decides desired lateral displacement. 
To perform offline reachability analysis using such desired trajectories:

- Run [`space_subdivision.m`](https://github.com/jinsunl/REFINE/blob/main/Offline_Reachability_Analysis/space_subdivision.m) to compute subdivision of the initial condition space and contrl parameter space for simulation on a full-size Frond-Wheel-Drive vehicle. 
(todo: add corresponding file for rover FRS computation)

- Run [`vehicle_dynamics_generation.m`](https://github.com/jinsunl/REFINE/blob/main/Offline_Reachability_Analysis/vehicle_dynamics_generation.m)
to generate closed-loop vehicle dynamics with proposed controller over 3 families of desired trajectories.

- Run [`FRS_computation_speed_change.m`](https://github.com/jinsunl/REFINE/blob/main/Offline_Reachability_Analysis/FRS_computation_speed_change.m) to compute FRS for speed change maneuvers.

- Run [`FRS_computation_direction_change.m`](https://github.com/jinsunl/REFINE/blob/main/Offline_Reachability_Analysis/FRS_computation_direction_change.m) to compute FRS for direction change maneuvers.

- Run [`FRS_computation_lane_change.m`](https://github.com/jinsunl/REFINE/blob/main/Offline_Reachability_Analysis/FRS_computation_lane_change.m) to compute FRS for lane change maneuvers.

- Run [`wrap_up_FRS.m`](https://github.com/jinsunl/REFINE/blob/main/Offline_Reachability_Analysis/wrap_up_FRS.m) to wrap up all computed FRS and necessary information as a map container that is used for online planning.
Resulted map container is provided as [`car_frs.mat`](https://drive.google.com/drive/folders/1WZbFFhCyhYQlMJxuV4caIzNoa-Q9VZkW?usp=sharing).



