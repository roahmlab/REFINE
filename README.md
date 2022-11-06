# REFINE: REachability-based trajectory design using robust Feedback lInearization and zoNotopEs

REFINE is a safety real-time trajectory planning framework for autonomous driving. 
REFINE utilizes a parametrized robust controller that partially linearizes the vehicle dynamics in the presence of modeling error. 
Offline zonotope-based reachability analysis is performed on the closed-loop, full order vheicle dynamics to compute the corresponding control-parametrized, over-approximate Forward Reachable Sets (FRS). 
Real-time trajectory planning is achieved by solving an optimization framework in real-time with the pre-computed, control-parametrized FRS being used to ensure vehicle safety.

<p align="center">
  <img height="300" src="/Image/overview.png"/>
</p>

# Installation Requirements
REFINE is built on Ubuntu 20.04 with ROS Noetic Distribution, and the algorithms are implemented in MATLAB and C++17. 
REFINE has the following required dependencies:
- [Docker](https://www.docker.com/) to download simulation package.
- [CORA 2018](https://tumcps.github.io/CORA/) for Forward Reachable Sets representation and computation.

# Overview
## 0. Installation
- Run dockerfile (need to modify this line once Lucas uploads the file).
- Lunch docker (need to modify this line once Lucas uploads the file).
- Run MATLAB inside docker (need to modify this line once Lucas uploads the file).
- Add required toolboxes in the search path of MATLAB.
- Run [`install.m`](https://github.com/jinsunl/REFINE/blob/main/install.m).
- In [`split.m`](https://github.com/jinsunl/REFINE/blob/main/split.m), replace line 20 with ```cd(your_matlab_directory/toolbox/matlab/strfun)``` and line 22 with ```cd('your_CORA2018_directory/global functions/globOptimization')``` (need to modify this line once Lucas uploads the file).


## 1. Offline reachability analysis
Vehicle behavior is offline computed in REFINE via reachability analysis using zonotopes. 
See [Offline_Reachability_Analysis](https://github.com/jinsunl/REFINE/tree/main/Offline_Reachability_Analysis) for detail.


## 2. Simulation
REFINE is evaluated in simulation on a full-size Front-Wheel-Drive vehicle model.
See [Full_Size_Vehicle_Simulation](https://github.com/jinsunl/REFINE/tree/main/Full_Size_Vehicle_Simulation) for detail.
Simulation video can be found [here](https://drive.google.com/drive/folders/1bXl07gTnaA3rJBl7J05SL0tsfIJEDfKy?usp=sharing).



## 3. Hardware Implementation
REFINE is evaluated in real hardware testing on a All-Wheel-Drive 1/10th race car robot.
See [Rover_Robot_Implementation](https://github.com/jinsunl/REFINE/tree/main/Rover_Robot_Implementation) for detail. 
Hardware Demo can be found [here](https://drive.google.com/drive/folders/1FvGHuqIRQpDS5xWRgB30h7exmGTjRyel?usp=sharing).



