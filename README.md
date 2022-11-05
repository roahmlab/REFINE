# REFINE: REachability-based trajectory design using robust Feedback lInearization and zoNotopEs

REFINE is a safety real-time trajectory planning framework for autonomous driving. REFINE utilizes a parametrized robust controller that partially linearizes the vehicle dynamics in the presence of modeling error. Offline zonotope-based reachability analysis is performed on the closed-loop, full order vheicle dynamics to compute the corresponding control-parametrized, over-approximate Forward Reachable Sets (FRS). Real-time trajectory planning is achieved by solving an optimization framework in real-time with the pre-computed, controlp-parametrized being used to ensure vehicle safety.
<img height="270" src="/Image/overview.png"/>

# Installation Requirements
To run the code in this repository, you will need the following:
- MATLAB 2021b or higher
- [CORA 2018](https://tumcps.github.io/CORA/) for Forward Reachable Sets representation and computation

# Overview
## 0. Installation
- Properly install the above toolboxes and have them added in the search path of MATLAB.
- Run [`install.m`](https://github.com/jinsunl/REFINE/blob/main/install.m).
- Change split.m


## 1. Offline reachability analysis
control parameter `P = (p_u,p_y)`



## 2. 

