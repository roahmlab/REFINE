# REFINE
The low level controller is located in `low-level-controller/src/rover_controller/include/fl_llc.h`

The online parameter generation is located in `online-parameter-generation/src/rover_controller/src/param_gen.cc`

We summarize several components of the rover experiment here for interested readers. 

## 1. Environment Sensing

We perform simultaneous localization and mapping (SLAM) using [Cartographer](https://ieeexplore.ieee.org/abstract/document/7487258) at a rate of 200Hz. Cartographer is a lidar based 2D SLAM algorithm that consists of a local sub-system, which builds locally consistent and successive submaps, and a global sub-system, which runs in background to achieve loop closure. Lidar scans are also used for obstacle detection at a rate of 10Hz using the method illustrated in [this paper](https://ieeexplore.ieee.org/document/8003904), which detects an object in the environment by multiple line segments. To account for estimation error as discussed in that paper, we inflate the detected line segments and convert them into zonotopes for online planning.

## 2. System Identification of Tire Models

The goal of system identification is to specify necessary parameters that describe dynamics of the Rover.
Because parameters like mass, length, and moment of inertia can be directly measured, we focus on identification of tire force related parameters including $\lambda^{\text{cri}}$, $\alpha^{\text{cri}}$, $\bar\mu$, $\bar{c}_{\alpha\text{f}}$ and $\bar{c}_{\alpha\text{r}}$, and explain how we generate the computational error $\Delta_u$, $\Delta_v$, $\Delta_r$ in \eqref{eq:highspeed_noise} as well as their bounding parameters $\bpro$, $\boff$, $M_u$, $M_v$, and $M_r$ in the next subsection.


## 3. Model Parameters 

## Controller Performance

