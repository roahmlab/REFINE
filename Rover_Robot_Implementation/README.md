# REFINE
The low level controller is located in `low-level-controller/src/rover_controller/include/fl_llc.h`

The online parameter generation is located in `online-parameter-generation/src/rover_controller/src/param_gen.cc`

We summarize several components of the rover experiment here for interested readers. Note references to equations correspond to equation numbers from the paper.

## 1. Desired Trajectories 

An illustration of the speed change maneuvers described by (57), (58), and (60) can be found in Fig R.1.

<figure>
<p align="center">
  <img height="150" src="../Image/udes.png"/>
   <figcaption> <i> Fig. R.1 -  Examples of $\udes(t,p)$ with $u_0=1.0$ [m/s], $\uc=0.5$ [m/s], $\tnb=1.5$ [s], $\amax=-1.5$ [m/s${}^2$], and $p_u$ taking values of 0.6, 1.2 and 2.0 from top to bottom. Note zero lateral control is commanded in all 3 examples. </i> </figcaption>
</p>
 </figure>

An illustration of the direction change maneuvers described by (57), (58), and (61) can be found in Fig R.2. #real-cool-heading

<figure>
<p align="center">
  <img height="150" src="../Image/des_dir_change.png"/>
   <figcaption> <i> Fig. R.2 -  Examples of $\hdes(t,p)$ and $\rdes(t,p)$ to achieve direction changes with $u_0=1.0$ [m/s], $\tnb=1.5$ [s], $\hdes_1=\frac{20}{27}$, $\hdes_2=\frac{27}{10}$, and $p_y$ taking values of -0.4, 0.4 and 0.8 from top to bottom. Note $p_u$ is set as $u_0$ to maintain the vehicle longitudinal speed before $\tnb$ among all 3 examples. </i> </figcaption>
</p>
 </figure>

## 2. Environment Sensing

We perform simultaneous localization and mapping (SLAM) using [Cartographer](https://ieeexplore.ieee.org/abstract/document/7487258) at a rate of 200Hz. Cartographer is a lidar based 2D SLAM algorithm that consists of a local sub-system, which builds locally consistent and successive submaps, and a global sub-system, which runs in background to achieve loop closure. Lidar scans are also used for obstacle detection at a rate of 10Hz using the method illustrated in [this paper](https://ieeexplore.ieee.org/document/8003904), which detects an object in the environment by multiple line segments. To account for estimation error as discussed in that paper, we inflate the detected line segments and convert them into zonotopes for online planning.

## 3. System Identification of Tire Models

The goal of system identification is to specify necessary parameters that describe dynamics of the Rover.
Because parameters like mass, length, and moment of inertia can be directly measured, we focus on identification of tire force related parameters including $\lambda^{\text{cri}}$, $\alpha^{\text{cri}}$, $\bar{\mu}$, $\bar{c}_ {\alpha\text{f}}$ and $\bar{c} _ {\alpha\text{r}}$, and explain how we generate the computational error $\Delta_u$, $\Delta_v$, $\Delta_r$ in (14) as well as their bounding parameters $b^{\text{pro}} _ u$, $b^{\text{off}} _ u$, $M_u$, $M_v$, and $M_r$ in the next subsection. Note this system identification is done by using a motion capture system; however, when REFINE is applied, the motion capture system is not used.

Recall the actual tire models in (6),(7), (10), and (11) become saturated at large slip ratios and slip angles. However, during experiments, the Rover is always expected to operate in linear regimes of tires by Assumption 3. Thus, to determine the tire force-related parameters of the Rover, we need to identify the critical slip ratio and critical slip angle at which tire force saturation begins, then fit linear tire models within the linear regimes. To identify the parameters related to longitudinal tire forces, the Rover executed a series of speed change maneuvers in a motion capture system to estimate $u$, $v$, and $r$. $\dot u$ is estimated using the onboard IMU.  Recall the ideal dynamics of longitudinal speed as in (4). By plugging in the speed information from the motion capture system, we generate the longitudinal tire force $F_x(t):=F_{x\text{f}}(t)+F_{x\text{r}}(t)$ that achieves the observed velocity trajectory. Because the Rover is AWD, both the front and the rear tires have the same tire speed and thus the same slip ratio, i.e. $\lambda_\text{f} = \lambda_\text{r}$.  Adding the two equations in (12) results in $F_x(t) = mg\bar\mu\lambda_\text{i}(t)$
where the subscript "i" can be replaced by either "f" for front tire or "r" for rear tire. Using the information from the encoder of driving motor and $u(t)$, slip ratios of both tires can be computed via (5). As shown in the Fig. R.3, the longitudinal tire force saturates when the slip ratio becomes bigger than $0.45$.

<figure>
<p align="center">
  <img height="150" src="../Image/slip_ratio.png"/>
   <figcaption> <i> Fig. R.3 -  System Identification on longitudinal tire force. A linear model is fit to data collected within the linear regime $[-0.45,0.45]$ of slip ratio. </i> </figcaption>
</p>
 </figure>

Thus we set $\lambda^{\text{cri}}=0.45$ and fit $\bar\mu$ from $F_x(t) = mg\bar\mu\lambda_\text{i}(t)$ by performing least squares over collected data that satisfies $|\lambda_\text{i}(t)|\leq\lambda^{\text{cri}}$ at any time.  

To identify the parameters related to lateral tire forces, we follow a similar procedure and let the Rover execute a series of direction change maneuvers with various longitudinal speeds. Ground truth $u$, $v$, and $r$ are again estimated using the motion capture system, and $\dot v$ and $\dot r$ are estimated using the onboard IMU for all time. Recall when $u(t)> u ^{\text{cri}}$, the error-free dynamics of $v$ and $r$ are as in (4). One can then compute $F_\text{yf}(t)$ and $F_\text{yr}(t)$ by using the relevant components of \eqref{eq:highspeed_perfect} for $F_\text{yf}(t)$ and $F_\text{yr}(t)$. Using $u(t)$, $v(t)$, $r(t)$, and the steering motor input, one can compute slip angles for both tires via (8) and (9). As shown in Fig. R.4, the lateral tire force saturates when the slip angle becomes bigger than 0.15.

<figure>
<p align="center">
  <img height="300" src="../Image/slip_angle.png"/>
  <figcaption> <i> Fig. R.4 - System identification on lateral tire forces. Linear models are fit using data collected within the linear regime $[-0.15,0.15]$ of slip angles. </i> </figcaption>
</p>
 </figure>

Thus we set $\alpha^{\text{cri}}=0.15$, and fit $\bar c_{\alpha\text{f}}$ and $\bar c_{\alpha\text{r}}$ in (13) by performing least squares over collected data that satisfies $|\alpha_\text{i}(t)|\leq\alpha^{\text{cri}}$ at any time.   

## 4. Model Parameters

Results of system identification are summarized in Fig. R.5 together with controller gains that are chosen to satisfy the conditions in Lemma 14.

<figure>
<p align="center">
  <img height="300" src="../Image/rover_params.png"/>
  <figcaption> <i> Fig. R.5 - Rover model parameters and REFINE controller parameters. </i> </figcaption>
</p>
 </figure>
 
## 5. Controller Performance

An example of tracking performance of the proposed controller on the Rover with identified parameters is shown in Fig. R.6.

<figure>
<p align="center">
  <img height="500" src="../Image/tracking2.png"/>
  <figcaption> <i> Fig. R.6 - An illustration of the tracking performance of the Rover with the proposed controller and identified parameters in Fig. R.5. </i> </figcaption>
</p>
 </figure>
