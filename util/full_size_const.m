%% full-size vehicle parameters
m = 1575;    
lf = 1.13;
lr = 1.67;
Izz = 3273; 
rw = 0.33;
lambda_cri = 0.15;
alpha_cri = 0.12;
mu_bar = 10;
Caf1 = 1.716e5;  
Car1 = 2.9028e5;
u_cri = 5;
L = 4.8;
W = 2.2;

%% control parameters
Kr = 2;
Ku = 4;
Kh = 5;
kappaPU = 1.3; % kappa_1,u 
kappaIU = 0.7; % kappa_2,u
phiPU = 1.3;   % phi_1,u
phiIU = 0.7;   % phi_2,u
kappaP = 0.5;  % kappa_1,r 
kappaI = 1;    % kappa_2,r
phiP = 4;      % phi_1,r
phiI = 1;      % phi_2,r
max_Fx_uncertainty  = 393;
Mu = max_Fx_uncertainty / m;
b_u_pro = Mu/5;
b_u_off = 0;
max_Fy_uncertainty = 10;
Mr = lr*(max_Fy_uncertainty)/Izz;


%% auxiliaries 
grav_const = 9.8;
amax = 5; % a^{dec}: deceleration for braking maneuver
tpk = 6;  % time duration of lane change maneuver
tpk_dir = 3; % time duration of direction change and speed change maneuvers
u_really_slow =5; % u^{cri}: critical speed that decides the guard of vehicle's hybrid system
top_speed = 30; % maximum speed of the vehicle
car_length = L/2; % footprint of the vehicle can be expressed as interval([-car_length; -car_width], [car_length; car_width])
car_width  = W/2;


tbrk2 =1;
Cw = 3.41;
Caf2 = 1;
Car2 = 1;     
% Cbf  = 1e5;
% Cbr  = 1e5;
% C_delta = 3;
% K_delta = 5; 
% Kw = 20;

% %Current Control Stuff
% C_friction_1 = 6.0897; % NEVER USE CURRENT CONTROL FOR CARS
% C_friction_2 = 10.5921;
% C_current    = 0.4642;

% max_Fy_uncertainty = 10; 
max_Fy_uncertainty_spd = 2; 
% max_Fyr_uncertainty = 10;
% max_Fyr_uncertainty_braking = 0.5; 
% max_Fyr_uncertainty_spd = 2; 
max_Fx_uncertainty_braking  = 0;
max_Fy_uncertainty_braking = 0.5;


