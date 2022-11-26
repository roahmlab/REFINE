%% rover parameter
m = 4.9565;    
lf = 0.203;  
lr = 0.107;
Izz = 0.11;
rw = 0.055;
lambda_cri = 0.45;
alpha_cri = 0.165;
mu_bar = 0.77;
Caf1 = 36.24;  
Car1 = 63.52;
u_cri = 0.5;
L = 0.52;
W = 0.28;

%% control parameters
Kr = 8;
Ku = 4;
Kh = 5;
kappaPU = 1.2; % kappa_1,u 
kappaIU = 0.8; % kappa_2,u
phiPU = 1.2;   % phi_1,u
phiIU = 0.8;   % phi_2,u
kappaP = 0.9;  % kappa_1,r 
kappaI = 0.6;    % kappa_2,r
phiP = 0.9;      % phi_1,r
phiI = 0.6;      % phi_2,r
max_Fx_uncertainty  = 5.45;
Mu = max_Fx_uncertainty / m;
b_u_pro = 1.2;
b_u_off = 0.51;
max_Fy_uncertainty = 1.06;
Mr = lr*(max_Fy_uncertainty)/Izz;


%% auxiliaries 
grav_const = 9.8;
amax = 1.5; % a^{dec}: deceleration for braking maneuver
tpk = 3;  % time duration of lane change maneuver
tpk_dir = 1.5; % time duration of direction change and speed change maneuvers
u_really_slow = 0.5; % u^{cri}: critical speed that decides the guard of vehicle's hybrid system
top_speed = 2.0; % maximum speed of the vehicle
car_length = L/2; % footprint of the vehicle can be expressed as interval([-car_length; -car_width], [car_length; car_width])
car_width  = W/2;
tbrk2 = 1;
Cw = 3.41;
Caf2 = 1;
Car2 = 1;     

max_Fy_uncertainty_spd = 0.6; 
max_Fx_uncertainty_braking  = 0.6;
max_Fy_uncertainty_braking = 0.3;


