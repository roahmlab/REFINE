
%% DO NOT SHARE !!!
m = 1575;    
lf = 1.13;
lr = 1.67;
Izz = 3273; 
Cw = 3.41;%???
Caf1 = 1.716e5;
Caf2 = 1;  
Car1 = 2.9028e5;
Car2 = 1;     
Cbf  = 1e5;
Cbr  = 1e5;


C_delta = 3;
K_delta = 5; %should not be used, delta control gain, now using linear
% controller, so don't need this. This is still used in slow 11 states
% agent

% feedback linearization parameters
% control gains
Kr = 2;
Ku = 4;
Kh = 5;
grav_const = 9.8;
Kw = 20;

%Current Control Stuff
C_friction_1 = 6.0897; % NEVER USE CURRENT CONTROL FOR CARS
C_friction_2 = 10.5921;
C_current    = 0.4642;


amax = 5;%
tpk = 6;
% tbrk = 2;% no longer used
tpk_dir = 3;
tbrk2 =1;
u_really_slow =5;


top_speed = 22;

mu_gnd =0.15;%0.275;%


car_length = 5.2/2; % these are supposed to be generator values so they need to be half of the actual value
car_width  = 2.8/2;
obs_localize_err = 0.08;

max_Fyr_uncertainty = 1; % from Hansen Data
max_Fyr_uncertainty_braking = 0.5; % from Hansen Data
max_Fyr_uncertainty_spd = 2; % from Hansen Data
max_Fx_uncertainty  = 1;% double confirm with hansen
max_Fx_uncertainty_braking  = 0;

kappaP = 0.5; kappaI = 1;
phiP = 4; phiI = 1;

kappaPU = 1.3; kappaIU = 0.7;
phiPU = 1.3; phiIU = 0.7;


%% DO NOT SHARE !!!
% m = 1575;
% lf = 1.13;
% lr = 1.67;
% Cw = 3.41;
% %             l = lf + lr;     
% Caf1 = 1.716e+05;
% %             Cr = 2.214094963126969e+05;
% %             g = 9.80655;
% Izz = 3273; % from ram
% Car1 = 2.9028e5;
% Caf2 = 1; Car2 = 1;
% 
% % feedback linearization parameters
% %control gains
% Kr = 2;
% Ku = 4;
% Kh = 5;
% 
% amax = 3;
% tpk = 6;
% tbrk2 =1;
% u_really_slow = 1;
% tpk_dir = 3;
% % hi_low_boundary = 15;
% 
% mu_gnd =0.15;%0.275;%
% 
% % Kvy = m*lf/Ca/(lf+lr);
% 
% kappaP = 0.5; kappaI = 1;
% phiP = 4; phiI = 1;
% 
% kappaPU = 1; kappaIU = 0.8;
% phiPU = 1; phiIU = 0.8;
% 
% max_Fyr_uncertainty = 40; % from Hansen Data
% max_Fyr_uncertainty_braking = 20; % from Hansen Data
% max_Fyr_uncertainty_spd = 2; % from Hansen Data
% max_Fx_uncertainty  = 0.1*m;% double confirm with hansen
% max_Fx_uncertainty_braking  = 0.02*m;
