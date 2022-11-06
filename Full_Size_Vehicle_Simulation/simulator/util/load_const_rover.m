
%% DO NOT SHARE !!!

m = 4.9565;    
lf = 0.20;  
lr = 0.11;
Izz = 0.6; 
Cw = 3.41;
Caf1 = 110;
Caf2 = 1;  
Car1 = 315;
Car2 = 1;                       
% feedback linearization parameters
%control gains
% Kv = 0;
Kr = 2;
Ku = 4;
Kh = 5;
% Kv = 1;

amax = 0.5;%
tpk = 3;
% tbrk = 2;% no longer used
tpk_dir = 1.5;
tbrk2 =2;
u_really_slow = 0.5;

top_speed = 1.2;

mu_gnd =0.15;%0.275;%


car_length = 0.52/2; % these are supposed to be generator values so they need to be half of the actual value
car_width  = 0.28/2;
obs_localize_err = 0.08;

max_Fyr_uncertainty = 5;
% Kvy = m*lf/Ca/(lf+lr);