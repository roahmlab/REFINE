% For each maneuver,
% We make a continous time dynamics for state propogations dyn_*_change
% and then make a discrete time dynamics for forces dyn_*_change_forces
% and then make braking dynamics dyn_*_brake
% and then braking forces dynamics dyn_*_brake_forces
% and lastly slow speed braking dyn_*_slow

% JL change: use a common braking dynamics for high speed and low speed no
% matter what the previous maneuver is.
% Also , Since u =5 doesn't work well with high speed model, we make a special case for u = 5.
%

%This script also uses t0 as a parameter
%x,y, psi can be ini = 0 by doing coordinate tf
% u v r have inital condition with uncertainty

%1. specify U change, with u0 and p_u
%2. specify braking with p_u, u0 specify starting speed(continuation of 1)(take second part of the reference, should be a
%function of p_u only) (u>1 version)
%3. specify braking with p_u only(take second part of the reference, should be a
%function of p_u only) (u<=1 version)
%4. Lane change at proper speed(u>>1)
% braking for lane change can be obtained by using 2 and 3 using special 
% condition, namely p_u = u0  = u0 of  2 and 3.



%            %original states   %sliceable states         % 
%in2 states: x, y, psi, u, v, r,u0,v0, r0,t0 ,p_u,   p_y,   JL_brake, Fyr_delta, Fx_delta r_err_sum, h_err_sum, u_err_sum, th0   t 
%            1  2  3    4  5  6  7  8  9  10  11    12    13         14           15      16        17        18         19     20
% vehicle parameters
clear;
load my_const.mat
syms psi
syms JL_brake_t1
syms Fyr_delta Fx_delta
%% speed change specify p_u and u0
% velocities
syms_flag = 1;
syms t p_u u0 t0 % we sometimes have t0 here. In our dir change case, it is 1.5 s
p_y = 0; syms v0 r0 u v r%v0=0; r0=0; 

syms err_r_sum err_h_sum err_u_sum th0 
%Here p_y does not appear in dynamics

scale_Ay_flag = 0; % set p_y = 0 or set scale_flag to  0 is same



[T,U,Z] = sin_one_hump_parameterized_traj_with_brake(t0, p_y,p_u,u0,t,syms_flag,scale_Ay_flag,JL_brake_t1);

ud = U(1,1);
vd = U(2,1);
rd = U(3,1);
hd = zeros(size(U(1,1)));
%% Speed Change Section
ddyn = gen_closed_loop_dyn (hd, u, ud, v, vd, r, rd,psi, Fyr_delta, Fx_delta, err_r_sum, err_h_sum, err_u_sum, t);
syms x y tdummy udummy p_y
dyn = [x; y; psi; u; v; r; u0;v0; r0;t0;p_u; p_y;  JL_brake_t1; Fyr_delta; Fx_delta; err_r_sum; err_h_sum; err_u_sum; th0; t];
matlabFunction(ddyn, 'File', 'dyn_u_change', 'vars', {tdummy dyn udummy});


%% braking specify p_u, u0 only (U change and braking are different since t are different that are fixed in load_const)

ud = U(1,2);%2 means second portion
vd = U(2,2);
rd = U(3,2);

% use brake version
ddyn = gen_closed_loop_dyn (hd, u, ud, v, vd, r, rd,psi, Fyr_delta, Fx_delta, err_r_sum, err_h_sum, err_u_sum, t);
syms x y tdummy udummy p_y
dyn = [x;y; psi; u; v; r; u0;v0; r0;t0;p_u; p_y;  JL_brake_t1; Fyr_delta; Fx_delta; err_r_sum; err_h_sum; err_u_sum; th0; t];
matlabFunction(ddyn, 'File', 'dyn_u_brake', 'vars', {tdummy dyn udummy});


%% braking low spd
ud = U(1,3); % no more r and u dynamics

% use brake version  %u,ud,psi, psid,t
ddyn = gen_low_speed (u,ud,psi, rd,t);
syms x y tdummy udummy p_y
dyn = [x; y; psi; u; v; r;u0;v0; r0;t0;p_u; p_y;  JL_brake_t1; Fyr_delta; err_r_sum; err_h_sum; err_h_sum; th0; t];
matlabFunction(ddyn, 'File', 'dyn_u_slow', 'vars', {tdummy dyn udummy});

%% Lane Change section: specify u0 p_y

load my_const.mat
syms psi
syms_flag = 1;
syms t u0 p_y t0 %av0 au0 
%%%!!!! p_u is set to u0 here since not want to do any
% speed change when doing lane change
p_u = u0; syms v0 r0  u v r% v0=0; r0=0;
syms JL_brake_t1

scale_Ay_flag = 1;
% lane_info = load('dir_change_Ay_info.mat');

% for lane_t0_idx = 1:4
%     t0_lane = (lane_t0_idx-1) * lane_info.t0_dt;
    
% [T,U,Z] = sin_two_hump_parameterized_traj_with_brake(t0_lane, p_y,p_u,u0,t,syms_flag,scale_Ay_flag, JL_brake_t1);
[T,U,Z] = gaussian_T_parameterized_traj_with_brake(t0, p_y,p_u,u0,t,syms_flag,scale_Ay_flag, JL_brake_t1);
ud = U(1,1);
vd = U(2,1);
rd = U(3,1);
hd = Z(1);

ddyn = gen_closed_loop_dyn (hd - th0, u, ud, v, vd, r, rd,psi, Fyr_delta, Fx_delta, err_r_sum, err_h_sum, err_u_sum, t);
syms x y tdummy udummy
dyn = [x; y; psi; u; v; r; u0;v0; r0;t0;p_u; p_y;  JL_brake_t1; Fyr_delta; Fx_delta; err_r_sum; err_h_sum; err_u_sum; th0; t];
matlabFunction(ddyn, 'File', 'dyn_y_change', 'vars', {tdummy dyn udummy});
% end


%%
ud = U(1,2);%2 means second portion
vd = U(2,2);
rd = U(3,2);

hd = subs(Z(1),t,tpk);

% use brake version
ddyn = gen_closed_loop_dyn (hd-th0, u, ud, v, vd, r, rd,psi, Fyr_delta, Fx_delta, err_r_sum, err_h_sum, err_u_sum, t);
syms x y tdummy udummy p_y
dyn = [x; y; psi; u; v; r; u0;v0; r0;t0;p_u; p_y;  JL_brake_t1; Fyr_delta; Fx_delta; err_r_sum; err_h_sum; err_u_sum; th0; t];
matlabFunction(ddyn, 'File', 'dyn_y_brake', 'vars', {tdummy dyn udummy});

%% braking low spd
ud = U(1,3);

% use brake version
ddyn = gen_low_speed (u,ud,psi,rd,t);
syms x y tdummy udummy p_y
dyn = [x; y; psi; u; v; r;u0;v0; r0;t0;p_u; p_y;  JL_brake_t1; Fyr_delta; err_r_sum; err_h_sum; err_u_sum; th0; t];
matlabFunction(ddyn, 'File', 'dyn_y_slow', 'vars', {tdummy dyn udummy});

%% Dir change(same as lane change but with diffeernet reference)
load my_const.mat
syms psi
syms_flag = 1;
syms t u0 p_y t0 %av0 au0 
%%%!!!! p_u is set to u0 here since not want to do any
% speed change when doing lane change
p_u = u0; syms v0 r0  u v r% v0=0; r0=0;
syms JL_brake_t1

% u = delu + u0;
% v = delv + v0;
% r = delr + r0;


scale_Ay_flag = 1;
[T,U,Z] = sin_one_hump_parameterized_traj_with_brake(t0, p_y,p_u,u0,t,syms_flag,scale_Ay_flag, JL_brake_t1);

ud = U(1,1);
vd = U(2,1);
rd = U(3,1);
hd = Z(1);

ddyn = gen_closed_loop_dyn (hd-th0, u, ud, v, vd, r, rd,psi, Fyr_delta, Fx_delta, err_r_sum, err_h_sum, err_u_sum, t);
syms x y tdummy udummy
dyn = [x; y; psi; u; v; r; u0;v0; r0;t0;p_u; p_y;  JL_brake_t1; Fyr_delta; Fx_delta; err_r_sum; err_h_sum; err_u_sum; th0; t];
matlabFunction(ddyn, 'File', 'dyn_dir_change', 'vars', {tdummy dyn udummy});

%% dir brake
ud = U(1,2);%2 means second portion
vd = U(2,2);
rd = U(3,2);

hd = subs(Z(1),t,tpk_dir);

% use brake version
ddyn = gen_closed_loop_dyn (hd-th0, u, ud, v, vd, r, rd,psi, Fyr_delta, Fx_delta, err_r_sum, err_h_sum, err_u_sum, t);

syms x y tdummy udummy p_y
dyn = [x; y; psi; u; v; r; u0;v0; r0;t0;p_u; p_y;  JL_brake_t1; Fyr_delta; Fx_delta; err_r_sum; err_h_sum; err_u_sum;th0; t];
matlabFunction(ddyn, 'File', 'dyn_dir_brake', 'vars', {tdummy dyn udummy});

%% braking low spd
ud = U(1,3);

% use brake version
ddyn = gen_low_speed (u,ud,psi, rd,t);
syms x y tdummy udummy p_y
dyn = [x; y; psi; u; v; r;u0;v0; r0;t0;p_u; p_y; JL_brake_t1;  Fyr_delta;  err_r_sum; err_h_sum;err_u_sum; th0; t];
matlabFunction(ddyn, 'File', 'dyn_dir_slow', 'vars', {tdummy dyn udummy});




%%
function dx = gen_low_speed(u,ud,psi, rd,t)
    load my_const.mat
    %at low speed does not model with force, model with kinematic model
%     Ku = (Ku*10);
    %R = u/r
    g = 9.8;
    l = lf + lr;
    Kus = m * g * (lr / (l * Caf1) - lf / (l * Car1));
    delta = rd / (u+0.05) *(l+Kus*u^2/g);%-Kr * 0.2 * (psi - psid); 
    
    mr = lf/l *m;
    
    r = delta*u/(l+Kus*u^2/g); % true Sean model
%     r = ( (4139891213737223*delta^3)/9007199254740992 - (4978951411784147*delta^2)/10141204801825835211973625643008 + (4417959881449245*delta)/4503599627370496 + 181940544839889/20282409603651670423947251286016 )*u/(l+Kus*u^2/g);
    
    v = r*(lr - u^2*mr/Car1);
    
%     Fxr = -10*u;
    Fxf = - m*v*r/2 - m/2*Ku*(u-ud) + m/2*diff(ud,t);
    dudt = 1/m*(2*(Fxf)+m*v*r);
                                                            %use dvdt drdt here that correspond to what is already there in the high dim system
    dx = [  u .* cos( psi ) - v .* sin( psi ); u .* sin( psi ) + v .* cos( psi );  r; dudt; 0; 0; zeros(9,1);0; 0; 0;0; 1 ];%h_state does not exist

    
    
end
function [dx] = gen_closed_loop_dyn ( hd, u, ud, v, vd, r, rd,psi, Fyr_delta, Fx_delta, err_r_sum, err_h_sum, err_u_sum, t)
%for regular speed, use simplified dynamics after simplifyiing with smart
%controller
load my_const.mat
dud = diff(ud,t);
drd = diff(rd,t);

 % here this max is different from the Fyr_delta in that in different time steps, Fyr_delta can change.
% but d on the next line will stay the same;
d = lr*(max_Fyr_uncertainty)/Izz;
d_u = max_Fx_uncertainty/m;           

kappa = kappaP + kappaI*(err_r_sum + err_h_sum) ;
phi = phiP + phiI*(err_r_sum + err_h_sum);
r_err = r-rd;
h_err = psi-hd;
err_term = Kr*r_err + Kh*h_err;          
V = -(kappa*d + phi) *err_term;

kappaU = kappaPU + kappaIU*(err_u_sum);
phiU = phiPU + phiIU*(err_u_sum);
u_err = u - ud;
err_term = Ku*u_err;
U = -(kappaU*d_u + phiU) * err_term;

dudt = -Ku * ( u - ud )+ dud + Fx_delta/m + U;
drdt = -Kr * ( r - rd )+ drd - Kh * (psi - hd) - lr*Fyr_delta/Izz + V;
Fyrest =  - Car1 * ( Car2*( v - lr * r )/(u) );
Fyr    = Fyrest + Fyr_delta;% force are pretty linear 
Fyf =  lr/lf*Fyrest + Izz/lf*( -Kr * ( r - rd )+ drd - Kh * (psi - hd) +V);
dvdt = 1/m * (Fyf +Fyr) - u*r + Fyr_delta/m;

dx = [  u .* cos( psi ) - v .* sin( psi ); u .* sin( psi ) + v .* cos( psi );  r; dudt; dvdt; drdt; zeros(9,1); r_err^2; h_err^2; u_err^2; 0; 1];
end
