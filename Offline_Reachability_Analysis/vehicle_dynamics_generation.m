% For each maneuver, we symbolicly generate:
% a continous time dynamics for driving maneuver dyn_*_change,
% a continous time dynamics for contigency braking in high-speed mode dyn_*_brake, and
% a continous time dynamics for contigency braking in low-speed mode dyn_*_slow.
 
% states: x, y, h, u, v, r,u0,v0, r0,t0 ,p_u,   p_y,  brake_time, Fy_error, Fx_error r_err_sum, h_err_sum, u_err_sum, th0   t 
%         1  2  3    4  5  6  7  8  9  10  11    12    13         14           15      16        17        18         19    20
clear;
load my_const.mat
syms h brake_time Fy_error Fx_error t p_u u0 t0 v0 r0 u v r err_r_sum err_h_sum err_u_sum th0 
%% speed change 
% velocities
syms_flag = 1;
p_y = 0;  
not_speed_change = 0; 
[T,U,Z] = sin_one_hump_parameterized_traj_with_brake(t0, p_y,p_u,u0,t,syms_flag,not_speed_change,brake_time);

% driving maneuver
ud = U(1,1);
vd = U(2,1);
rd = U(3,1);
hd = zeros(size(U(1,1)));
ddyn = gen_closed_loop_dyn (hd, u, ud, v, vd, r, rd,h, Fy_error, Fx_error, err_r_sum, err_h_sum, err_u_sum, t);
syms x y tdummy udummy p_y
dyn = [x; y; h; u; v; r; u0;v0; r0;t0;p_u; p_y;  brake_time; Fy_error; Fx_error; err_r_sum; err_h_sum; err_u_sum; th0; t];
matlabFunction(ddyn, 'File', 'dyn_u_change', 'vars', {tdummy dyn udummy});

% contigency braking (high-speed)
ud = U(1,2);
vd = U(2,2);
rd = U(3,2);
ddyn = gen_closed_loop_dyn (hd, u, ud, v, vd, r, rd,h, Fy_error, Fx_error, err_r_sum, err_h_sum, err_u_sum, t);
dyn = [x;y; h; u; v; r; u0;v0; r0;t0;p_u; p_y;  brake_time; Fy_error; Fx_error; err_r_sum; err_h_sum; err_u_sum; th0; t];
matlabFunction(ddyn, 'File', 'dyn_u_brake', 'vars', {tdummy dyn udummy});

% contigency braking (low-speed)
ud = U(1,3); 
ddyn = gen_low_speed (u,ud,h, rd,t,err_u_sum);
dyn = [x; y; h; u; v; r;u0;v0; r0;t0;p_u; p_y;  brake_time; Fy_error; err_r_sum; err_h_sum; err_u_sum; th0; t];
matlabFunction(ddyn, 'File', 'dyn_u_slow', 'vars', {tdummy dyn udummy});

%% Lane Change
p_u = u0;
not_speed_change = 1;
[T,U,Z] = gaussian_T_parameterized_traj_with_brake(t0, p_y,p_u,u0,t,syms_flag,not_speed_change, brake_time);

% driving maneuver
ud = U(1,1);
vd = U(2,1);
rd = U(3,1);
hd = Z(1);
ddyn = gen_closed_loop_dyn (hd - th0, u, ud, v, vd, r, rd,h, Fy_error, Fx_error, err_r_sum, err_h_sum, err_u_sum, t);
dyn = [x; y; h; u; v; r; u0;v0; r0;t0;p_u; p_y;  brake_time; Fy_error; Fx_error; err_r_sum; err_h_sum; err_u_sum; th0; t];
matlabFunction(ddyn, 'File', 'dyn_y_change', 'vars', {tdummy dyn udummy});

% contigency braking (high-speed)
ud = U(1,2);
vd = U(2,2);
rd = U(3,2);
hd = subs(Z(1),t,tpk);
ddyn = gen_closed_loop_dyn (hd-th0, u, ud, v, vd, r, rd,h, Fy_error, Fx_error, err_r_sum, err_h_sum, err_u_sum, t);
dyn = [x; y; h; u; v; r; u0;v0; r0;t0;p_u; p_y;  brake_time; Fy_error; Fx_error; err_r_sum; err_h_sum; err_u_sum; th0; t];
matlabFunction(ddyn, 'File', 'dyn_y_brake', 'vars', {tdummy dyn udummy});

% contigency braking (low-speed)
ud = U(1,3);
ddyn = gen_low_speed (u,ud,h,rd,t,err_u_sum);
dyn = [x; y; h; u; v; r;u0;v0; r0;t0;p_u; p_y;  brake_time; Fy_error; err_r_sum; err_h_sum; err_u_sum; th0; t];
matlabFunction(ddyn, 'File', 'dyn_y_slow', 'vars', {tdummy dyn udummy});

%% Direction change
p_u = u0;
not_speed_change = 1;
[T,U,Z] = sin_one_hump_parameterized_traj_with_brake(t0, p_y,p_u,u0,t,syms_flag,not_speed_change, brake_time);

% driving maneuver
ud = U(1,1);
vd = U(2,1);
rd = U(3,1);
hd = Z(1);
ddyn = gen_closed_loop_dyn (hd-th0, u, ud, v, vd, r, rd,h, Fy_error, Fx_error, err_r_sum, err_h_sum, err_u_sum, t);
dyn = [x; y; h; u; v; r; u0;v0; r0;t0;p_u; p_y;  brake_time; Fy_error; Fx_error; err_r_sum; err_h_sum; err_u_sum; th0; t];
matlabFunction(ddyn, 'File', 'dyn_dir_change', 'vars', {tdummy dyn udummy});

% contigency braking (high-speed)
ud = U(1,2);
vd = U(2,2);
rd = U(3,2);
hd = subs(Z(1),t,tpk_dir);
ddyn = gen_closed_loop_dyn (hd-th0, u, ud, v, vd, r, rd,h, Fy_error, Fx_error, err_r_sum, err_h_sum, err_u_sum, t);
dyn = [x; y; h; u; v; r; u0;v0; r0;t0;p_u; p_y;  brake_time; Fy_error; Fx_error; err_r_sum; err_h_sum; err_u_sum;th0; t];
matlabFunction(ddyn, 'File', 'dyn_dir_brake', 'vars', {tdummy dyn udummy});

% contigency braking (low-speed)
ud = U(1,3);
ddyn = gen_low_speed (u,ud,h, rd,t,err_u_sum);
dyn = [x; y; h; u; v; r;u0;v0; r0;t0;p_u; p_y; brake_time;  Fy_error;  err_r_sum; err_h_sum;err_u_sum; th0; t];
matlabFunction(ddyn, 'File', 'dyn_dir_slow', 'vars', {tdummy dyn udummy});



%% help functions (dynamics)
function dx = gen_low_speed(u,ud,h, rd,t,err_u_sum)
    load my_const.mat
    l = lf + lr;
    Cus = m * grav_const * (lr / (l * Caf1) - lf / (l * Car1));
    delta = rd / (u+0.05) *(l+Cus*u^2/grav_const);
    
    mr = lf/l *m;
    
    rlo = delta*u/(l+Cus*u^2/grav_const);     
    vlo = rlo*(lr - u^2*mr/Car1);
    
    Mu_lo = max_Fx_uncertainty_braking / m;
    kappaU = kappaPU + kappaIU*(err_u_sum);
    phiU = phiPU + phiIU*(err_u_sum);
    u_err = u - ud;
    err_term = Ku*u_err;
    tau_u = -(kappaU*Mu_lo + phiU) * err_term;

    Fxf = - m*vlo*rlo/2 - m/2*Ku*(u-ud) + m/2*diff(ud,t);
    dudt = 1/m*(2*(Fxf)+m*vlo*rlo) + max_Fx_uncertainty_braking/m + tau_u;
                                                            
    dx = [  u .* cos( h ) - vlo .* sin( h ); u .* sin( h ) + vlo .* cos( h );  rlo; dudt; 0; 0; zeros(9,1);0; 0; 0;0; 1 ];%h_state does not exist

    
    
end
function [dx] = gen_closed_loop_dyn ( hd, u, ud, v, vd, r, rd,h, Fy_error, Fx_error, err_r_sum, err_h_sum, err_u_sum, t)
load my_const.mat
dud = diff(ud,t);
drd = diff(rd,t);
       

kappa = kappaP + kappaI*(err_r_sum + err_h_sum) ;
phi = phiP + phiI*(err_r_sum + err_h_sum);
r_err = r-rd;
h_err = h-hd;
err_term = Kr*r_err + Kh*h_err;          
tau_r = -(kappa*Mr + phi) *err_term;

kappaU = kappaPU + kappaIU*(err_u_sum);
phiU = phiPU + phiIU*(err_u_sum);
u_err = u - ud;
err_term = Ku*u_err;
tau_u = -(kappaU*Mu + phiU) * err_term;

dudt = -Ku * ( u - ud )+ dud + Fx_error/m + tau_u;
drdt = -Kr * ( r - rd )+ drd - Kh * (h - hd) - lr*Fy_error/Izz + tau_r;
Fyrest =  - Car1 * ( Car2*( v - lr * r )/(u));
Fyf =  lr/lf*Fyrest + Izz/lf*( -Kr * ( r - rd )+ drd - Kh * (h - hd) +tau_r);
dvdt = 1/m * (Fyf +Fyrest + Fy_error) - u*r + Fy_error/m;

dx = [  u .* cos( h ) - v .* sin( h ); u .* sin( h ) + v .* cos( h );  r; dudt; dvdt; drdt; zeros(9,1); r_err^2; h_err^2; u_err^2; 0; 1];
end
