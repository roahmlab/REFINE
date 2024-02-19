% For each maneuver, we symbolicly generate:
% a continous time dynamics for driving maneuver dyn_*_change,
% a continous time dynamics for contigency braking in high-speed mode dyn_*_brake, and
% a continous time dynamics for contigency braking in low-speed mode dyn_*_slow.
 
% states: x, y, h, vx, vy, r,vx0,vy0, r0,t0 ,p_vx,   p_y,  brake_time, Fy_error, Fx_error r_err_sum, h_err_sum, u_err_sum, h0   t 
%         1  2  3    4  5  6  7  8  9  10  11    12    13         14           15      16        17        18         19    20
clear;
load my_const.mat
syms h brake_time Fy_error Fx_error t p_vx vx0 t0 vy0 r0 vx vy r err_r_sum err_h_sum err_vx_sum h0 
%% speed change 
% velocities
syms_flag = 1;
p_y = 0;  
not_speed_change = 0; 
[T,U,Z] = sin_one_hump_parameterized_traj_with_brake(t0, p_y,p_vx,vx0,t,syms_flag,not_speed_change,brake_time);

% driving maneuver
vxd = U(1,1);
vyd = U(2,1);
rd = U(3,1);
hd = zeros(size(U(1,1)));
ddyn = gen_closed_loop_dyn (hd, vx, vxd, vy, vyd, r, rd,h, Fy_error, Fx_error, err_r_sum, err_h_sum, err_vx_sum, t);
syms x y tdummy udummy p_y
dyn = [x; y; h; vx; vy; r; vx0;vy0; r0;t0;p_vx; p_y;  brake_time; Fy_error; Fx_error; err_r_sum; err_h_sum; err_vx_sum; h0; t];
matlabFunction(ddyn, 'File', 'dyn_u_change', 'vars', {tdummy dyn udummy});

% contigency braking (high-speed)
vxd = U(1,2);
vyd = U(2,2);
rd = U(3,2);
ddyn = gen_closed_loop_dyn (hd, vx, vxd, vy, vyd, r, rd,h, Fy_error, Fx_error, err_r_sum, err_h_sum, err_vx_sum, t);
dyn = [x;y; h; vx; vy; r; vx0;vy0; r0;t0;p_vx; p_y;  brake_time; Fy_error; Fx_error; err_r_sum; err_h_sum; err_vx_sum; h0; t];
matlabFunction(ddyn, 'File', 'dyn_u_brake', 'vars', {tdummy dyn udummy});

% contigency braking (low-speed)
vxd = U(1,3); 
ddyn = gen_low_speed (vx,vxd,h, rd,t,err_vx_sum);
dyn = [x; y; h; vx; vy; r;vx0;vy0; r0;t0;p_vx; p_y;  brake_time; Fy_error; Fx_error; err_r_sum; err_h_sum; err_vx_sum; h0; t];
matlabFunction(ddyn, 'File', 'dyn_u_slow', 'vars', {tdummy dyn udummy});

%% Lane Change
p_vx = vx0;
not_speed_change = 1;
[T,U,Z] = gaussian_T_parameterized_traj_with_brake(t0, p_y,p_vx,vx0,t,syms_flag,not_speed_change, brake_time, isSim);

% driving maneuver
vxd = U(1,1);
vyd = U(2,1);
rd = U(3,1);
hd = Z(1);
ddyn = gen_closed_loop_dyn (hd - h0, vx, vxd, vy, vyd, r, rd,h, Fy_error, Fx_error, err_r_sum, err_h_sum, err_vx_sum, t);
dyn = [x; y; h; vx; vy; r; vx0;vy0; r0;t0;p_vx; p_y;  brake_time; Fy_error; Fx_error; err_r_sum; err_h_sum; err_vx_sum; h0; t];
matlabFunction(ddyn, 'File', 'dyn_y_change', 'vars', {tdummy dyn udummy});

% contigency braking (high-speed)
vxd = U(1,2);
vyd = U(2,2);
rd = U(3,2);
hd = subs(Z(1),t,tpk);
ddyn = gen_closed_loop_dyn (hd-h0, vx, vxd, vy, vyd, r, rd,h, Fy_error, Fx_error, err_r_sum, err_h_sum, err_vx_sum, t);
dyn = [x; y; h; vx; vy; r; vx0;vy0; r0;t0;p_vx; p_y;  brake_time; Fy_error; Fx_error; err_r_sum; err_h_sum; err_vx_sum; h0; t];
matlabFunction(ddyn, 'File', 'dyn_y_brake', 'vars', {tdummy dyn udummy});

% contigency braking (low-speed)
vxd = U(1,3);
ddyn = gen_low_speed (vx,vxd,h,rd,t,err_vx_sum);
dyn = [x; y; h; vx; vy; r;vx0;vy0; r0;t0;p_vx; p_y;  brake_time; Fy_error; Fx_error; err_r_sum; err_h_sum; err_vx_sum; h0; t];
matlabFunction(ddyn, 'File', 'dyn_y_slow', 'vars', {tdummy dyn udummy});

%% Direction change
p_vx = vx0;
not_speed_change = 1;
[T,U,Z] = sin_one_hump_parameterized_traj_with_brake(t0, p_y,p_vx,vx0,t,syms_flag,not_speed_change, brake_time);

% driving maneuver
vxd = U(1,1);
vyd = U(2,1);
rd = U(3,1);
hd = Z(1);
ddyn = gen_closed_loop_dyn (hd-h0, vx, vxd, vy, vyd, r, rd,h, Fy_error, Fx_error, err_r_sum, err_h_sum, err_vx_sum, t);
dyn = [x; y; h; vx; vy; r; vx0;vy0; r0;t0;p_vx; p_y;  brake_time; Fy_error; Fx_error; err_r_sum; err_h_sum; err_vx_sum; h0; t];
matlabFunction(ddyn, 'File', 'dyn_dir_change', 'vars', {tdummy dyn udummy});

% contigency braking (high-speed)
vxd = U(1,2);
vyd = U(2,2);
rd = U(3,2);
hd = subs(Z(1),t,tpk_dir);
ddyn = gen_closed_loop_dyn (hd-h0, vx, vxd, vy, vyd, r, rd,h, Fy_error, Fx_error, err_r_sum, err_h_sum, err_vx_sum, t);
dyn = [x; y; h; vx; vy; r; vx0;vy0; r0;t0;p_vx; p_y;  brake_time; Fy_error; Fx_error; err_r_sum; err_h_sum; err_vx_sum;h0; t];
matlabFunction(ddyn, 'File', 'dyn_dir_brake', 'vars', {tdummy dyn udummy});

% contigency braking (low-speed)
vxd = U(1,3);
ddyn = gen_low_speed (vx,vxd,h, rd,t,err_vx_sum);
dyn = [x; y; h; vx; vy; r;vx0;vy0; r0;t0;p_vx; p_y; brake_time;  Fy_error; Fx_error; err_r_sum; err_h_sum;err_vx_sum; h0; t];
matlabFunction(ddyn, 'File', 'dyn_dir_slow', 'vars', {tdummy dyn udummy});



%% help functions (dynamics)
function dx = gen_low_speed(vx,vxd,h, rd,t,err_vx_sum)
    load my_const.mat
    l = lf + lr;
    Cus = m * grav_const * (lr / (l * Caf1) - lf / (l * Car1));
    delta = rd / (vx+0.05) *(l+Cus*vx^2/grav_const);
    
    mr = lf/l *m;
    
    rlo = delta*vx/(l+Cus*vx^2/grav_const);     
    vlo = rlo*(lr - vx^2*mr/Car1);
    
    if isSim
        Mu_lo = max_Fx_uncertainty_braking / m;
    else
        Mu_lo = b_u_pro*vx+b_u_off;
    end

    kappa_vx = kappaP_vx + kappaI_vx*(err_vx_sum);
    phi_vx = phiP_vx + phiI_vx*(err_vx_sum);
    vx_err = vx - vxd;
    err_term = Ku*vx_err;
    tau_vx = -(kappa_vx*Mu_lo + phi_vx) * err_term;

    Fxf = - m*vlo*rlo/2 - m/2*Ku*(vx-vxd) + m/2*diff(vxd,t);
    if isSim
        dvxdt = 1/m*(2*(Fxf)+m*vlo*rlo) + Mu_lo + tau_vx;
    else
        dvxdt = 1/m*(2*(Fxf)+m*vlo*rlo) + tau_vx + (9.8095)*vx+(-41.129)*vx^2+(60.344)*vx^3 ; % (9.8095)*vx+(-41.129)*vx^2+(60.344)*vx^3  is an over approximation of delta_u based on REFINE-Fig.7
    end

    dx = [  vx .* cos( h ) - vlo .* sin( h ); vx .* sin( h ) + vlo .* cos( h );  rlo; dvxdt; 0; 0; zeros(9,1);0; 0; 0;0; 1 ];%h_state does not exist

    
    
end
function [dx] = gen_closed_loop_dyn ( hd, vx, vxd, vy, vyd, r, rd,h, Fy_error, Fx_error, err_r_sum, err_h_sum, err_vx_sum, t)
load my_const.mat
dvxd = diff(vxd,t);
drd = diff(rd,t);
       
% r terms
kappa = kappaP + kappaI*(err_r_sum + err_h_sum) ;
phi = phiP + phiI*(err_r_sum + err_h_sum);
r_err = r-rd;
h_err = h-hd;
err_term = Kr*r_err + Kh*h_err;          
tau_r = -(kappa*Mr + phi) *err_term;

% vx terms
kappa_vx = kappaP_vx + kappaI_vx*(err_vx_sum);
phi_vx = phiP_vx + phiI_vx*(err_vx_sum);
vx_err = vx - vxd;
err_term = Ku*vx_err;
tau_vx = -(kappa_vx*Mu + phi_vx) * err_term;


dvxdt = -Ku * ( vx - vxd )+ dvxd + Fx_error/m + tau_vx;
drdt = -Kr * ( r - rd )+ drd - Kh * (h - hd) - lr*Fy_error/Izz + tau_r;
Fyrest =  - Car1 * ( Car2*( vy - lr * r )/(vx));
Fyf =  lr/lf*Fyrest + Izz/lf*( -Kr * ( r - rd )+ drd - Kh * (h - hd) +tau_r);
dvydt = 1/m * (Fyf +Fyrest + Fy_error) - vx*r + Fy_error/m;

dx = [  vx .* cos( h ) - vy .* sin( h ); vx .* sin( h ) + vy .* cos( h );  r; dvxdt; dvydt; drdt; zeros(9,1); r_err^2; h_err^2; vx_err^2; 0; 1];
end
