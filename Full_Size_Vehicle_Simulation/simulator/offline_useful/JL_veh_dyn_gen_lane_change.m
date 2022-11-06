clear; close all; clc

%% lane + spd change  
%%%%%% u>6
load_const
syms psi
syms_flag = 1;
syms t u0 Ay t0 %av0 au0 
%%%!!!! Au is set to u0 here since not want to do any
% speed change when doing lane change
syms Au; syms v0 r0  u v r% v0=0; r0=0;

syms JL_brake_t1

% u = delu + u0;
% v = delv + v0;
% r = delr + r0;


scale_Ay_flag = 1;
[T,U,Z] = gaussian_T_parameterized_traj_with_brake(t0, Ay,Au,u0,t,syms_flag,scale_Ay_flag);

ud = U(1,1);
vd = U(2,1);
rd = U(3,1);

ddyn = gen_closed_loop_dyn (u, ud, v, vd, r, rd, psi, t);
syms x y tdummy udummy
dyn = [x; y; psi; u; v; r;u0;v0; r0;t0;Au; Ay; JL_brake_t1;  t];
matlabFunction(ddyn, 'File', 'dyn_y_u_change', 'vars', {tdummy dyn udummy});

[Fxf, Fyf, Fyr, delta] = gen_tire_force_steering_angle(u, ud, v, vd, r, rd, psi, t);
%in2 states:Fxf Fyf delta u, v, r,u0,v0, r0,t0 ,   Au, Ay,  JL_brake_t1, t
%            1  2    3    4   5  6  7  8  9  10  11    12    13          14  
ddyn = [Fxf; Fyf;  delta;  u; v; r; u0; v0; r0;  t0;     Au;    Ay;  JL_brake_t1; t];
syms Fxf Fyf Fyr delta discreteT
 dyn = [Fxf; Fyf;  delta;  u; v; r; u0;v0; r0;t0;    Au; Ay; JL_brake_t1;  t];
matlabFunction(ddyn, 'File', 'dyn_y_u_change_forces', 'vars', {tdummy dyn udummy discreteT});


%%%%%% u<6 lane change
ud = u0 + t/tpk*(Au-u0);
psid = (10*Ay*exp(-(1.2*(t + t0) - 3)^2))/11;
ddyn = gen_low_speed(u,ud,psi,psid,t);
matlabFunction(ddyn, 'File', 'dyn_y_u_change_u5', 'vars', {tdummy dyn udummy});


%% brake
%%% u>6
ud = Au - (Au-5) * t / JL_brake_t1; % make the desired speed 5 instead of 6 since system response has delay
vd = 0;
rd = 0;
ddyn = gen_closed_loop_dyn (u, ud, v, vd, r, rd, psi, t);
dyn = [x; y; psi; u; v; r;u0;v0; r0;t0;Au; Ay;  JL_brake_t1; t];
matlabFunction(ddyn, 'File', 'dyn_brake_high', 'vars', {tdummy dyn udummy});
[Fxf, Fyf, ~, delta] = gen_tire_force_steering_angle(u, ud, v, vd, r, rd, psi, t);
ddyn = [Fxf; Fyf;  delta;  u; v; r; u0; v0; r0;  t0;     Au;    Ay;  JL_brake_t1; t];
syms Fxf Fyf Fyr delta discreteT
 dyn = [Fxf; Fyf;  delta;  u; v; r; u0;v0; r0;t0;    Au; Ay;  JL_brake_t1; t];
matlabFunction(ddyn, 'File', 'dyn_brake_high_forces', 'vars', {tdummy dyn udummy discreteT});


%%% u<6 brake (Sean's model)
ud = 5*exp(-3*t^2);
psid = 0;
ddyn = gen_low_speed(u,ud,psi,psid,t);
dyn = [x; y; psi; u; v; r;u0;v0; r0;t0;Au; Ay; JL_brake_t1;  t];
matlabFunction(ddyn, 'File', 'dyn_brake_low', 'vars', {tdummy dyn udummy});





%% dir + speed change
clear
load_const
syms psi
syms_flag = 1;
syms t u0 Ay t0 %av0 au0 
%%%!!!! Au is set to u0 here since not want to do any
% speed change when doing lane change
syms Au; syms v0 r0  u v r% v0=0; r0=0;

syms JL_brake_t1

scale_Ay_flag = 1;
[T,U,Z] = gaussian_one_hump_parameterized_traj_with_brake(t0, Ay,Au,u0,t,syms_flag,scale_Ay_flag);

ud = U(1,1);
vd = U(2,1);
rd = U(3,1);

ddyn = gen_closed_loop_dyn (u, ud, v, vd, r, rd, psi, t);
syms x y tdummy udummy
dyn = [x; y; psi; u; v; r;u0;v0; r0;t0;Au; Ay;  JL_brake_t1; t];
matlabFunction(ddyn, 'File', 'dyn_dir_change', 'vars', {tdummy dyn udummy});
[Fxf, Fyf, ~, delta] = gen_tire_force_steering_angle(u, ud, v, vd, r, rd, psi, t);
%in2 states:Fxf Fyf delta u, v, r,u0,v0, r0,t0 ,   Au, Ay,   t
%            1  2    3    4   5  6  7  8  9  10  11    12    13    
ddyn = [Fxf; Fyf;  delta;  u; v; r; u0; v0; r0;  t0;     Au;    Ay; JL_brake_t1;  t];
syms Fxf Fyf Fyr delta discreteT
 dyn = [Fxf; Fyf;  delta;  u; v; r; u0;v0; r0;t0;    Au; Ay;  JL_brake_t1; t];
matlabFunction(ddyn, 'File', 'dyn_dir_change_forces', 'vars', {tdummy dyn udummy discreteT});






%% help function
function dx = gen_low_speed(u,ud,psi, psid,t)
    load_const
    
    Ku = (Ku*10);
    
    g = 9.8;
    l = lf + lr;
    Kus = m * g * (lr / (l * Cf) - lf / (l * Ca));
    delta = -Kr * 0.2 * (psi - psid);
    
    r = tan(delta)*u/(l+Kus*u^2/g); % true Sean model
%     r = ( (4139891213737223*delta^3)/9007199254740992 - (4978951411784147*delta^2)/10141204801825835211973625643008 + (4417959881449245*delta)/4503599627370496 + 181940544839889/20282409603651670423947251286016 )*u/(l+Kus*u^2/g);
    
    v = r*(lr - u^2*lr/(Ca*l));
    
    Fxr = -10*u;
    Fxf = -Fxr - m*v*r/2 - m/2*Ku*(u-ud) + m/2*diff(ud,t);
    du = 1/m*(2*(Fxf+Fxr)+m*v*r);
    
    dx = [u .* cos( psi ) - v .* sin( psi ); u .* sin( psi ) + v .* cos( psi );  r; ...
        du; 0;0;zeros(6+1,1); 1];
    
    
end
function [dx]=gen_closed_loop_dyn (u, ud, v, vd, r, rd,psi,t)
%for regular speed, use simplified dynamics after simplifyiing with smart
%controller
load_const
dud = diff(ud,t);
dvd = diff(vd,t);
dudt = -Ku * ( u - ud )+ dud;
dvdt = -Kv * ( v - vd )+ dvd - Kr * ( r - rd );
drdt = 2/Izz * ( ( lf + lr ) * Ca * ( ( v - lr * r )/(u) ) + lf * m * u * r/2 - m/2 * Kv * lf * ( v - vd ) + m/2 * lf * dvd - Kr * lf* m/2 * (r - rd) );



dx = [  u .* cos( psi ) - v .* sin( psi ); u .* sin( psi ) + v .* cos( psi );  r; dudt; dvdt; drdt; zeros(6+1,1); 1 ];
end
function [dx]=gen_closed_loop_dyn_brk (u, ud, v, vd, r, rd,psi,t)
% use a approxiamtion of  v - lr * r  instead of dividing by u so 
load_const
dud = diff(ud,t);
dvd = diff(vd,t);
dudt = -Ku * ( u - ud )+ dud;
dvdt = -Kv * ( v - vd )+ dvd - Kr * ( r - rd );
drdt = 2/Izz * ( ( lf + lr ) * Ca * ( ( v - lr * r ) ) + lf * m * u * r/2 - m/2 * Kv * lf * ( v - vd ) + m/2 * lf * dvd - Kr * lf* m/2 * (r - rd) );
% since always brake to 0 at the end and r is usually 0 since never
% changing lane while braking, just always omit u when braking.

% for slow speed dividing by u doesn't work, so instead, get rid of the u
% in the demonminator.

dx = [  u .* cos( psi ) - v .* sin( psi ); u .* sin( psi ) + v .* cos( psi );  r; dudt; dvdt; drdt; zeros(6+1,1); 1 ];
end
