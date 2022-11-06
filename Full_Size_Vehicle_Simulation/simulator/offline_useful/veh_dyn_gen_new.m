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

%1. specify U change, with u0 and Au
%2. specify braking with Au, u0 specify starting speed(continuation of 1)(take second part of the reference, should be a
%function of Au only) (u>1 version)
%3. specify braking with Au only(take second part of the reference, should be a
%function of Au only) (u<=1 version)
%4. Lane change at proper speed(u>>1)
% braking for lane change can be obtained by using 2 and 3 using special 
% condition, namely Au = u0  = u0 of  2 and 3.



%     
%in2 states: x, y, psi, u, v, r,u0,v0, r0,t0 ,Au,   Ay,   t
%            1  2  3    4  5  6  7  8  9  10  11    12    13
% vehicle parameters
clear;
load_const
syms psi
syms JL_brake_t1
%% speed change specity Au and u0
% velocities
syms_flag = 1;
syms t Au u0 t0%Ay av0 au0 
Ay = 0; syms v0 r0 u v r%v0=0; r0=0;
%Here Ay does not appear in dynamics

scale_Ay_flag = 0; % set Ay = 0 or set scale_flag to  0 is same
[T,U,Z] = gaussian_one_hump_parameterized_traj_with_brake(t0, Ay,Au,u0,t,syms_flag,scale_Ay_flag);

ud = U(1,1);
vd = U(2,1);
rd = U(3,1);


ddyn = gen_closed_loop_dyn (u, ud, v, vd, r, rd, psi, t);
syms x y tdummy udummy Ay
dyn = [x; y; psi; u; v; r; u0;v0; r0;t0;Au; Ay;  JL_brake_t1; t];
matlabFunction(ddyn, 'File', 'dyn_u_change', 'vars', {tdummy dyn udummy});

% forces
[Fxf, Fyf, ~, delta] = gen_tire_force_steering_angle(u, ud, v, vd, r, rd, psi, t);
%in2 states:Fxf Fyf delta u, v, r,u0,v0, r0,t0 ,   Au, Ay,   t
%            1  2    3    4   5  6  7  8  9  10  11    12    13    
ddyn = [Fxf; Fyf;  delta;  u; v; r; u0; v0; r0;  t0;     Au;    Ay;  JL_brake_t1; t];
syms Fxf Fyf Fyr delta discreteT
 dyn = [Fxf; Fyf;  delta;  u; v; r; u0;v0; r0;t0;    Au; Ay; JL_brake_t1;  t];
matlabFunction(ddyn, 'File', 'dyn_u_change_forces', 'vars', {tdummy dyn udummy discreteT});

%% braking specify Au, u0 only (U change and braking are different since t are different that are fixed in load_const)

ud = U(1,2);%2 means second portion
vd = U(2,2);
rd = U(3,2);

% use brake version
ddyn = gen_closed_loop_dyn (u, ud, v, vd, r, rd, psi, t);
syms x y tdummy udummy Ay
dyn = [x; y; psi; u; v; r;u0;v0; r0;t0;Au; Ay; JL_brake_t1;  t];
matlabFunction(ddyn, 'File', 'dyn_u_brake', 'vars', {tdummy dyn udummy});

% forces
[Fxf, Fyf, ~, delta] = gen_tire_force_steering_angle(u, ud, v, vd, r, rd, psi, t);
%in2 states:Fxf Fyf delta u, v, r,u0,v0, r0,t0 ,   Au, Ay,   t
%            1  2    3    4   5  6  7  8  9  10  11    12    13    
ddyn = [Fxf; Fyf;  delta;  u; v; r; u0; v0; r0;  t0;     Au;    Ay; JL_brake_t1;  t];
syms Fxf Fyf Fyr delta discreteT
 dyn = [Fxf; Fyf;  delta;  u; v; r; u0;v0; r0;t0;    Au; Ay;  JL_brake_t1; t];
matlabFunction(ddyn, 'File', 'dyn_u_brake_forces', 'vars', {tdummy dyn udummy discreteT});

%% braking low spd
ud = U(1,2);
vd = U(2,2);
rd = U(3,2);

% use brake version
ddyn = gen_closed_loop_dyn_brk (u, ud, v, vd, r, rd, psi, t);
syms x y tdummy udummy Ay
dyn = [x; y; psi; u; v; r;u0;v0; r0;t0;Au; Ay;  JL_brake_t1; t];
matlabFunction(ddyn, 'File', 'dyn_u_slow', 'vars', {tdummy dyn udummy});

%% lane change: specify u0 Ay
clear
load_const
syms psi
syms_flag = 1;
syms t u0 Ay t0 %av0 au0 
%%%!!!! Au is set to u0 here since not want to do any
% speed change when doing lane change
Au = u0; syms v0 r0  u v r% v0=0; r0=0;
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
dyn = [x; y; psi; u; v; r;u0;v0; r0;t0;Au; Ay;  JL_brake_t1; t];
matlabFunction(ddyn, 'File', 'dyn_y_change', 'vars', {tdummy dyn udummy});
%foces
[Fxf, Fyf, Fyr, delta] = gen_tire_force_steering_angle(u, ud, v, vd, r, rd, psi, t);
%in2 states:Fxf Fyf delta u, v, r,u0,v0, r0,t0 ,   Au, Ay,   t
%            1  2    3    4   5  6  7  8  9  10  11    12    13    
ddyn = [Fxf; Fyf;  delta;  u; v; r; u0; v0; r0;  t0;     Au;    Ay;  JL_brake_t1; t];
syms Fxf Fyf Fyr delta discreteT
 dyn = [Fxf; Fyf;  delta;  u; v; r; u0;v0; r0;t0;    Au; Ay; JL_brake_t1;  t];
matlabFunction(ddyn, 'File', 'dyn_y_change_forces', 'vars', {tdummy dyn udummy discreteT});

%%
ud = U(1,2);%2 means second portion
vd = U(2,2);
rd = U(3,2);

% use brake version
ddyn = gen_closed_loop_dyn (u, ud, v, vd, r, rd, psi, t);
syms x y tdummy udummy Ay
dyn = [x; y; psi; u; v; r;u0;v0; r0;t0;Au; Ay;  JL_brake_t1; t];
matlabFunction(ddyn, 'File', 'dyn_y_brake', 'vars', {tdummy dyn udummy});
%forces
[Fxf, Fyf, ~, delta] = gen_tire_force_steering_angle(u, ud, v, vd, r, rd, psi, t);
%in2 states:Fxf Fyf delta u, v, r,u0,v0, r0,t0 ,   Au, Ay,   t
%            1  2    3    4   5  6  7  8  9  10  11    12    13    
ddyn = [Fxf; Fyf;  delta;  u; v; r; u0; v0; r0;  t0;     Au;    Ay; JL_brake_t1;  t];
syms Fxf Fyf Fyr delta discreteT
 dyn = [Fxf; Fyf;  delta;  u; v; r; u0;v0; r0;t0;    Au; Ay; JL_brake_t1;  t];
matlabFunction(ddyn, 'File', 'dyn_y_brake_forces', 'vars', {tdummy dyn udummy discreteT});

%% braking low spd
ud = U(1,2);
vd = U(2,2);
rd = U(3,2);

% use brake version
ddyn = gen_closed_loop_dyn_brk (u, ud, v, vd, r, rd, psi, t);
syms x y tdummy udummy Ay
dyn = [x; y; psi; u; v; r;u0;v0; r0;t0;Au; Ay;  JL_brake_t1; t];
matlabFunction(ddyn, 'File', 'dyn_y_slow', 'vars', {tdummy dyn udummy});

%% dir change(same as lane change but with diffeernet reference)
clear
load_const
syms psi
syms_flag = 1;
syms t u0 Ay t0 %av0 au0 
%%%!!!! Au is set to u0 here since not want to do any
% speed change when doing lane change
Au = u0; syms v0 r0  u v r% v0=0; r0=0;
syms JL_brake_t1

% u = delu + u0;
% v = delv + v0;
% r = delr + r0;


scale_Ay_flag = 1;
[T,U,Z] = gaussian_one_hump_parameterized_traj_with_brake(t0, Ay,Au,u0,t,syms_flag,scale_Ay_flag);

ud = U(1,1);
vd = U(2,1);
rd = U(3,1);

ddyn = gen_closed_loop_dyn (u, ud, v, vd, r, rd, psi, t);
syms x y tdummy udummy
dyn = [x; y; psi; u; v; r;u0;v0; r0;t0;Au; Ay;  JL_brake_t1; t];
matlabFunction(ddyn, 'File', 'dyn_dir_change', 'vars', {tdummy dyn udummy});

% forces
[Fxf, Fyf, ~, delta] = gen_tire_force_steering_angle(u, ud, v, vd, r, rd, psi, t);
%in2 states:Fxf Fyf delta u, v, r,u0,v0, r0,t0 ,   Au, Ay,   t
%            1  2    3    4   5  6  7  8  9  10  11    12    13    
ddyn = [Fxf; Fyf;  delta;  u; v; r; u0; v0; r0;  t0;     Au;    Ay; JL_brake_t1;  t];
syms Fxf Fyf Fyr delta discreteT
 dyn = [Fxf; Fyf;  delta;  u; v; r; u0;v0; r0;t0;    Au; Ay;  JL_brake_t1; t];
matlabFunction(ddyn, 'File', 'dyn_dir_change_forces', 'vars', {tdummy dyn udummy discreteT});
%% dir brake
ud = U(1,2);%2 means second portion
vd = U(2,2);
rd = U(3,2);

% use brake version
ddyn = gen_closed_loop_dyn (u, ud, v, vd, r, rd, psi, t);
syms x y tdummy udummy Ay
dyn = [x; y; psi; u; v; r;u0;v0; r0;t0;Au; Ay;  JL_brake_t1; t];
matlabFunction(ddyn, 'File', 'dyn_dir_brake', 'vars', {tdummy dyn udummy});
% forces
[Fxf, Fyf, ~, delta] = gen_tire_force_steering_angle(u, ud, v, vd, r, rd, psi, t);
%in2 states:Fxf Fyf delta u, v, r,u0,v0, r0,t0 ,   Au, Ay,   t
%            1  2    3    4   5  6  7  8  9  10  11    12    13    
ddyn = [Fxf; Fyf;  delta;  u; v; r; u0; v0; r0;  t0;     Au;    Ay;  JL_brake_t1; t];
syms Fxf Fyf Fyr delta discreteT
 dyn = [Fxf; Fyf;  delta;  u; v; r; u0;v0; r0;t0;    Au; Ay;  JL_brake_t1; t];
matlabFunction(ddyn, 'File', 'dyn_dir_brake_forces', 'vars', {tdummy dyn udummy discreteT});

%% braking low spd
ud = U(1,2);
vd = U(2,2);
rd = U(3,2);

% use brake version
ddyn = gen_closed_loop_dyn_brk (u, ud, v, vd, r, rd, psi, t);
syms x y tdummy udummy Ay
dyn = [x; y; psi; u; v; r;u0;v0; r0;t0;Au; Ay; JL_brake_t1;  t];
matlabFunction(ddyn, 'File', 'dyn_dir_slow', 'vars', {tdummy dyn udummy});





%% JL addict

clear
load_const
really_low_spd = 0.2;
syms psi
syms_flag = 1;
syms t u0 Ay t0 %av0 au0 
syms Au;
syms v0 r0  u v r% v0=0; r0=0;
syms x y tdummy udummy Ay
syms JL_brake_t1
dyn = [x; y; psi; u; v; r;u0;v0; r0;t0;Au; Ay; JL_brake_t1;  t];

%%% BREAKING
% u<6 brake (Sean's model)
% ud = 5*exp(-3*t^2); % let's say we want to slow down within 2 second
% ud = really_low_spd*(1-0.5*t)^4 + 15*really_low_spd/5*0.5*t*(1-0.5*t)^3;
% psid = 0; % assume desired heading angle is 0
% ddyn = gen_low_speed(u,ud,psi,psid,t);
% matlabFunction(ddyn, 'File', 'dyn_brake_low', 'vars', {tdummy dyn udummy});

% u>6 brake
ud = Au - (Au) * t / JL_brake_t1; % make the desired speed 5 instead of 6 since system response has delay
vd = 0;
rd = 0;
ddyn = gen_closed_loop_dyn (u, ud, v, vd, r, rd, psi, t);
dyn = [x; y; psi; u; v; r;u0;v0; r0;t0;Au; Ay;  JL_brake_t1; t];
matlabFunction(ddyn, 'File', 'dyn_brake_high', 'vars', {tdummy dyn udummy});
[Fxf, Fyf, ~, delta] = gen_tire_force_steering_angle(u, ud, v, vd, r, rd, psi, t);
ddyn = [Fxf; Fyf;  delta;  u; v; r; u0; v0; r0;  t0;     Au;    Ay;  JL_brake_t1; t];
return
syms Fxf Fyf Fyr delta discreteT
 dyn = [Fxf; Fyf;  delta;  u; v; r; u0;v0; r0;t0;    Au; Ay;  JL_brake_t1; t];
matlabFunction(ddyn, 'File', 'dyn_brake_high_forces', 'vars', {tdummy dyn udummy discreteT});




%%% LANE CHANGE 
ud = u0;
psid = (10*Ay*exp(-(1.2*(t + t0) - 3)^2))/11;
dyn = [x; y; psi; u; v; r;u0;v0; r0;t0;Au; Ay;  JL_brake_t1; t];
ddyn = gen_low_speed(u,ud,psi,psid,t);
matlabFunction(ddyn, 'File', 'dyn_y_change_u5', 'vars', {tdummy dyn udummy});


%%% DIR CHANGE
ud = u0;
% psid = (5*3^(1/2)*Ay*pi^(1/2)* (tanh(2*(t+t0-1.5))+1)  )/33;
psid = (5*3^(1/2)*Ay*pi^(1/2)* (tanh(2.2*((t+0.1)+t0-1.5))+1)  )/33;
% psid = (5*3^(1/2)*Ay*pi^(1/2)* erf(3^(1/2)*(t + t0 - 3/2))  )/33
ddyn = gen_low_speed(u,ud,psi,psid,t);
matlabFunction(ddyn, 'File', 'dyn_dir_change_u5', 'vars', {tdummy dyn udummy});

%%% SPEED CHANGE
syms Au
ud = u0 + t*(Au/3 - u0/3);
psid = 0;
ddyn = gen_low_speed(u,ud,psi,psid,t);
dyn = [x; y; psi; u; v; r;u0;v0; r0;t0;Au; Ay;  JL_brake_t1; t];
matlabFunction(ddyn, 'File', 'dyn_u_change_u5', 'vars', {tdummy dyn udummy});


%% %         1                                     2  %           %           %   3   4     5   6



%%
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
Fyr =  - Car1 * ( Car2*( v - lr * r )/(u) );
drdt = 2/Izz * ( - ( lf + lr ) * Fyr + lf * m * u * r/2 - m/2 * Kv * lf * ( v - vd ) + m/2 * lf * dvd - Kr * lf* m/2 * (r - rd) );



dx = [  u .* cos( psi ) - v .* sin( psi ); u .* sin( psi ) + v .* cos( psi );  r; dudt; dvdt; drdt; zeros(6+1,1); 1 ];
end
function [dx]=gen_closed_loop_dyn_brk (u, ud, v, vd, r, rd,psi,t)
% use a approxiamtion of  v - lr * r  instead of dividing by u so 
load_const
dud = diff(ud,t);
dvd = diff(vd,t);
dudt = -Ku * ( u - ud )+ dud;
dvdt = -Kv * ( v - vd )+ dvd - Kr * ( r - rd );
Fyr =  - Car1 * ( Car2*( v - lr * r ));
drdt = 2/Izz * ( -( lf + lr ) * Fyr + lf * m * u * r/2 - m/2 * Kv * lf * ( v - vd ) + m/2 * lf * dvd - Kr * lf* m/2 * (r - rd) );
% since always brake to 0 at the end and r is usually 0 since never
% changing lane while braking, just always omit u when braking.

% for slow speed dividing by u doesn't work, so instead, get rid of the u
% in the demonminator.

dx = [  u .* cos( psi ) - v .* sin( psi ); u .* sin( psi ) + v .* cos( psi );  r; dudt; dvdt; drdt; zeros(6+1,1); 1 ];
end
%@JL, consider using the following when for all dynamics above when doing
%sparse poly zono. causes splitting when not using that
function dx = gen_closed_loop_dyn_together (u,ud,v,vd,r,rd,psi,t)
u_limited = 1/(1+exp(-9*(u-1)))*u +1/(1+exp(-9*(-u+1.05)));

load_const
dud = diff(ud,t);
dvd = diff(vd,t);
dudt = -Ku * ( u - ud )+ dud;
dvdt = -Kv * ( v - vd )+ dvd - Kr * ( r - rd );
drdt = 2/Izz * ( ( lf + lr ) * Ca * ( ( v - lr * r )/(u_limited) ) + lf * m * u * r/2 - m/2 * Kv * lf * ( v - vd ) + m/2 * lf * dvd - Kr * lf* m/2 * (r - rd) );



dx = [  u .* cos( psi ) - v .* sin( psi ); u .* sin( psi ) + v .* cos( psi );  r; dudt; dvdt; drdt; zeros(6+1,1); 1 ];
%                u at high spd          u at low sp8 

end