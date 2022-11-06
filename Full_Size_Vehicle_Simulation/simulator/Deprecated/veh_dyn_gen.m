%use simplified states: get rid of u state, always use deltau to denote
%u = delta_u + u0;
%same idea for v and r
%x,y, psi can be ini = 0 by doing coordinate tf
% u v r have inital condition with uncertainty
%av0, au0 deprecated
%inital acc, deprecated

%1. specify U change, with u0 and Au
%2. specify braking with Au, u0 specify starting speed(continuation of 1)(take second part of the reference, should be a
%function of Au only) (u>1 version)
%3. specify braking with Au only(take second part of the reference, should be a
%function of Au only) (u<=1 version)
%4. Lane change at proper speed(u>>1)
% braking for lane change can be obtained by using 2 and 3 using special 
% condition, namely Au = u0  = u0 of  2 and 3.



%            0  0  0      0   0      0   omit omit ini       desired
%in2 states: x, y, psi, delu, delv, delr, av0,au0,u0,v0, r0,Au, Ay,   t
%1  2  3    4        5     6  7    8  9  10  11 12  13   14
% vehicle parameters
load_const
syms delu delv delr psi
%% speed change specity Au and u0
% velocities
syms_flag = 1;
syms t Au u0 %Ay av0 au0 v0 r0
Ay = 0; av0 = 0; au0 = 0; v0=0; r0=0;

u = delu + u0;
v = delv + v0;
r = delr + r0;


scale_Ay_flag = 0; % set Ay = 0 or set scale_flag to  0 is same
[T,U,Z] = gaussian_parameterized_traj_with_brake(Ay,Au,u0,t,syms_flag,scale_Ay_flag);

ud = U(1,1);
vd = U(2,1);
rd = U(3,1);


ddyn = gen_closed_loop_dyn (u, ud, v, vd, r, rd, psi, t);
syms x y tdummy udummy av0 au0 v0 r0 Ay
dyn = [x; y; psi; delu; delv; delr; av0;au0;u0;v0; r0;Au; Ay;   t];
matlabFunction(ddyn, 'File', 'dyn_u_change', 'vars', {tdummy dyn udummy});

%% braking specify Au, u0 only (U change and braking are different since t are different that are fixed in load_const)

ud = U(1,2);
vd = U(2,2);
rd = U(3,2);

% use brake version
ddyn = gen_closed_loop_dyn (u, ud, v, vd, r, rd, psi, t);
syms x y tdummy udummy av0 au0 v0 r0 Ay
dyn = [x; y; psi; delu; delv; delr; av0;au0;u0;v0; r0;Au; Ay;   t];
matlabFunction(ddyn, 'File', 'dyn_u_brake', 'vars', {tdummy dyn udummy});


%% braking low spd
ud = U(1,2);
vd = U(2,2);
rd = U(3,2);

% use brake version
ddyn = gen_closed_loop_dyn_brk (u, ud, v, vd, r, rd, psi, t);
syms x y tdummy udummy av0 au0 v0 r0 Ay
dyn = [x; y; psi; delu; delv; delr; av0;au0;u0;v0; r0;Au; Ay;   t];
matlabFunction(ddyn, 'File', 'dyn_u_slow', 'vars', {tdummy dyn udummy});

%% lane change: specify u0 Ay
clear
load_const
syms delu delv delr psi
syms_flag = 1;
syms t u0 Ay %av0 au0 v0 r0
%%%!!!! Au is set to u0 here since not want to do any
% speed change when doing lane change
Au = u0; av0 = 0; au0 = 0; v0=0; r0=0;

u = delu + u0;
v = delv + v0;
r = delr + r0;


scale_Ay_flag = 1;
[T,U,Z] = gaussian_parameterized_traj_with_brake(Ay,Au,u0,t,syms_flag,scale_Ay_flag);

ud = U(1,1);
vd = U(2,1);
rd = U(3,1);

ddyn = gen_closed_loop_dyn (u, ud, v, vd, r, rd, psi, t);
syms x y tdummy udummy av0 au0 v0 r0
dyn = [x; y; psi; delu; delv; delr; av0;au0;u0;v0; r0;Au; Ay;   t];
matlabFunction(ddyn, 'File', 'dyn_y_change', 'vars', {tdummy dyn udummy});
%% %         1                                     2  %           %           %   3   4     5   6

function [dx]=gen_closed_loop_dyn (u, ud, v, vd, r, rd,psi,t)
%for regular speed, use simplified dynamics after simplifyiing with smart
%controller
load_const
dud = diff(ud,t);
dvd = diff(vd,t);
dudt = -Ku * ( u - ud )+ dud;
dvdt = -Kv * ( v - vd )+ dvd - Kr * ( r - rd );
drdt = 2/Izz * ( ( lf + lr ) * Ca * ( ( v - lr * r )/(u) ) + lf * m * u * r/2 - m/2 * Kv * lf * ( v - vd ) + m/2 * lf * dvd - Kr * lf* m/2 * (r - rd) );



dx = [  u .* cos( psi ) - v .* sin( psi ); u .* sin( psi ) + v .* cos( psi );  r; dudt; dvdt; drdt; zeros(7,1); 1 ];
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

dx = [  u .* cos( psi ) - v .* sin( psi ); u .* sin( psi ) + v .* cos( psi );  r; dudt; dvdt; drdt; zeros(7,1); 1 ];
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



dx = [  u .* cos( psi ) - v .* sin( psi ); u .* sin( psi ) + v .* cos( psi );  r; dudt; dvdt; drdt; zeros(7,1); 1 ];
%                u at high spd          u at low spd

end