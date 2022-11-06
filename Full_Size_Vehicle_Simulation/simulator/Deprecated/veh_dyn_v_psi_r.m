function dx = veh_dyn_v_psi_r(tdummy,in2,udummy)

%in2 states: x, y, u, v, psi, r, Ay, Au,av0,au0,u0,v0,r0, t 
            %1  2  3  4  5    6  7    8  9  10  11 12  13 14
% vehicle parameters
load_const

% time
t = in2( 14, : );
psi = in2( 5, : );

% amplitude
Ay = in2( 7, : );
Au = in2( 8, : );
av0 = in2( 9, : );
au0 = in2( 10, : );
u0 = in2( 11, : );
v0 = in2( 12, : );
r0 = in2( 13, : );

% velocities
[~, U, ~]= gaussian_parameterized_traj_with_brake(Ay,Au,u0,t,1);
ud = U(1,1);
dud = U(2,1);
vd = U(3,1);
dvd = U(4,1);
rd = U(5,1);
% [T,U,Z]=saferl_parameterized_traj_with_brake(Ay, Au,av0,au0,u0,v0,r0,t,1);
% ud = (Au-u0)/tpk*t+u0; 
% dud = (Au-u0)/tpk;
% vd = U(2,1);
% dvd = Z(3,1);
% rd = U(3,1);
u = in2( 3, : );
v = in2( 4, : );
r = in2( 6, : );

dudt = -Ku * ( u - ud )+ dud;
dvdt = -Kv * ( v - vd )+ dvd - Kr * ( r - rd );
drdt = 2/Izz * ( ( lf + lr ) * Ca * ( ( v - lr * r )/u ) + lf * m * u * r/2 - m/2 * Kv * lf * ( v - vd ) + m/2 * lf * dvd - Kr * lf* m/2 * (r - rd) );


dx = [  u .* cos( psi ) - v .* sin( psi ); u .* sin( psi ) + v .* cos( psi ); dudt; dvdt; r; drdt; zeros(7,1); 1 ];
 %         1                                     2  %           %           %   3   4     5   6     
