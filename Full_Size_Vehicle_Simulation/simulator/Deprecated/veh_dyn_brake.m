function dx = veh_dyn_brake(tdummy,in2,udummy)


% vehicle parameters
load_const

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
u = in2( 3, : );
v = in2( 4, : );
r = in2( 6, : );

% velocities
[~, U,~] = gaussian_parameterized_traj_with_brake(Ay,Au,u0,t,1);
ud = U(1,2);
dud = U(2,2);
vd = U(3,2);
dvd = U(4,2);
rd = U(5,2);

dudt = -Ku * ( u - ud )+ dud;
dvdt = -Kv * ( v - vd )+ dvd - Kr * ( r - rd );
drdt = 2/Izz * ( ( lf + lr ) * Ca * ( ( v - lr * r )/u ) + lf * m * u * r/2 - m/2 * Kv * lf * ( v - vd ) + m/2 * lf * dvd - Kr * lf* m/2 * (r - rd) );


dx = [  u .* cos( psi ) - v .* sin( psi ); u .* sin( psi ) + v .* cos( psi ); dudt; dvdt; r; drdt; zeros(7,1); 1 ];
