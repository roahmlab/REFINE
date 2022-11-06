function [Fxf, Fyf,Fyr, delta] = gen_tire_force_steering_angle(u, ud, v, vd, r, rd, psi, t)
%in2 states:Fxf Fyf delta u, v, r,u0,v0, r0,t0 ,Au,   Ay,   t
%            1  2    3    4  5  6  7  8  9  10  11    12    13
load_const
dud = diff(ud,t);
dvd = diff(vd,t);
theta_vf = (v + lf*r)/u; 
Fxf = -m*v*r/2 - m/2*Ku*(u-ud) + m/2*dud;
Fyr = -Car1*Car2*(v-lr*r)/u; %all forces are single wheel force
Fyf = -Fyr + m*u*r/2 - m/2*Kv*(v-vd) + m/2*dvd -m/2*Kr*(r-rd);
delta = Fyf /Caf1/Caf2 + theta_vf; %2.25 Rajamani
end