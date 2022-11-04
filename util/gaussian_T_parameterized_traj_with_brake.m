function [T U Z] = gaussian_T_parameterized_traj_with_brake(t0,del_y,Au,u0,t,symbolic_flag, scale_Ay_flag, brake_time)
%In symbolic = 0, t argument will be ignored
load my_const.mat
% tbrk = Au/amax;
% This was the effort to fit a function so del_y actually directly means
% the amount of y direction change, which will be more meaningful as a
% parameter. 
%  Ay= del_y;%/(-5.363);%/(1.35-1.057*u0);%(del_y+0.284-0.1027*u0) / (0.5538-0.94*u0);
% Ay = (del_y-4.6595+0.5324*u0)/(-7.8876);%fitted using sim data
% Ay = (del_y-3.8579+0.4422*u0)/(-7.7241);%fitted using reference data for u = 3:15 , Ay = 0:0.1:1
%% NO MORE FITTING, just use a different amount of Ay at different speed
if(scale_Ay_flag)
%     Ay = (del_y-1.61 + 0.2181*u0) / (-6.041); %For Ay>0, del_y<0 only!!!
    Ay = del_y;
%     if u0 > hi_low_boundary
%         tpk = tpk_hi;
%     end
else
     Ay = 0; %for u change
end
% 
% if u0 > hi_low_boundary
%         tbrk = tbrk_hi;
% end
if symbolic_flag
    t1=t;
    t2=t;
    t3=t;
    T = [];
    Z = [];
    
    if ~exist('brake_time')
        tbrk1 = (Au - u_really_slow)/amax;
    else
        tbrk1 = brake_time;
    end
    
else
    tbrk1 = (Au - u_really_slow)/amax; %amax = 2
    if tbrk1 < 0
        tbrk1  = 0
    end
    %     tbrk2 = 2; % braking from 5m/s to 1. Sean's model is potentially used
    %this is set in load consts
    tf = tpk + tbrk1 + tbrk2 -t0;
    
    T = linspace(0, tf,1000);
    Z = [];
    idx = find(T>=(tpk-t0),1)-1;
    t1 = T(1:idx);
    
    if tbrk1 == 0
        t2 = [];
        idx2 = idx;
    else
        idx2 = find(T>=(tpk+tbrk1-t0),1)-1;
        t2 = T((idx+1):idx2);
        t2 = t2 - (tpk-t0);
    end
    
    t3 = T(idx2+1:end);
    t3 = t3 - (tpk+tbrk1-t0);
end

%% specify ud, vd , rd in the first 3.25 seconds

t1_shifted = t1 + t0;

ud1 = (Au-u0)/tpk*t1_shifted+u0; %piecewise lienar
dud1 = (Au-u0)/tpk*ones(1,length(t1_shifted));

vd1 = zeros(size(t1_shifted));%1/13*-Ay*exp(-2.7*(t1_shifted - 1.5).^2).*(4*t1_shifted - 6);
dvd1 = zeros(size(t1_shifted));%1/13*-Ay*(4*exp(-(27*(t1_shifted - 3/2).^2)/10) - exp(-(27*(t1_shifted - 3/2).^2)/10).*(4*t1_shifted - 6).*((27*t1_shifted)/5 - 81/10));
% if symbolic_flag
 
%     syms rd(t)
%     r0 = 0;
% %     cond = [vd(0) == v0, ud(0)== u0, rd(0) == r0, u(0) == u0, v(0) == v0, r(0)==r0];
%     rd = dsolve(diff( rd, t ) == 2/Izz * ( ( lf + lr ) * Ca * ( ( vd1 - lr * rd )/u0 ) + lf * m * u0 * rd/2 + m/2 * lf * dvd1 ),rd(0)==r0);
%     rd1 = vpa(rd,4);
% else
tau = tpk;
%const_hd = 11*sqrt(2)*exp(-0.5)/2/tau;
%hd = -1/const_hd *exp(-(5.5*(t-tau/2)/tau)^2);
%hd = 
%following is diff -0.66/0.4/sqrt(2*pi)*exp(-0.5*(t-tau/2)^2/0.4^2), same
%as paper
rd1 = -Ay*((1125899906842624*exp(-((11*t1_shifted)/2 - (11*tau)/4).^2/tau^2).*((121*t1_shifted)/2 - (121*tau)/4))/(5311659529936249*tau));
%1.1 is a good relative value between the two references
drd1 = -Ay*((68116944363978752*exp(-((11*t1_shifted)/2 - (11*tau)/4).^2/tau^2))/(5311659529936249*tau) - (1125899906842624*exp(-((11*t1_shifted)/2 - (11*tau)/4).^2/tau^2).*((121*t1_shifted)/2 - (121*tau)/4).^2)/(5311659529936249*tau^3));
%% JL : brake 1 
% (Do not use this symbolic mode to generate braking trajectory!!!!!)
if symbolic_flag % do not use it!!!
    ud2 = Au - (Au-u_really_slow)*t2/tbrk1;
    dud2 = -(Au-u_really_slow)/tbrk1*ones(1,length(t2));
    vd2 = zeros(1,length(t2));
    dvd2 = zeros(1,length(t2));
    rd2 = zeros(1,length(t2));
    drd2 = zeros(1,length(t2));

else
    if tbrk1 == 0
        ud2 = []; dud2 = []; vd2 = []; dvd2 = []; rd2 = []; drd2 = [];
    else
        ud2 = Au - (Au-u_really_slow)*t2/tbrk1;
        dud2 = -(Au-u_really_slow)/tbrk1*ones(1,length(t2));
        vd2 = zeros(1,length(t2));
        dvd2 = zeros(1,length(t2));
        rd2 = zeros(1,length(t2));
        drd2 = zeros(1,length(t2));
    end
end
%% JL : brake 2
% ud3 = 5*exp(-3 * t3.^2);
% dud3 = 5*exp(-3 * t3.^2) * (-6) .* t3;
ud3 = t3*0;%u_really_slow*(1-t3).^4 + 15/(5/u_really_slow)*t3.*(1-t3).^3;
dud3 = t3*0;%u_really_slow*(t3 - 1).^3 - 9*t3*u_really_slow.*(t3 - 1).^2;



vd3 = zeros(1,length(t3));
dvd3 = zeros(1,length(t3));
rd3 = zeros(1,length(t3));
drd3 = zeros(1,length(t3));

%% Group
ud = [ud1 ud2 ud3];
dud=[dud1 dud2 dud3];
vd = [vd1 vd2 vd3];
dvd = [dvd1 dvd2 dvd3];
rd  = [rd1 rd2 rd3];
drd = [drd1 drd2 drd3];
U = [ud;vd;rd;dud;dvd;drd];
if ~symbolic_flag
    %integrate desired as reference and output as Z
    p = cumsum(rd)*(t1(2)-t1(1));
    x_dot = ud.*cos(p) - vd.*sin(p);
    y_dot = ud.*sin(p) + vd.*cos(p);
    Z = cumsum([x_dot;y_dot],2)*(t1(2)-t1(1));
    Z = [Z; p];
else
    Z = [int(rd1,t)-subs(int(rd1,t),t, 0);0;0];
end
end