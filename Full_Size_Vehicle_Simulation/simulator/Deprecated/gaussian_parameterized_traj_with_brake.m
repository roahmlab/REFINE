function [T U Z] = gaussian_parameterized_traj_with_brake(del_y,Au,u0,t,symbolic_flag, scale_Ay_flag)
%In symbolic = 0, t argument will be ignored
load_const
% tbrk = Au/amax;
%  Ay= del_y;%/(-5.363);%/(1.35-1.057*u0);%(del_y+0.284-0.1027*u0) / (0.5538-0.94*u0);
% Ay = (del_y-4.6595+0.5324*u0)/(-7.8876);%fitted using sim data
% Ay = (del_y-3.8579+0.4422*u0)/(-7.7241);%fitted using reference data for u = 3:15 , Ay = 0:0.1:1
%% NO MORE FITTING, just use Ay or del_y, same shit
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
    T = [];
    Z = [];
else
    tf= tbrk + tpk;
    
    T = linspace(0, tf,1000);
    Z = [];
    idx = find(T<tpk);
    idx = idx(end);
    
    t1 = T(1:idx);
    t2 = T(idx+1:end);
    t2= t2 - tpk;
end
%% specify ud, vd , rd in the first 3.25 seconds
ud1 = (Au-u0)/tpk*t1+u0; %piecewise lienar
dud1 = (Au-u0)/tpk*ones(1,length(t1));
vd1 = Ay*exp(-3*(t1 - 3/2).^2).*(6*t1 - 9);
dvd1 = 6*Ay*exp(-3*(t1 - 3/2).^2) -Ay *  exp(-3*(t1 - 3/2).^2).*(6*t1 - 9).^2;
% if symbolic_flag
 
%     syms rd(t)
%     r0 = 0;
% %     cond = [vd(0) == v0, ud(0)== u0, rd(0) == r0, u(0) == u0, v(0) == v0, r(0)==r0];
%     rd = dsolve(diff( rd, t ) == 2/Izz * ( ( lf + lr ) * Ca * ( ( vd1 - lr * rd )/u0 ) + lf * m * u0 * rd/2 + m/2 * lf * dvd1 ),rd(0)==r0);
%     rd1 = vpa(rd,4);
% else
rd1 = Ay/1.1*exp(-3*(t1 - 3/2).^2).*(6*t1 - 9); %1.1 is a good relative value between the two references
%% specify braking trajectory
% rd2 = zeros(1,length(t2));
ud2 = Au-Au*t2/tbrk;
dud2 = -Au/tbrk*ones(1,length(t2));
vd2 = zeros(1,length(t2));
dvd2 = zeros(1,length(t2));
rd2 = zeros(1,length(t2));
%% Group
ud = [ud1 ud2];
dud=[dud1 dud2];
vd = [vd1 vd2];
% vd(210:end) = vd2(end);
dvd = [dvd1 dvd2];
% dvd(210:end) = dvd2(end);
rd  = [rd1 rd2];
% rd(210:end) = rd2(end);
U = [ud;vd;rd;dud;dvd];
if ~symbolic_flag
    %integrate desired as reference and output as Z
    p = cumsum([rd1 rd2])*(t1(2)-t1(1));
    x_dot = ud.*cos(p) - vd.*sin(p);
    y_dot = ud.*sin(p) + vd.*cos(p);
    Z = cumsum([x_dot;y_dot],2)*(t1(2)-t1(1));
end
end