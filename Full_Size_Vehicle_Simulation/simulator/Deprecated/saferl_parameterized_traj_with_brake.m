% syms t k_pk kv tf tpk
%kv - 1 < k_pk< kv+ 0.5
function [T,F,Z]=saferl_parameterized_traj_with_brake(Ay, Au,av0,au0,u0,v0,r0,t,symbolic_flag)
% Ay describe the amount of y want to be completed at the end of the 3.5
% seconds. 
% Au describes the amount of velocity change at the end of 3.25 seconds
% u0 initial velocity
% v0 initial lateral velocity
% r0 initial heading rate
% au0 initial u directional acc
% av0 initial v dir acc
load_const
% if symbolic_flag
%     syms Ay Au av0 au0 u0 v0 r0 t
% end
    tbrk = Au/amax;

    ka = au0;
    k_pk = Au;
    kv = u0;
    % 
    tf= tbrk + tpk;

    
    T = linspace(0, tf,1000);
    idx = find(T<tpk);
    idx = idx(end);
    
    
    t1 = T(1:idx);
    t2 = T(idx+1:end);
if symbolic_flag
    t1=t;
    t2=t;
end

%% u dir
delv1=k_pk-kv-ka*tpk;
c31 =tpk;
dela1 = -ka;
c121=1/c31^3*[-12 6*c31; 6*c31 -2*c31^2]*[delv1;dela1];
c11= c121(1); c21= c121(2);
f1= c11/6*t1.^3+c21/2*t1.^2+ka*t1 + kv; %velocity ud from [0,tpk]
f1_a = c11/2*t1.^2 + c21*t1 + ka;
%% braking manever
delv2 = -k_pk;
c32 = tf-tpk;
 dela2=0;
c122= 1/c32^3*[-12 6*c32; 6*c32 -2*c32^2]*[delv2;dela2];
c12=c122(1);c22=c122(2);
f2 = c12/6*(t2-tpk).^3+c22/2*(t2-tpk).^2+ k_pk;%ka*t2 ;
f2_a = c12/2*(t2-tpk).^2 + c22*(t2-tpk);
ud_all = [f1 f2];
%  = cumsum(ud)*(t1(2)-t1(1));
%% v dir
a0 = av0; 
del_p = Ay-v0*tpk-a0*tpk^2/2;
del_v = -v0-a0*tpk ;
consts =  1/tpk^5*[320 -120*tpk;
                -200*tpk 72*tpk^2;
                40*tpk^2 -12*tpk^3]*[del_p; del_v];
alpha = consts(1);
beta = consts(2);
gam = consts(3);


f3_p = alpha/120*t1.^5+beta/24*t1.^4+gam/6*t1.^3 +a0/2*t1.^2+v0*t1;
f3   = alpha/24*t1.^4+beta/6*t1.^3+gam/2*t1.^2 +a0*t1+v0;
f3_a = alpha/6*t1.^3 +beta/2*t1.^2+gam*t1 +a0;

f4 = zeros(1,length(t2));
f4_a = zeros(1,length(t2));
vd_all = [f3 f4];

%% rd
if symbolic_flag
    rd_all = [];
    for i = 1:2
        syms rd(t)
        ud = ud_all(i);
        vd = vd_all(i);
    %     cond = [vd(0) == v0, ud(0)== u0, rd(0) == r0, u(0) == u0, v(0) == v0, r(0)==r0];
        rd = dsolve(diff( rd, t ) == 2/Izz * ( ( lf + lr ) * Ca * ( ( vd - lr * rd( t ) )/ud) + ...
        lf * m * ud * rd( t )/2  + m/2 * diff( vd, t )),rd(0)==r0);
        rd = vpa(rd,4);
        rd_all = [rd_all rd];
    end
else
    rd1 = -Ay/10*exp(-3*(t1 - 3/2).^2).*(6*t1 - 9);
    rd2 = zeros(1,length(t2));
    rd_all = [rd1 rd2];
end



% if symbolic_flag
% %in2 states: x, y, u, v, psi, r, Ay, Au,av0,au0,u0,v0,r0, t 
%             %1  2  3  4  5    6  7    8  9  10  11 12  13 14
%     syms x u v 
% end
Z=[f3_p ones(1,length(t2))*f3_p(end);
    f1_a f2_a;  
    f3_a f4_a;];


F= [ud_all;vd_all; rd_all];
% %%
% figure(11);clf;hold on;
% plot(T,F(1,:))
% % plot(t2+tpk,f2)
% figure(12);clf;hold on;
% plot(T(1:end-1),diff(F(1,:))./diff(T));
% 
% figure(13);clf;hold on;
% plot(T,F(2,:));


%this thing here cannot deal with dynamic where the first 0.5s the desired
%velocity needs to be something else, it has to be piecewise, tanh a better
%option.
%vx + (2*vx*t^3)/(tf - tpk)^3 - (vx*t^2*(6*tf - 6*tpk))/(2*(tf - tpk)^3)