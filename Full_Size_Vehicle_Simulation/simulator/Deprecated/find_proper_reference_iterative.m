%% description

% step1: define reference the old way, 
% step2: adjust vd rd to a best fitting ref(3 iter)
% step3: adjust Ay to proper lane width(2 iter)
close all; clear 

load_const;

x0 = 0;
y0 = 0;
h0 =0;

% trajectory parameters
u0 = 20 ;
v0 = 0;
Ay = 0.1;
% Au = 12;
% av0 = 0;
% au0 = 0;
r0 = 0;
t_f = tpk +tbrk; %3.25+4
%% automated from here
A_go = highway_cruising_6_state_agent() ;

z_0 = [x0;y0;h0;u0;v0;r0] ;

syms_flag = 0; % if set to one will make t symbolic, for dynamics generation
% syms t Au u0 %Ay av0 au0 v0 r0 
Au = u0; t= 0; scale_Ay_flag = 1; % enable lane change manuver by setting this to 1
[T_go,U_go,Z_go] = gaussian_parameterized_traj_with_brake(Ay,Au,u0,t,syms_flag, scale_Ay_flag);
%% run iterative find vd rd exercise
for i = 1:10
    A_go.reset(z_0) ;
% figure(2); hold on; axis auto
if syms_flag
%     ezplot(U_go(1,1), [0,tpk]);
    ezplot(Z_go(1,1), [0,tpk]);
    ezplot(U_go(2,1), [0,tpk]);
    ezplot(U_go(3,1), [0,tpk]);
    ezplot(U_ga(3,1), [0,tpk]);
    ezplot(U_ga(5,1), [0,tpk]);
    
    legend('int(vd)','vd','rd','vgaussian','rgaussian')
else
%     plot(T_go, U_go)
%     plot(Z_go(1,:), Z_go(2,:))
end

A_go.move(tpk,T_go,U_go) ; % this may not be good


X = A_go.state';
T = A_go.time';

%get tire forces
[~,Fxf,Fyf] = cellfun(@(t,x)  A_go.dynamics(t,x.',T_go,U_go,Z_go), num2cell(T), num2cell(X,[2]),'uni',0);
%% plot
text_size = 10;
figure(1);clf;
subplot(4,1,1);hold on;axis equal
A_go.plot();
plot(Z_go(1,:), Z_go(2,:));xlabel('x');ylabel('y')
set(gca,'FontSize',text_size) % Creates an axes and sets its FontSize to 18

subplot(4,1,2);hold on;%axis equal
plot(T_go,U_go(2,:));
plot(T_go,U_go(3,:));
plot(A_go.time,A_go.state(5,:));
plot(A_go.time,A_go.state(6,:));
legend('vd','rd','v','r');xlabel('t(s)');ylabel('m/s or rad/s')
set(gca,'FontSize',text_size) % Creates an axes and sets its FontSize to 18

subplot(4,1,3);hold on;%axis equal
plot(T_go,U_go(1,:));
plot(A_go.time,A_go.state(4,:));
legend('ud','u');xlabel('t(s)');ylabel('m/s')
set(gca,'FontSize',text_size) % Creates an axes and sets its FontSize to 18

subplot(4,1,4);hold on;%axis equal
plot( A_go.time,cell2mat(Fxf));
plot( A_go.time,cell2mat(Fyf));
legend('Fxf','Fyf'); xlabel('t(s)');ylabel('N')
set(gca,'FontSize',text_size) % Creates an axes and sets its FontSize to 18

%% reset reference to current values
vdrd = match_trajectories (T_go,T',X(:,5:6)');
U_go(2:3,:) = vdrd;
U_go(4,:) = [diff(U_go(2,:))/(T_go(2)-T_go(1)) 0];
U_go(5,:) = [diff(U_go(3,:))/(T_go(2)-T_go(1)) 0];
end
return
%% below code for finding a mapping between u and delta y, not important.
% evantually went with not the following mapping but different range for Ay
% for different bins
figure(2);
%Ay = 0.5
u= [3 5 8 10 12];
dely=[1.36 2.27 3.63 4.54 5.45]; %ITS LINEAR!!!!!
plot(u, dely)
figure(3);
%u = 10;
Ay = [0.1 0.3 0.5 1];
dely =[0.92 2.76 4.54 8.54];
plot(Ay,dely)
t=0;syms_flag = 0;
C=[];
d=[];
for u = 4:2:15
    u
    for Ay = 0:0.1:0.8
        C = [C;Ay u 1];
        [T_go,U_go,Z_go] = gaussian_parameterized_traj_with_brake(Ay,u,u,t,syms_flag,1);
        z_0 = [0;0;0;u;0;0] ;
%          A_go.reset(z_0) ;
%          A_go.move(T_go(end),T_go,U_go) ;
         del_y = Z_go(2,end);
%        del_y = A_go.state(2,end);%
        d = [d;del_y];
    end
end
%%
valid_idx = abs(d) < 5;
% fun = @(x,xdata)x(1)*exp(x(2)*xdata);
% x = lsqlin(C(valid_idx,:),d(valid_idx),[],[])
%x =   [-6.9873; -0.5085 ;3.9522];
fitobject = fit([C(valid_idx,1),C(valid_idx,2)],d(valid_idx),'poly11')
%%
figure(3);clf;hold on;

scatter3(C(valid_idx,1),C(valid_idx,2),d(valid_idx));
Ay = -1:0.1:1;
u = 1:15;
[AY,U]= meshgrid(Ay, u);
Z=fitobject(AY, U);
surf(AY, U, Z);
%%

