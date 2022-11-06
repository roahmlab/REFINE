%% description
clear; 
warning('ON', 'All')
load('lane_change_Ay_info.mat');
load_const;

u_color =[0.0    0.4470    0.7410];
vr_color =[0.8500    0.3250    0.0980];
%%
x0 = 0;
y0 = 0;
h0 =0;

% trajectory parameters % 0.5 0.8 1.0
u0 = 6;start_idx = 0;
w0 =0.5;
Au =  1.2;
[~,idx] = min(abs(u0-u0_vec));
v0 =0;%v_value_vec(idx,start_idx)%+r0v0_limit(idx);
r0 =0;%r_value_vec(idx,start_idx)%-r0v0_limit(idx);
del_y_arr = linspace(0,Ay_vec(idx),9);%    del_y_arr = -0.8:0.2:0;
del_y_arr = del_y_arr(2:2:end);

Ay = 0.6;%del_y_arr(end)+(rand*2-1)*(del_y_arr(end)- del_y_arr(end-1))/2;

% Au = 19;
% av0 = 0;
% au0 = 0;
t0 = 0.75*start_idx;
t_f = 4.4;%tpk_dir +tbrk-0.1; %3.25+4
%% automated from here
A_go = highway_cruising_7_state_agent('use_rover_controller',false) ;

z_0 = [x0;y0;h0;u0;v0;r0;w0] ;
A_go.reset(z_0) ;

syms_flag = 0; % if set to one will make t symbolic, for dynamics generation
% syms t Au u0 %Ay av0 au0 v0 r0 
 t= 0; scale_Ay_flag = 1; % enable lane change manuver by setting this to 1
[T_go,U_go,Z_go] = gaussian_T_parameterized_traj_with_brake(t0,Ay,Au,u0,t,syms_flag, scale_Ay_flag);
% [T_go,U_go,Z_go] = gaussian_one_hump_parameterized_traj_with_brake(t0,Ay,Au,u0,t,syms_flag, scale_Ay_flag);


%%

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

A_go.move(t_f,T_go,U_go,Z_go) ; % this may not be good


X = A_go.state';
T = A_go.time';

%get tire forces
%  [dzdt, Fxwf, Fywf, Fywr ,delta ,w_cmd, v, r, Fxwr]
      [~, Fxwf, Fywf, Fywr ,delta ,w_cmd, ~, ~, Fxwr] = cellfun(@(t,x)  A_go.dynamics(t,x.',T_go,U_go,Z_go), num2cell(T), num2cell(X,[2]),'uni',0);
%% plot


text_size = 10;
figure(1);clf;
subplot(5,1,1);hold on;axis equal
A_go.plot();
plot(Z_go(1,:), Z_go(2,:));xlabel('x');ylabel('y')
set(gca,'FontSize',text_size) % Creates an axes and sets its FontSize to 18

subplot(5,1,2);hold on;%axis equal
plot(T_go,U_go(2,:));
plot(T_go,U_go(3,:));
plot(A_go.time,A_go.state(5,:));
plot(A_go.time,A_go.state(6,:));
legend('vd','rd','v','r');xlabel('t(s)');ylabel('m/s or rad/s')
set(gca,'FontSize',text_size) % Creates an axes and sets its FontSize to 18

subplot(5,1,3);hold on;%axis equal
plot(T_go,U_go(1,:));
plot(A_go.time,A_go.state(4,:));
legend('ud','u');xlabel('t(s)');ylabel('m/s')
set(gca,'FontSize',text_size) % Creates an axes and sets its FontSize to 18

subplot(5,1,4);hold on;%axis equal
plot( A_go.time,cell2mat(Fxwf));
plot( A_go.time,cell2mat(Fywf));
plot( A_go.time,cell2mat(Fywr));
legend('Fxwf','Fywf','Fywr'); xlabel('t(s)');ylabel('N')
set(gca,'FontSize',text_size) % Creates an axes and sets its FontSize to 18

subplot(5,1,5);hold on;%axis equal
plot( A_go.time,cell2mat(delta));
plot( A_go.time,cell2mat(w_cmd));
legend('\delta','wcmd'); xlabel('t(s)');
set(gca,'FontSize',text_size) % Creates an axes and sets its FontSize to 18
[delta_sparse,w_cmd_sparse]=match_trajectories(T_go,A_go.time, cell2mat(delta)',A_go.time,cell2mat(w_cmd)');
cmd_data = [T_go' delta_sparse' w_cmd_sparse'];
writematrix(cmd_data,"manual_command_two_hump_Ay="+num2str(Ay)+"_u="+num2str(u0)+".csv") 
return
%% plotting for FL zono
[T_go,U_go,Z_go] = gaussian_one_hump_parameterized_traj_with_brake(t0,0,Au,u0,t,syms_flag, scale_Ay_flag);

close all;
text_size = 15;
toffset =0;
tend = toffset+11;
figure('Renderer', 'painters', 'Position', [0 0 700 500/2*3]);
subplot(3,1,1);hold on;%axis equal 
% speed change
yyaxis left
plot(T_go,U_go(1,:),'Color',u_color);
ylabel('m/s')
yyaxis right
plot(T_go,U_go(2,:),'Color',vr_color);
plot(T_go,U_go(3,:),'Color',vr_color);
ylabel('m/s or rad/s')
% plot(A_go.time,A_go.state(4,:));
% plot(T_go,U_go(3,:));
% plot(E.Vehicle(1).time-toffset,E.Vehicle(1).state(4,:));
% plot(E.Vehicle(1).time-toffset,E.Vehicle(1).state(5,:));
% plot(E.Vehicle(1).time-toffset,E.Vehicle(1).state(6,:));
% legend('v','r');xlabel('t(s)');ylabel('m/s or rad/s')
legend('$u_\textrm{des}$', '$v_\textrm{des}$','$r_\textrm{des}$','Interpreter','latex');xlabel('Speed Change t(s)');
grid on;grid minor;
xlim([0,tend-toffset]);
set(gca,'FontSize',text_size) % Creates an axes and sets its FontSize to 18


[T_go,U_go,Z_go] = gaussian_one_hump_parameterized_traj_with_brake(t0,Ay,u0,u0,t,syms_flag, scale_Ay_flag);

subplot(3,1,2);hold on;%axis equal
yyaxis left
plot(T_go,U_go(1,:),'Color',u_color);
ylabel('m/s')
yyaxis right
plot(T_go,U_go(2,:),'Color',vr_color);
plot(T_go,U_go(3,:),'Color',vr_color);
% plot(A_go.time,A_go.state(5,:));
% plot(A_go.time,A_go.state(6,:));
% yyaxis left
% plot(E.Vehicle(1).time-toffset,E.Vehicle(1).state(8,:));
% ylabel('m');
% yyaxis right
% plot(E.Vehicle(1).time-toffset,E.Vehicle(1).state(9,:));
% ylabel('rad')
% plot(E.Vehicle(1).time,E.Vehicle(1).state(10,:));
legend('$u_\textrm{des}$', '$v_\textrm{des}$','$r_\textrm{des}$','Interpreter','latex');xlabel('Direction Change t(s)');ylabel('m/s or rad/s')
xlim([0,tend-toffset]);
grid on; grid minor;
set(gca,'FontSize',text_size) % Creates an axes and sets its FontSize to 18

[T_go,U_go,Z_go] = gaussian_T_parameterized_traj_with_brake(t0,Ay,u0,u0,t,syms_flag, scale_Ay_flag);
A_go.reset(z_0) ;
A_go.move(t_f,T_go,U_go,Z_go) ;

subplot(3,1,3);hold on;%axis equal
% plot(A_go.time,A_go.state(5,:));
% plot(A_go.time,A_go.state(6,:));
yyaxis left
plot(T_go,U_go(1,:),'Color',u_color);
ylabel('m/s')
yyaxis right
plot(T_go,U_go(2,:),'Color',vr_color);
plot(T_go,U_go(3,:),'Color',vr_color);
% yyaxis left
% plot(E.Vehicle(1).time-toffset,E.Vehicle(1).state(8,:));
% ylabel('m');
% yyaxis right
% plot(E.Vehicle(1).time-toffset,E.Vehicle(1).state(9,:));
% ylabel('rad')
% plot(E.Vehicle(1).time,E.Vehicle(1).state(10,:));
legend('$u_\textrm{des}$', '$v_\textrm{des}$','$r_\textrm{des}$','Interpreter','latex');xlabel('Lane Change t(s)');ylabel('m/s or rad/s');
xlim([0,tend-toffset]);
grid on; grid minor;
set(gca,'FontSize',text_size) % Creates an axes and sets its FontSize to 18
saveas(gcf,'Zono_FL_Parameterization_All.pdf')
return
%% plotting for Ford zono
close all;
text_size = 15;
toffset =4.6-1+1.51;
t_end = toffset + 8;
figure('Renderer', 'painters', 'Position', [0 0 700 500]);
% subplot(2,1,1);hold on;axis equal
% % A_go.plot();
% plot(E.Vehicle(1).state(1,:),E.Vehicle(1).state(2,:));xlabel('x');ylabel('y')
% set(gca,'FontSize',text_size) % Creates an axes and sets its FontSize to 18

subplot(2,1,1);hold on;%axis equal
% plot(T_go,U_go(2,:));
% plot(T_go,U_go(3,:));
yyaxis right
plot(E.Vehicle(1).time-toffset,E.Vehicle(1).state(5,:),'Color',vr_color);
plot(E.Vehicle(1).time-toffset,E.Vehicle(1).state(6,:),'Color',vr_color);
ylabel('m/s or rad/s')
yyaxis left
plot(E.Vehicle(1).time-toffset,E.Vehicle(1).state(4,:),'Color',u_color);
ylim([28.0, 29])

% legend('v','r');xlabel('t(s)');ylabel('m/s or rad/s')
legend('$u$','$v$','$r$','Location','southeast','Interpreter','latex');xlabel('t(s)');ylabel('m/s')
grid on;grid minor;
xlim([0,t_end-toffset]);
set(gca,'FontSize',text_size) % Creates an axes and sets its FontSize to 18

subplot(2,1,2);hold on;%axis equal
% plot(T_go,U_go(1,:));
yyaxis left
plot(E.Vehicle(1).time-toffset,E.Vehicle(1).state(8,:));
ylabel('m');
yyaxis right
plot(E.Vehicle(1).time-toffset,E.Vehicle(1).state(9,:));
ylabel('rad')
% plot(E.Vehicle(1).time,E.Vehicle(1).state(10,:));
legend('path offset','path angle','s','Location','southeast');xlabel('t(s)');
xlim([0,t_end-toffset]);
grid on; grid minor;
set(gca,'FontSize',text_size) % Creates an axes and sets its FontSize to 18

saveas(gcf,'Ford_Zono_Parameterization_tracking.pdf')
return
%%
figure(4);clf;hold on;
 real_states =reshape(cell2mat(dzdt),6,[]);
% ss_est =reshape(cell2mat(dz),6,[]);
for i = 1:6
    subplot(6,1,i)
    cla;hold on;
    plot( A_go.time,real_states(i,:))
    plot( A_go.time,ss_est(i,:))
    legend('model 1','model Sean')
end
return
% plot( A_go.time,reshape(cell2mat(dzdt),6,[]));
% plot( A_go.time,reshape(cell2mat(dz),6,[]));
% return
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

