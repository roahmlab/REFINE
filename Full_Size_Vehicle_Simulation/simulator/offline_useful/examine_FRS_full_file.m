clear;close all
frs_full = load('FRS_Rover_26-Jul-2021_no_force.mat');
info_file = load('dir_change_Ay_info.mat'); 
load_const;

dir_change = 1 ;
u0_fixed = 1;
u0 = u0_fixed;%randRange(u0_fixed-1,u0_fixed+1 );
Au = u0;
t0 = 0;
t0_idx = t0/1.5+1;
del_y = 0.3;
t = linspace(0, tpk+Au/amax+tbrk2);
syms_flag = 0;

[~,u0_idx]=min(abs(info_file.u0_vec - u0));
FRS_u0 = frs_full.M_mega{u0_idx};
if dir_change
    FRS_file = FRS_u0('dir');
    FRS_file = FRS_file{1};
    
    [T_go,U_go,Z_go] = gaussian_one_hump_parameterized_traj_with_brake(t0,del_y,Au,u0,t,syms_flag,1);
else
    FRS_u0('lantb')
    [T_go,U_go,Z_go] = gaussian_T_parameterized_traj_with_brake(t0,del_y,Au,u0,t,syms_flag,1);
end
A_go = highway_cruising_7_state_agent('use_rover_controller',false);
v0 = 0;%randRange( -r0v0_limit(u0_fixed),r0v0_limit(u0_fixed));
r0 = 0;%randRange( -r0v0_limit(u0_fixed),r0v0_limit(u0_fixed));
w0 = u0;

z_0 = [0;0;0;u0;v0;r0;w0] ;
A_go.reset(z_0) ;
A_go.move(T_go(end),T_go,U_go,Z_go) ;


% plot(Z_go(1,:), Z_go(2,:));
%

X = A_go.state';
T = A_go.time';
[~, Fxf, Fyf, Fyr ,~ ,~, ~, ~] = cellfun(@(t,x)  A_go.dynamics(t,x.',T_go,U_go,Z_go), num2cell(T), num2cell(X,[2]),'uni',0);

figure(1); clf;hold on;
%in2 states: x, y, psi, u, v, r,u0,v0, r0,t0 ,Au,   Ay,   t
%            1  2  3    4  5  6  7  8  9  10  11    12    13
for i =1:length(FRS_file.vehRS_save) %change to 65 if only want to plot to peak
    p_unsliced = plotFilled(FRS_file.vehRS_save{i},[1 2],'b');
    p_unsliced.FaceAlpha = 0.05;
    p_unsliced.EdgeAlpha = 0.1;
    
    zonoslice = zonotope_slice(FRS_file.vehRS_save{i}, [7;8;9;12], [u0;v0;r0;del_y]);
    zonoslice = deleteAligned(zonoslice);
    
    p_FRS = plotFilled(zonoslice, [1, 2], 'g');
    p_FRS.FaceAlpha = .2;
    p_FRS.EdgeAlpha = 0.01;
    
%     ft = deleteAligned(project(FRS_file.vehRS_save{i},3));
%     headingc = center(ft);
%     headingg = abs(generators(ft));
%     if isempty(headingg)
%         headingg = 0;
%     end
%     gen = [car_length car_width] + obs_localize_err; %footprint and obstacle localization error
    %% footprint error
%     [len , width]=get_footprint_gen(gen,headingg);% make the footprint wiggle to get largest footprint
%     h =  headingc;
%     ego_gen = [[cos(h)*len; sin(h)*len], [sin(-h)*width; cos(-h)*width]];
%     gen = zeros(dim,2);gen(1:2,1:2) = ego_gen;
%     ftprint_zono  = zonotope([zeros(dim,1), gen]);
%     slicewithft = zonoslice+ftprint_zono;
    %% prediction err
%     closest_time = i/10;
%     [~,error_idx ] = min(abs(predict_err.error_time_vector - closest_time));
%     lon_gen = max(abs(predict_err.lon_err_lan_min(error_idx)), predict_err.lon_err_lan_max(error_idx));
%     lat_gen = max(abs(predict_err.lat_err_lan_min(error_idx)), predict_err.lat_err_lan_max(error_idx));
%     head_gen = max(abs(predict_err.head_err_lan_min(error_idx)), predict_err.head_err_lan_max(error_idx));
    
%     gen = [lon_gen lat_gen];
%     [len , width]=get_footprint_gen(gen,head_gen);% make the footprint wiggle to get largest footprint
%     h =  headingc;
%     ego_gen = [[cos(h)*len; sin(h)*len], [sin(-h)*width; cos(-h)*width]];
%     gen = zeros(dim,2);gen(1:2,1:2) = ego_gen;
%     predict_err_zono  = zonotope([zeros(dim,1), gen]);
%     slicewithft_predict_err = slicewithft+predict_err_zono;

    
    
%     p3 = plotFilled(slicewithft_predict_err, [1, 2], 'g');
%     p3.FaceAlpha = 0.1;
%     p3.EdgeAlpha = 0.1;
end
axis equal
grid on
grid(gca,'minor')
ylim([-0.5,1.5]);
 A_go.plot();


% xlim([-5,130])


% yline(2,'LineWidth',2)
% yline(6,'LineWidth',2)
% yline(-2,'LineWidth',2)
drawnow
figure(2);clf;
text_size = 10;
% figure(1);clf;
subplot(4,1,1);hold on;axis equal
% A_go.plot();
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
drawnow