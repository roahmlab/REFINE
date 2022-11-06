clear;close all


% FRS_zero_ini = load("./data/lane_change/zero_initial_condition/lane_change_t0=0_u=1_Ay=1,0.3.mat"); % I only have this one in the repo, genearte your own will take 90 seconds each
% FRS_vary_ini = load("./data/lane_change/various_initial_condition/lane_change_t0=0_u=1_Ay=1,0.3.mat"); % I only have this one in the repo, genearte your own will take 90 seconds each
FRS_zero_ini = load("./data/dir_change/zero_initial_condition/dir_change_t0=0_u=1_Ay=2,0.45.mat"); % I only have this one in the repo, genearte your own will take 90 seconds each
FRS_vary_ini = load("./data/dir_change/various_initial_condition/dir_change_t0=0_u=1_Ay=2,0.45.mat"); % I only have this one in the repo, genearte your own will take 90 seconds each
load lane_change_Ay_info.mat
predict_err = load('lateral_prediction_error.mat');
dim = 14;
load_const;
u0_fixed = 1;
u0 = u0_fixed;%randRange(u0_fixed-1,u0_fixed+1 );
Au = u0;
t0 = 0;
v0 = 0;%0.059;%randRange( -r0v0_limit(u0_fixed),r0v0_limit(u0_fixed));
r0 = 0;%0.59;%randRange( -r0v0_limit(u0_fixed),r0v0_limit(u0_fixed));
w0= u0;
% del_y = linspace(0,Ay_vec(u0_fixed),9);
del_y = 0.59;%randRange(del_y(7),del_y(9));
% t = linspace(0, tpk+Au-u_really_slow0/amax+tbrk2);
syms_flag = 0;
[T_go,U_go,Z_go] = gaussian_one_hump_parameterized_traj_with_brake(t0,del_y,Au,u0,[],syms_flag,1);
% [T_go,U_go,Z_go] = gaussian_T_parameterized_traj_with_brake(t0,del_y,Au,u0,t,syms_flag,1);
cars = cell(0);

%

figure(1); clf;

%     if j == 1
        
        FRS_file_zero = FRS_zero_ini;
%     else
        
        FRS_file = FRS_vary_ini;
%     end

%in2 states: x, y, psi, u, v, r,u0,v0, r0,t0 ,Au,   Ay,   t
%            1  2  3    4  5  6  7  8  9  10  11    12    13
for i =1 :length(FRS_file.vehRS_save) %change to 65 if only want to plot to peak
    subplot(5,1,1);hold on ; %no error
%     p_unsliced = plotFilled(FRS_file.vehRS_save{i},[1 2],'b');
%     p_unsliced.FaceAlpha = 0.05;
%     p_unsliced.EdgeAlpha = 0.5;
%     
    zonoslice = zonotope_slice(FRS_file_zero.vehRS_save{i}, [7;8;9;12], [u0;v0;r0;del_y]);
    zonoslice = deleteAligned(zonoslice);
    
    p_FRS = plotFilled(zonoslice, [1, 2], 'r');
    p_FRS.FaceAlpha = .2;
    p_FRS.EdgeAlpha = 0.5;
    ylabel('CORA error');
    %%
    subplot(5,1,2);hold on;
    zonoslice_ini = zonotope_slice(FRS_file.vehRS_save{i}, [7;8;9;12], [u0;v0;r0;del_y]);
    zonoslice_ini = deleteAligned(zonoslice_ini);
    
    p_FRS = plotFilled(zonoslice_ini, [1, 2], 'r');
    p_FRS.FaceAlpha = .2;
    p_FRS.EdgeAlpha = 0.5;
    ylabel('previous plan model error');
    %% use heading error from the one with varied initial coniditon
    ft = deleteAligned(project(FRS_file.vehRS_save{i},3));
    headingc = center(ft);
    headingg = abs(generators(ft));
    if isempty(headingg)
        headingg = 0;
    end
    
    %% footprint error
    subplot(5,1,3);hold on;
    gen = [car_length car_width] ; %footprint
    [len , width]=get_footprint_gen(gen,headingg);% make the footprint wiggle to get largest footprint
    h =  headingc;
    ego_gen = [[cos(h)*len; sin(h)*len], [sin(-h)*width; cos(-h)*width]];
    gen = zeros(dim,2);gen(1:2,1:2) = ego_gen;
    ftprint_zono  = zonotope([zeros(dim,1), gen]);
    slicewithft = zonoslice+ftprint_zono;
    
    p_FRS = plotFilled(slicewithft, [1, 2], 'c');
    p_FRS.FaceAlpha = .2;
    p_FRS.EdgeAlpha = 0.5;
    ylabel('footprint error');

    %% obs detection error
    subplot(5,1,4);hold on;
    gen = [obs_localize_err obs_localize_err];
    [len , width]=get_footprint_gen(gen, 0);% make the footprint wiggle to get largest footprint
    h =  headingc;
    ego_gen = [[cos(h)*len; sin(h)*len], [sin(-h)*width; cos(-h)*width]];
    gen = zeros(dim,2);gen(1:2,1:2) = ego_gen;
    localize_error  = zonotope([zeros(dim,1), gen]);
    slicewithftobs = zonoslice+localize_error;
       
    p_FRS = plotFilled(slicewithftobs, [1, 2], 'm');
    p_FRS.FaceAlpha = .2;
    p_FRS.EdgeAlpha = 0.5;
    ylabel('obstacle error')
    
    %% prediction err
     subplot(5,1,5);hold on;
    closest_time = i/10;% assume it is 0.1 s per zonotope
    [~,error_idx ] = min(abs(predict_err.error_time_vector - closest_time));
    lon_gen = max(0, predict_err.lon_err_lan_max(error_idx));
    lat_gen = max(abs(predict_err.lat_err_lan_min(error_idx)), predict_err.lat_err_lan_max(error_idx));
    head_gen = max(abs(predict_err.head_err_lan_min(error_idx)), predict_err.head_err_lan_max(error_idx));
%     lon_gen =0.001;
%     lat_gen =0.001;
%     head_gen = 0.001;
    
    gen = [lon_gen lat_gen];
    [len , width]=get_footprint_gen(gen,head_gen);% make the footprint wiggle to get largest footprint
    h =  headingc;
    ego_gen = [[cos(h)*len; sin(h)*len], [sin(-h)*width; cos(-h)*width]];
    gen = zeros(dim,2);gen(1:2,1:2) = ego_gen;
    predict_err_zono  = zonotope([zeros(dim,1), gen]);
    slicewithft_predict_err = zonoslice+predict_err_zono;
   
  
    p3 = plotFilled(slicewithft_predict_err, [1, 2], 'g');
    p3.FaceAlpha = 0.1;
    p3.EdgeAlpha = 0.5;
    
    ylabel('prediction model error');
end
 
%   axis equal
%     grid on
%     grid(gca,'minor')
%     xlim([-0.3,3]);
%     title('Purple: unsliced, Red: sliced')
%     ylabel(convertStringsToChars(num2str(j)+" Ay bin"))
    
   
%     return 
for car_idx = 1:5
    subplot(5,1,car_idx)
A_go = highway_cruising_7_state_agent('use_rover_controller',false);


z_0 = [0;0;0;u0;v0;r0;w0] ;
A_go.reset(z_0) ;
A_go.move(T_go(end),T_go,U_go,Z_go) ;
A_go.plot
ylim([-0.2, 1])
   axis equal
    grid on
    grid(gca,'minor')
% cars{end+1} = A_go;
end
X = A_go.state';
T = A_go.time';
[~, Fxf, Fyf, Fyr ,~ ,~, ~, ~] = cellfun(@(t,x)  A_go.dynamics(t,x.',T_go,U_go,Z_go), num2cell(T), num2cell(X,[2]),'uni',0);


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