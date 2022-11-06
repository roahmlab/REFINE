% close all, clc
method = 0; % 0: flzono, 1: gpops
addpath(genpath('/installs/cpp-opt'))
addpath(genpath('/data'))
addpath(genpath('/simulator'))
if method == 0 % flzono
%     frs_filename = 'FRS_Rover_22-Jan-2022_no_force.mat' ;
%     frs_filename = 'car_frs.mat';
    frs_filename = 'FRS_Rover_30-Aug-2022_no_force.mat';
%     frs_filename = 'FRS_Rover_29-Sep-2022_no_force_dt=0.005.mat';
    if ~exist('frs','var')
        disp('Loading frs')
        frs = load(frs_filename) ;
    else
        disp('table already loaded') ;
    end
else % gpops
    frs = [];
end


%%
%Some heading changes, so they dont face weird directions, can use
%quiver(x,y,cos(h), sin(h)),'AutoScale_Factor') to plot waypoints. 
% sim_data = sim_data.save_struct;
plot_sim_flag = 1;
plot_AH_flag = 1;
AH_debug_flag = 1;

%% set up required objects for simulation
% agent -0.7 2m 6m 10m 0.7m
% bounds = [0,100,-0.7,30] ;
lanewidth = 3.7;
bounds = [0, 2000, -(lanewidth / 2) - 1, ((lanewidth/2) * 5) + 1];
goal_radius = 12;
world_buffer = 1 ; % this is used to make start and goal locations that are not too close to the robot or boundary
% planner
t_move = 3;
t_plan = 3;
t_failsafe_move = 3;


verbose_level = 0;

num_ego_vehicles = 1;
assert(num_ego_vehicles == 1, "Cannot have more than one ego vehicle");
num_moving_cars = 15;
num_static_cars = 3;
num_total_cars = num_ego_vehicles + num_moving_cars + num_static_cars;
% automated from here
W = dynamic_car_world('JL_case_construction', 1, 'bounds', bounds, ...
    'buffer', world_buffer, 'goal', [1010;3.7], ...
    'verbose', verbose_level, 'goal_radius', goal_radius, ...
    'num_cars', num_total_cars, 'num_moving_cars', num_moving_cars, ...
    't_move_and_failsafe', t_move+t_failsafe_move) ;%1 is ego , 2 means 1 obs
% if method == 1 % gpops
%     A3 = highway_cruising_10_state_agent_reduced('use_rover_controller',false);
%     A3.desired_initial_condition = [10;0; 0; 15;0;0;15];
%     A3.integrator_type= 'ode45';
% %     HLP = highway_HLP;  % wavy trajectory...
%     HLP = JL_highway_HLP_old;
% %     HLP.lookahead = 60;
% 
%     AH = highwayAgentHelper(A3,[],HLP,'t_plan',t_plan,'t_move',t_move,'t_failsafe_move',t_failsafe_move,...
%                 'eps',0.001,'verbose',verbose_level,'plot_flag',AH_debug_flag); 
%     S = rlsimulator(AH,W,'plot_sim_flag',plot_sim_flag, 'safety_layer','G','plot_AH_flag',plot_AH_flag,'method', method);
% elseif method == 0 % flzono
%     A3 = highway_cruising_10_state_agent('use_rover_controller',false); 
%     A3. desired_initial_condition = [10;0; 0; 20;0;0;20;0;0;0];
%     A3.integrator_type= 'ode45';
%     HLP = highway_HLP;
%     
%     %%% all opt, not fmincon
%     HLP = JL_highway_HLP_old;
%     HLP.lookahead = 90; % 45 for #9, %% LNOTE: lookahead
%     AH = JL_highwayAgentHelper(A3,frs,HLP,'t_plan',t_plan,'t_move',t_move,'t_failsafe_move',t_failsafe_move,...
%         'eps',0.001,'verbose',verbose_level,'plot_flag',AH_debug_flag);
%     %%% not all opt, stop when feasible, not fmincon
% %     AH = highwayAgentHelper(A3,frs,HLP,'t_plan',t_plan,'t_move',t_move,'t_failsafe_move',t_failsafe_move,...
% %         'eps',0.001,'verbose',verbose_level,'plot_flag',AH_debug_flag);
%     %%% not all opt, stop when feasible, fmincon
% %     AH = JL_highwayAgentHelper(A3,frs,HLP,'t_plan',t_plan,'t_move',t_move,'t_failsafe_move',t_failsafe_move,...
% %         'eps',0.001,'verbose',verbose_level,'plot_flag',AH_debug_flag);
%     S = rlsimulator(AH,W,'plot_sim_flag',plot_sim_flag, 'safety_layer','A','plot_AH_flag',plot_AH_flag,'method', method);
% 
% else % sos
% 
% end
% 
% 
% % automatic mode: A: using a High level planner to generate waypoints, use a
% % optimization program(sample) to select best parameter
% % (Broke) replacement mode: Z: edit the user selected parameter to something safe
% % No safe mode: N: no safe stuff..
% % G: gpops2
% AH.S = S;
% AH.pause_flag = 0;

%%
S.eval = 1; %turn on evaluation so summary will be saved
% to the episode number

% Mcity_obs = load('Mcity_obs.mat');
% AH.planned_path(2,1:175) = -38;
hlp_lookahead = 90;
% fail_iters = [6 13 22 23 25 28 29 30 35 44 48 50 52 53 56 58 70 72 74 76 77 78 80 85 93 95 96];
for j = 1:1000 % fail_iters

    %% REAL RESET
    W = dynamic_car_world('JL_case_construction', 1, 'bounds', bounds, ...
        'buffer', world_buffer, 'goal', [1010;3.7], ...
        'verbose', verbose_level, 'goal_radius', goal_radius, ...
        'num_cars', num_total_cars, 'num_moving_cars', num_moving_cars, ...
        't_move_and_failsafe', t_move+t_failsafe_move) ;%1 is ego , 2 means 1 obs
    if method == 0
        A3 = highway_cruising_10_state_agent('use_rover_controller',false); 
        A3. desired_initial_condition = [10;0; 0; 20;0;0;20;0;0;0];
        A3.integrator_type= 'ode45';
        HLP = JL_highway_HLP_old;
        HLP.lookahead = hlp_lookahead; % 45 for #9, %% LNOTE: lookahead
        AH = JL_highwayAgentHelper(A3,frs,HLP,'t_plan',t_plan,'t_move',t_move,'t_failsafe_move',t_failsafe_move,...
            'eps',0.001,'verbose',verbose_level,'plot_flag',AH_debug_flag);
        S = rlsimulator(AH,W,'plot_sim_flag',plot_sim_flag, 'safety_layer','A','plot_AH_flag',plot_AH_flag,'method', method);
    elseif method == 1
        A3 = highway_cruising_10_state_agent_reduced('use_rover_controller',false);
        A3.desired_initial_condition = [10;0; 0; 20;0;0;20];
        A3.integrator_type= 'ode45';
        HLP = JL_highway_HLP_old;
    %     HLP.lookahead = 60;
    
        AH = highwayAgentHelper(A3,[],HLP,'t_plan',t_plan,'t_move',t_move,'t_failsafe_move',t_failsafe_move,...
                    'eps',0.001,'verbose',verbose_level,'plot_flag',AH_debug_flag); 
        S = rlsimulator(AH,W,'plot_sim_flag',plot_sim_flag, 'safety_layer','G','plot_AH_flag',plot_AH_flag,'method', method);
    end
        AH.S = S;
    AH.pause_flag = 0;
    
    %%
    S.eval = 1; %turn on evaluation so summary will be saved
    % to the episode number
    
    %% REAL RESET END

        rng(j+1);
        IsDone4 = 0;
        S.epscur = j;
        S.reset();
        for i = 1:4000
            i
            S.AH.HLP.lookahead = hlp_lookahead;
%             if i>6
%                 S.AH.HLP.lookahead = 45;
%             end
            AH.planned_path = [linspace(0,1000);repmat([0;0],1,100)];
            [~,~,IsDone,LoggedSignal]=S.step([rand*2-1;rand*2-1]);
            if IsDone == 1 || IsDone == 3 || IsDone == 4 || IsDone == 5
                %crash
                %      crash with safety layer on
                %                      safely stopped but stuck
                %                                           reached goal!
                break
            end
        end
        pause(1)
end



done = 'Setup Complete'
