%clear;
%close all;
% plot flag
% clear;-
run_sos = 0;
run_zono = 1;
run_gpops = 0;


if run_zono
%     frs_filename = 'FRS_Rover_22-Jan-2022_no_force.mat' ;
    frs_filename = 'FRS_Rover_30-Aug-2022_no_force.mat';
    if ~exist('frs','var')
        disp('Loading frs')
        frs = load(frs_filename) ;
    else
        disp('table already loaded') ;
    end
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
bounds = [0,1000,-lanewidth/2-1,lanewidth/2*5+1] ;
goal_radius = 12;
world_buffer = 1 ; % this is used to make start and goal locations that are not too close to the robot or boundary
HLP_buffer = 0.1;
RTD_HLP_grow_tree_mode ='seed';
% planner
buffer = 0.3 ; % [m] distance used to buffer obstacles for planning, must be larger than the agent's footprint
t_move =3;
t_plan =3;
t_failsafe_move = 3;
Ts = t_move;
Tf = Ts*150;

% verbose level for printing
verbose_level = 0;

% automated from here
if run_gpops
    A3 = highway_cruising_10_state_agent_reduced('use_rover_controller',false);
    A3. desired_initial_condition = [10;0; 0; 15;0;0;15];
else
    A3 = highway_cruising_10_state_agent('use_rover_controller',false); 
    A3. desired_initial_condition = [10;0; 0; 15;0;0;15;0;0;0];
end

%if we are running zonotope stuff, use full agent with FL controller
% if running gpops , use reduced agent
A3.integrator_type= 'ode45';

% A3. desired_initial_condition(4) = 6;
% A3. desired_initial_condition(7) = 6;
% A3.desired_initial_condition=[-5;0.0;0;1.0;0.0;0.0;0;0;1];
% A3. desired_initial_condition(1) = A3. desired_initial_condition(1);
% A3. desired_initial_condition(2) = -38;
% A3. desired_initial_condition(3) = 0;
% 
% A3. desired_initial_condition(4) = 0;
% A3. desired_initial_condition(7) = 0;
W = dynamic_car_world('bounds',bounds,'buffer',world_buffer,'goal',[1010;3.7],...
    'verbose',verbose_level,'goal_radius',goal_radius,'num_cars',30,'num_moving_cars',25,'t_move_and_failsafe', t_move+t_failsafe_move) ;%1 is ego , 2 means 1 obs
%%
HLP = highway_HLP;%RRT_star_HLP('timeout',0.5,'bounds',bounds,'grow_tree_mode',RTD_HLP_grow_tree_mode,'verbose',verbose_level);
% HLP = JL_highway_HLP_old;
if run_zono
%     AH = highwayAgentHelper(A3,frs,HLP,'t_plan',t_plan,'t_move',t_move,'t_failsafe_move',t_failsafe_move,...
%         'eps',0.001,'verbose',verbose_level,'plot_flag',AH_debug_flag);
    AH = JL_highwayAgentHelper(A3,frs,HLP,'t_plan',t_plan,'t_move',t_move,'t_failsafe_move',t_failsafe_move,...
        'eps',0.001,'verbose',verbose_level,'plot_flag',AH_debug_flag);
    S = rlsimulator(AH,W,'plot_sim_flag',plot_sim_flag, 'safety_layer','A','plot_AH_flag',plot_AH_flag);

elseif run_sos
    AH = highwaySOSAgentHelper(A3,frs,HLP,'t_plan',t_plan,'t_move',t_move,'t_failsafe_move',t_failsafe_move,'eps',0.001,'verbose',verbose_level,'plot_flag',AH_debug_flag);
elseif run_gpops
    AH = highwayAgentHelper(A3,frs,HLP,'t_plan',t_plan,'t_move',t_move,'t_failsafe_move',t_failsafe_move,...
            'eps',0.001,'verbose',verbose_level,'plot_flag',AH_debug_flag); 
    S = rlsimulator(AH,W,'plot_sim_flag',plot_sim_flag, 'safety_layer','G','plot_AH_flag',plot_AH_flag);

end
% automatic mode: A: using a High level planner to generate waypoints, use a
% optimization program(sample) to select best parameter
% (Broke) replacement mode: Z: edit the user selected parameter to something safe
% No safe mode: N: no safe stuff..
% G: gpops2
AH.S = S;
AH.pause_flag = 0;
random_inputs = true; %true for user inputs using arrow keys, false for random inputs
%%
S.eval = 1; %turn on evaluation so summary will be saved
% to the episode number
%%
% Mcity_obs = load('Mcity_obs.mat');
% AH.planned_path(2,1:175) = -38;

    for j = 2:100
        S.epscur = j;
        S.reset();
        for i = 1:3000
            i
%             if i <8
%                 W.obstacles = sim_data.obstacles{1};
%                 W.obstacles_seen = sim_data.obstacles{1};
%                 AH.planned_path = sim_data.path{1};
%                 
%             elseif i < 16
%                 
%                 AH.planned_path = sim_data.path{2};
%                 W.obstacles = sim_data.obstacles{2};
%                 W.obstacles_seen = sim_data.obstacles{2};
%                 
%             else
%                 AH.planned_path = sim_data.path{3};
%                 W.obstacles = sim_data.obstacles{3};
%                 W.obstacles_seen = sim_data.obstacles{3};
%             end
%             
%             
            
            
%            if A3.state(1,end) > 0
%             W.obstacles = Mcity_obs.Mcity_obs;
% %             W.obstacles(1,56) = 35;
% %             W.obstacles(1,57) = 35.3;
% %             
%             W.obstacles_seen =  Mcity_obs.Mcity_obs;
%             AH.planned_path = sim_data.path{1};
%             AH.planned_path = [[77 95 deg2rad(-90)];[77 50 deg2rad(-95)];[72 30 deg2rad(-95)];[60 10  deg2rad(-95-15)];[41 -5  deg2rad(-95-45)]]';
%            else
               AH.planned_path = [linspace(0,1000);repmat([0;0],1,100)];
%             AH.planned_path = sim_data.path{2};
%             W.obstacles = sim_data.obstacles{2};
%             W.obstacles_seen = sim_data.obstacles{2};
%            end
            
            
            
            %            else
            %
            %             AH.planned_path = sim_data.path{3};
            %             W.obstacles = sim_data.obstacles{3};
            %             W.obstacles_seen = sim_data.obstacles{3};
            
            
            %            end
            if random_inputs
                value = 114;
            else
                figure(1)
                k = waitforbuttonpress;
                value = double(get(gcf,'CurrentCharacter'));
            end
            
            % 28 leftarrow
            % 29 rightarrow
            % 30 uparrow
            % 31 downarrow
            if i == 15
                a=1;
            end
            if value == 28
                [~,~,IsDone,LoggedSignal]=S.step([-2/3;0]);
            elseif value == 29
                [~,~,IsDone,LoggedSignal]=S.step([1;0]);
            elseif value == 30
                [~,~,IsDone,LoggedSignal]=S.step([1/3;1]);
            elseif value == 31
                [~,~,IsDone,LoggedSignal]=S.step([1/3;-1]);
            elseif value == 114
                [~,~,IsDone,LoggedSignal]=S.step([rand*2-1;rand*2-1]);
            else
                [~,~,IsDone,LoggedSignal]=S.step([1/3;0]);
            end
            
            if IsDone == 1 || IsDone == 3 || IsDone == 4 || IsDone == 5
                %crash
                %      crash with safety layer on
                %                      safely stopped but stuck
                %                                           reached goal!
                break
            end
            
        end
    end

done = 'Setup Complete'