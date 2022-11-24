% Main function for REFINE simulation

% load FRS
frs_filename = 'car_frs.mat';
if ~exist('frs','var')
    disp('Loading frs')
    frs = load(frs_filename) ;
else
    disp('table already loaded') ;
end


plot_sim_flag = 1;
plot_AH_flag = 1;
save_result = false; % make it true if you want to save the simulation data

%% set up required objects for simulation
lanewidth = 3.7;
bounds = [0, 2000, -(lanewidth / 2) - 1, ((lanewidth/2) * 5) + 1];
goal_radius = 12;
world_buffer = 1 ;
t_move = 3;
t_plan = 3;
t_failsafe_move = 3;
verbose_level = 0;
num_ego_vehicles = 1;
num_moving_cars = 15;
num_static_cars = 3;
num_total_cars = num_ego_vehicles + num_moving_cars + num_static_cars;
hlp_lookahead = 90;


for j = 1:1000 
    % RESET simulation environment
    World = dynamic_car_world( 'bounds', bounds, ...
        'buffer', world_buffer, 'goal', [1010;3.7], ...
        'verbose', verbose_level, 'goal_radius', goal_radius, ...
        'num_cars', num_total_cars, 'num_moving_cars', num_moving_cars, ...
        't_move_and_failsafe', t_move+t_failsafe_move) ;

    Agent = highway_cruising_10_state_agent; % takes care of vehicle states and dynamics
    Agent.desired_initial_condition = [10;0; 0; 20;0;0;20;0;0;0];
    Agent.integrator_type= 'ode45';
    HLP = simple_highway_HLP; % high-level planner
    HLP.lookahead = hlp_lookahead; 
    AgentHelper = highwayAgentHelper(Agent,frs,HLP,'t_plan',t_plan,'t_move',t_move,'t_failsafe_move',t_failsafe_move,...
        'verbose',verbose_level); % takes care of online planning
    Simulator = rlsimulator(AgentHelper,World,'plot_sim_flag',plot_sim_flag,'plot_AH_flag',plot_AH_flag,'save_result',save_result);

    AgentHelper.S = Simulator;
    Simulator.eval = 1; %turn on evaluation so summary will be saved

   

    rng(j+1);
    IsDone4 = 0;
    Simulator.epscur = j;
    Simulator.reset();
    for i = 1:4000
        i
        AgentHelper.planned_path = [linspace(0,1000);repmat([0;0],1,100)];
        [~,~,IsDone,LoggedSignal]=Simulator.step([rand*2-1;rand*2-1]);
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



done = 'Simulation Complete'
