function simGPOPS(episode)
    
    data = load([episode.folder,'\',episode.name]);
    tempidx = find(episode.name(15:end) == '_')+14-1;
    episode_idx = str2num(episode.name(15:tempidx))

    if episode_idx ~= 19
        return
    end
        
    plot_sim_flag = 1;
    plot_AH_flag = 1;
    AH_debug_flag = 1;
    
    lanewidth = 3.7;
    bounds = [0,1000,-lanewidth/2-1,lanewidth/2*5+1] ;
    goal_radius = 12;
    world_buffer = 1 ;
    t_move =3;
    t_plan =3;
    t_failsafe_move = 3;
    
    verbose_level = 0;
    
    A3 = highway_cruising_10_state_agent_reduced('use_rover_controller',false);
    A3. desired_initial_condition = [10;0; 0; 15;0;0;15];
    A3.integrator_type= 'ode45';
    
    W = JL_dynamic_car_world(data.envCars,'bounds',bounds,'buffer',world_buffer,...
        'verbose',verbose_level,'goal_radius',goal_radius,'num_cars',30,'num_moving_cars',25,'t_move_and_failsafe', t_move+t_failsafe_move) ;%1 is ego , 2 means 1 obs
    
    HLP = JL_GPOPS_highway_HLP;
%     HLP = JL_highway_HLP_old;
    AH = highwayAgentHelper(A3,[],HLP,'t_plan',t_plan,'t_move',t_move,'t_failsafe_move',t_failsafe_move,...
                'eps',0.001,'verbose',verbose_level,'plot_flag',AH_debug_flag); 
    S = rlsimulator(AH,W,'plot_sim_flag',plot_sim_flag, 'safety_layer','G','plot_AH_flag',plot_AH_flag);
    
    AH.S = S;
    AH.pause_flag = 0;
    
    S.eval = 1;
    S.epscur = episode_idx; 
    flags = struct;
    flags.discrete_flag = S.discrete_flag;
    flags.replace_action = S.replace_action;
    flags.safety_layer = S.safety_layer;
    S.W.setup();
    S.AH.reset(flags,S.epscur);
    S.plot();
    for i = 1:3000
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

end