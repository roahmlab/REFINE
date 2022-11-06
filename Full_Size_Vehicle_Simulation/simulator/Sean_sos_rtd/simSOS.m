function simSOS(frs, episode, save_log_flag, saveVideo, new_SIM)

       
    warning ('on','all');
    warning ('off','MATLAB:gui:array:InvalidArrayShape');
    DEG2RAD = pi/180;
    videotype = 'Uncompressed AVI';

    tempidx = find(episode.name(15:end) == '_')+14-1;
    episode_idx = str2num(episode.name(15:tempidx));
    if new_SIM
        tempidx = find(episode.name(23:end) == '_')+22-1;
        episode_idx = str2num(episode.name(23:tempidx));
    end
    episode_idx

    if episode_idx ~= 5
        return
    end
    
    data = load([episode.folder,'\',episode.name]);
    num_vehs = size(data.envCars,1);

    clc; close all;
    p.no_of_lanes = 3;
    p.lane_width = 3.7;
    E = environment(p);
    E.Config.Ts_sim =0.1;
    
    % set ego vehicle
    ego_ic = data.agent_info.state(:,1);
    v = vehicle(E, [ego_ic(4); ego_ic(1); round(ego_ic(2)/p.lane_width)+1; 1]); % [v0;x0;lid0;dir]
    E.addVehicle(v);
    E.Vehicle(1).LongCtrl.set_speed = 20; %ego_ic(4); 
    E.Vehicle(1).model_type = 'Bicycle';
    E.Vehicle(1).solver = 'ODE45';
    E.Vehicle(1).Params.tc_act = 0.2; % time constant steer/curvature
    E.Vehicle(1).LatCtrl.response_time = 6;% time for lane change execution
    E.Vehicle(1).Prediction.Ts_pred = 0.5; % prediction sampling time
    E.Vehicle(1).Prediction.View.plot_convex_hull = false;
    E.Vehicle(1).Prediction.View.plot_ghost_footprints = false;
    E.P = UMplanner(E.Vehicle(1),3,frs);
%     E.P.HLP = JL_highway_HLP_SOS(); % don't use Sean's hlp
    E.P.HLP = JL_highway_HLP_old;
    HLP.lookahead = 35;
    E.Vehicle(1).LongCtrl = longCtrl_SOS(E.Vehicle(1));
    E.Vehicle(1).LatCtrl = latCtrl_SOS(E.Vehicle(1));
    E.Vehicle(1).LatCtrl.set_lateral = E.Vehicle(1).States.Y;
    E.Vehicle(1).LatCtrl.response_time = 5;
    E.Vehicle(1).Prediction.View.plot_prediction_for_static_obstacle_only = false;
    E.Vehicle(1).Prediction.collision_check_method = 'convex_hull';

    % set surrounding vehicles
    for i = 2:num_vehs
        car_ic = data.envCars(i,:);
        v = vehicle(E, [car_ic(2); car_ic(1); round(car_ic(3)/p.lane_width)+1; 1]);
        E.addVehicle(v);
        E.Vehicle(i).LongCtrl = longCtrl_noACC(E.Vehicle(i));
        E.Vehicle(i).LongCtrl.set_speed = car_ic(2); % not very important actually
        E.Vehicle(i).model_type = 'Unicycle'; %since integration is all together now, this option does not matter anymore, always use bicycle
    end

    % phantom vehicle
    % Phantom vehicle is used for predictions (phantom vehicle id is associated
    % with the traffic vehicle id). In this example we have only one Phantom
    % vehicle for vehicle with id=1 that corresponds to the Ego vehicle.
    E.addPhantom(vehicle(E,[ego_ic(4); ego_ic(1); round(ego_ic(2)/p.lane_width)+1; 1]));
    E.Phantom(1).LongCtrl = longCtrl_SOS(E.Phantom(1));
    E.Phantom(1).LatCtrl = latCtrl_SOS(E.Phantom(1));
    E.Phantom(1).LatCtrl.set_lateral = E.Phantom(1).States.Y;
    E.Phantom(1).LongCtrl.set_speed = 20; %ego_ic(4);
    E.Phantom(1).LatCtrl.response_time = 5;
    E.Phantom(1).model_type = 'Bicycle';
    E.Phantom(1).solver = 'ODE45';
    E.Phantom(1).Params.tc_act = 0.2;
    E.Phantom(1).LatCtrl.response_time = 5;
    E.Phantom(1).Prediction.Ts_pred = 0.5;
    E.Phantom(1).PathPlanning.override_main_vehicle_commands = false;
    E.Phantom(1).lane_biasing = true;

    %% simulate
    indx = 1;
    lc_dir = 1;
    
    tempidx = find(episode.name(14:end) == '_')+13-1;
    filename = ['SOS', episode.name(1:11), '_episode', episode.name(14:tempidx)];
    if new_SIM
        tempidx = find(episode.name(22:end) == '_')+21-1;
        filename = ['SOS', episode.name(1:11), '_episode', episode.name(22:tempidx)];
    end

    if saveVideo
        video_name = ['video_', filename];
        writerObj = VideoWriter(video_name,videotype);
        set(writerObj,'FrameRate',1/E.Config.Ts_sim);
        open(writerObj);
    end

    %stop_sim flag tells you what's wrong:
    %0 ok, 1 collision, 2 overtime in real time, 3 no plan found at step 1, 4 came to a stop
    start_sim_tic = tic;
    while E.Vehicle(1).States.X < data.world_info.goal(1) && E.stop_sim == 0
        if toc(start_sim_tic) > 50000 %exit if ran more than 150 seconds
            E.stop_sim =2 ;
        end
        if ~E.View.button.Value
            E.View.button.String = 'Pause';
            if indx > 1
                E.P.plan((indx-1)*E.Config.Ts_sim);
            end
            E.time = (indx-1)*E.Config.Ts_sim; % start at time = 0
            for ftpt_idx = 1:length(E.Vehicle) % since we are recording footprint in step function, need to generate it
                E.Vehicle(ftpt_idx).updateFootprintPoints
            end
            for ftpt_idx = 1:length(E.Phantom) % since we are recording footprint in step function, need to generate it
                E.Phantom(ftpt_idx).updateFootprintPoints
            end
            E.step %steping index, to plot plan and actual together,
            E.P.speedometer = text(E.P.master_ax,E.P.master_ax.XLim(2)-1, mean(E.P.master_ax.YLim), num2str(E.Vehicle(1).States.V,'%.1f')+"m/s",'FontSize',12,'BackgroundColor','w','EdgeColor','k');
            set(E.P.traj_plotter,'XData',E.Vehicle(1).state(2,2:end),'YData',E.Vehicle(1).state(1,2:end)); %skip the first one since it is starting from random place we don't have control over
            set(E.P.traj_plotter.Parent, 'XLim', [-8.5196 15.9196]);
            if rem(indx,1) == 0 %draw less frequently if needed
                drawnow
            end
            title(E.View.XY,['time = ',num2str(indx*E.Config.Ts_sim),' s, x = ',num2str(E.Vehicle(1).States.X),' m'])

            indx = indx+1;

            if saveVideo
                frame = getframe(E.View.figure);
                writeVideo(writerObj,frame);
            end

            noplanned = abs(rem((((indx-1)*E.Config.Ts_sim - E.P.time_generated) - E.P.t_move),E.P.t_move)) > 0.01 && (indx-1)*E.Config.Ts_sim-E.P.time_generated < 10;

            figure(1)
        else
            drawnow
            E.View.button.String = 'Run';
        end
        
        if indx <= 3 && E.P.no_plan_found == 1 % 3 here since step 1 and 2 it doesn't plan for some reason
            E.stop_sim = 3;
        end
        if E.Vehicle(1).States.Vx < 0.1
            E.stop_sim = 4;
        end

    end

    %% save things
    if save_log_flag %&& E.stop_sim==4
        save_struct = struct;
        for i = 1:num_vehs
            save_struct.Vehicle_time{i} = E.Vehicle(i).time;
            save_struct.Vehicle_state{i} = E.Vehicle(i).state;
        end
        save_struct.Obs = []; %E.Obs.corners;
        save_struct.plant = E.P.planHistt;
        save_struct.plan =  E.P.planHistp;
        save_struct.tsolve = E.P.planHistsolvetime;
        save_struct.total_time = toc(start_sim_tic);
        save_struct.FRS = E.P.FRSHist;
        
        % reference data for plot
        Filename = [filename,'_stop=',num2str(E.stop_sim)];
        save(Filename,'save_struct');
    end
    if saveVideo
        close(writerObj);
    end
end





