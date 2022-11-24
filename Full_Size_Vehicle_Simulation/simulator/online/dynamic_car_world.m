% this is the class of the testing environment
classdef dynamic_car_world < world
    properties
        buffer = 1 ;
        obstacles_unseen
        obstacles_seen
        bounds_as_polyline
        
        % plotting
        obstacle_seen_color = [255 255 255]/255 ;
        obstacle_unseen_color = [1 0.6 0.6] ;
        obstacle_history

        
        obstacle_size = [4.8, 2] ;
        envCars
        num_cars = 2;
        num_moving_cars = 2;
        o %this is obstacle centered at 0
        o_no_frs
        v_array = [22 24 26 28 30 32];
        obs_FRS
        obstacles_old % for collision check
        
        lanewidth = 3.7
        
        car_safe_dist = 15;%40 % car 6 m long
        
        SIM_MAX_DISTANCE = 800;
        car_max_spd = 25;
        car_min_spd = 15;
        
        start_line
        t_move_and_failsafe

    end
    
    methods
        %% constructor
        function W = dynamic_car_world(varargin)
            % set default properties
            start_line = varargin{2}(1);
            bounds = [start_line -start_line 0 12] ;
            W.start_line = -start_line;
            N_obstacles = 0 ;
            
            % parse other args
            W = parse_args(W,'bounds',bounds,'N_obstacles',N_obstacles,varargin{:}) ;
            
            % call setup to generate obstacles
            W.setup() ;
            
            % check for mapping toolbox
            try
                polyxpoly([],[],[],[]) ;
            catch
                error(['Please make sure the Mapping Toolbox is installed',...
                    ' so you can use polyxpoly.'])
            end
        end
        

        function setup(W,seed)
            % set the dynamic world with moving & static vehicles
            if exist('seed','var')
                rng(seed)
            end
            W.placeCars();
            W.obstacle_history = [];

            B = W.bounds ;
            xlo = B(1) ; xhi = B(2) ; ylo = B(3) ; yhi = B(4) ;
            W.bounds_as_polyline = make_box([xhi-xlo,yhi-ylo]) + repmat(mean([xhi xlo ; yhi ylo],2),1,5) ;
            
            
            % generate start position on left side of room with initial
            % heading of 0, and make sure it's not too close to the walls
            b = W.buffer ;
            
            xlo = xlo + 2*b ;
            xhi = xhi - 2*b ;
            ylo = ylo + 2*b ;
            yhi = yhi - 2*b ;
            
            if isempty(W.start)
                s = [xlo ;
                    rand_range(ylo, yhi) ;
                    0 ] ;
                W.start = s ;
            end
            
            % generate goal position on right side of room
            if isempty(W.goal)
                g = [xhi - 10 ;
                    (ylo+yhi)/2] ;
                W.goal = g ;
            end
            if ~isempty(W.envCars)
                W.N_obstacles = size(W.envCars,1)-1;
                % generate obstacles around room
                N_obs = W.N_obstacles ;
                
                if N_obs > 0
                    
                    O = nan(2, 6*N_obs) ; % preallocate obstacle matrix
                    
                    % obstacle base
                    l = W.obstacle_size(1);
                    w = W.obstacle_size(2);
                    
                    o = [-l/2  l/2 l/2 -l/2 -l/2 ;
                        -w/2 -w/2 w/2  w/2 -w/2 ] ;
                    W.o_no_frs = o;
                    W.o = cell(length(W.v_array),30); % 10 is time index
                    obs_count = 2; %start with second one, first one is ego
                    for idx = 1:6:(6*N_obs-1)
                        c = [W.envCars(obs_count,1); W.envCars(obs_count,3)];
                        O(:,idx:idx+4) = W.o_no_frs + repmat(c,1,5) ;
                        obs_count = obs_count + 1;
                    end
                    
                    W.obstacles = O ;
                    W.obstacles_old = O ;  
                    W.N_obstacles = N_obs ;
                end
                
                W.obstacles_unseen = [] ;
                W.obstacles_seen = W.obstacles;
                W.plot_data.obstacles_seen = W.obstacles ;
                W.plot_data.obstacles_unseen = [] ;
                W.plot_data.start = [] ;
                W.plot_data.goal = [] ;
                W.plot_data.goal_zone = [] ;
                W.plot_data.bounds = [] ;
                W.reset();
            end
        end

        function placeCars(W)
            W.envCars = zeros(W.num_cars, 3);
            
            
            for i = 2:W.num_cars
                laneOverlap = true;
                while laneOverlap
                    is_static_obs = i <= (W.num_cars - W.num_moving_cars);
                        if is_static_obs
                            MAX_DIST =  abs(1000) - W.car_safe_dist; 
                        else
                            MAX_DIST = W.SIM_MAX_DISTANCE;
                        end
                        xPos = MAX_DIST * rand  + 80;% start at least 80 ahead
                    
                    
                    if is_static_obs
                        laneNum = randi([0,1])*2;
                    else
                        laneNum = randi([0,2]); % {0, 1, 2}
                    end
                    laneOverlap = false;
                    for j = 1:i
                        if  abs(xPos - W.envCars(j,1)) < W.car_safe_dist 
                            laneOverlap = true;
                            break;
                        end
                    end
                    if ~laneOverlap % assign car to new slot
                        W.envCars(i,1) = xPos;
                        if is_static_obs
                            W.envCars(i,2) = 0;
                        else
                            W.envCars(i,2) = W.car_min_spd+rand*(W.car_max_spd-W.car_min_spd);
                        end
                        W.envCars(i,3) = W.lanewidth * laneNum;
                    end
                end
            end
        end
        
        
        function world_info = get_world_info(W,agent_info,~)
            W.vdisp('Getting world info!',3)
            obs_count = 2;N_obs = size(W.envCars,1)-1; % 1 is the ego vehicle; 
            
            %following is dynamic obstacles that need to be updated
            O = nan(2,6*N_obs);
            O_future = nan(2,6*N_obs);
            for idx = 1:6:(6*N_obs-1)
                c = [W.envCars(obs_count,1); W.envCars(obs_count,3)];
                O_future(:,idx:idx+4) = W.o_no_frs + repmat(c,1,5) + [(agent_info.time(end)+W.t_move_and_failsafe)*W.envCars(obs_count,2);0];
                       O(:,idx:idx+4) = W.o_no_frs + repmat(c,1,5) + [(agent_info.time(end))*W.envCars(obs_count,2);0];

                obs_count = obs_count + 1;
            end
            O_dynamic = cell(0);
            O_dynamic{1} = O; % possition
            O_dynamic{2} = W.envCars(2:end,2); % velocity
            W.obstacles_seen = O;
            W.obstacles_old = O;
            W.obstacles = O_future;
            world_info.obstacles = [];
            world_info.dyn_obstacles = O_dynamic;
            world_info.bounds = W.bounds ;
            world_info.start = W.start ;
            world_info.goal = W.goal ;
            world_info.dimension = W.dimension ;
            world_info.obs_FRS = W.obs_FRS;
        end
        
        
        %% collision check and goal  
        function out = collision_check(W,agent_info,check_full_traj_flag)
            
            % by default, don't check the full trajectory
            if nargin < 3
                check_full_traj_flag = false ;
            end
            
            % initialize output (innocent until proven guilty)
            out = 0 ;
            
            % extract agent info
            pos_idx = agent_info.position_indices ;
            h_idx = agent_info.heading_index ;
            fp = agent_info.footprint_vertices ;
            Z = agent_info.state ;
            T = agent_info.time ;
            
            % set up obstacles
            O = [W.obstacles_seen, W.bounds_as_polyline,nan(2,1)] ;
            O_old = [W.obstacles_old, W.bounds_as_polyline,nan(2,1)] ;
            % if the agent has moved, we need to check if it crashed;
            % alternatively, we could check the entire trajectory for crashes;
            % finally, if it didn't move, then check the full trajectory
            t_start = W.current_time ;
            if check_full_traj_flag
                t_log = true(size(T)) ;
            else
                t_log = T >= t_start ;
            end
            
            if sum(t_log) > 1
                T = T(t_log) ;
                Z = Z(:,t_log) ;
                
                N_chk = 1; % number of checks to make
                
                t_world = T(1) ; % start time of each check
                
                for chk_idx = 1:N_chk
                    % get the time vector to check
                    t_log_idx = T >= t_world ;
                    T_idx = T(t_log_idx) ;
                    Z_idx = Z(:,t_log_idx) ;
                    
                    % create the time vector for interpolation
                    t_chk = (0:0.02:5 ) + t_world ;
                    t_chk = t_chk(t_chk <= T_idx(end)) ;
                    
                    % create the interpolated trajectory
                    Z_chk = match_trajectories(t_chk,T_idx,Z_idx) ;
                    O_idx = 1;
                    while O_idx + 5 <= size(O_old,2)
                        if abs(O(1,O_idx)-O_old(1,O_idx)) > 100
                            O(:,O_idx:O_idx+5) = []; O_old(:,O_idx:O_idx+5) = [];
                        else
                            O_idx = O_idx + 6;
                        end
                    end
                    O_x = match_trajectories(t_chk,[T_idx(1) T_idx(end)],[O_old(1,:)',O(1,:)'])' ;
                    O_y = match_trajectories(t_chk,[T_idx(1) T_idx(end)],[O_old(2,:)',O(2,:)'])' ;
                    
                    
                    % create a copy of the agent footprint, rotated at each
                    % point of the trajectory
                    X = Z_chk(pos_idx,:) ; % xy positions of trajectory
                    N = size(X,2) ;
                    X = repmat(X(:),1,size(fp,2)) ;
                    
                    % X is rep'd for each pt of agent footprint vertices
                    
                    if ~isempty(h_idx) && (length(agent_info.footprint) > 1)
                        % if there is a heading, and the agent is not a circle,
                        % then rotate each footprint contour
                        H = Z_chk(h_idx,:) ;
                        R = rotmat(double(H)) ;
                        F = R*repmat(fp,N,1) + X ;
                    else
                        % otherwise, just place the footprint contour at each
                        % point of the trajectory
                        F = repmat(fp,N,1) + X ;
                    end
                    
                    Fx = [F(1:2:end,:)' ; nan(1,N)] ;
                    Fy = [F(2:2:end,:)' ; nan(1,N)] ;
                    F = [Fx(:)' ; Fy(:)'] ;
                    ci_bool = ones( 1,size(F,2)/6);
                    for chk_time_idx = 1: size(F,2)/6
                        F_idx = (6*(chk_time_idx-1)+1):(6*(chk_time_idx));
                        [ci,cyi] =polyxpoly(F(1,F_idx),F(2,F_idx),O_x(chk_time_idx,:),O_y(chk_time_idx,:)) ;
                        if ~isempty(ci)
                            break
                        else
                            ci_bool(chk_time_idx)   = 0;
                        end
                    end
                    
                    if any(ci_bool)
                        out = 1 ;
                    else
                        out = 0;
                    end
                    t_world = t_world + 5 ;
                end
            end
            
            % update the world time index
            W.current_time = agent_info.time(end) ;
        end

        function out = goal_check(W,agent_info)
            out = agent_info.position(1,end)>= W.goal(1);
        end
        %% plotting
        function plot(W,agent_info)
            % set up hold if needed
            hold_check = false ;
            if ~ishold
                hold_check = true ;
                hold on
            end
            obs_count = 2;N_obs = size(W.envCars,1)-1;
            O_seen = nan(2,6*N_obs);
            for idx = 1:6:(6*N_obs-1)
                c = [W.envCars(obs_count,1); W.envCars(obs_count,3)];
                O_seen(:,idx:idx+4) = W.o_no_frs + repmat(c,1,5) + [(agent_info.time(end))*W.envCars(obs_count,2);0];
                obs_count = obs_count + 1;
            end
            % plot sensed obstacles            
            if isempty(O_seen)
                O_seen = nan(2,1) ;
            end
            W.obstacles_seen = O_seen;
            if ~check_if_plot_is_available(W,'road_lanes')
                road_lanes ={};
                w= 2;
                road_lanes{1} =  fill([-W.start_line 0 0 -W.start_line -W.start_line],[-0.7 -0.7 12.7 12.7 -0.7],[207,207,207]/255);
                road_lanes{2} = plot([-W.start_line,0],[12, 12],'LineWidth',w,'Color',[255, 255, 255]/255);
                road_lanes{3} =  plot([-W.start_line,0],[8, 8],'--','LineWidth',w,'Color',[1 1 1]);
                road_lanes{4} =  plot([-W.start_line,0],[4, 4],'--','LineWidth',w,'Color',[1 1 1]);
                road_lanes{5} =  plot([-W.start_line,0],[0, 0],'LineWidth',w,'Color',[255, 255, 255]/255);
                W.plot_data.road_lanes = road_lanes ;
            end
            [X_plot, Y_plot] = W.convert_data_for_plots([nan O_seen(1,:)],[nan O_seen(2,:)]);

            if check_if_plot_is_available(W,'obstacles_seen')
                W.plot_data.obstacles_seen.XData = X_plot ;
                W.plot_data.obstacles_seen.YData = Y_plot ;
            else
                seen_data = patch(X_plot, Y_plot, W.obstacle_seen_color);
                W.plot_data.obstacles_seen = seen_data ;
            end
            
            % plot unsensed obstacles
            O_unseen = W.obstacles_unseen ;
            
            if isempty(O_unseen)
                O_unseen = nan(2,1) ;
            end
            
            if check_if_plot_is_available(W,'obstacles_unseen')
                W.plot_data.obstacles_unseen.XData = O_unseen(1,:) ;
                W.plot_data.obstacles_unseen.YData = O_unseen(2,:) ;
            else
                unseen_data = plot(O_unseen(1,:),O_unseen(2,:),'Color',W.obstacle_unseen_color) ;
                W.plot_data.obstacles_unseen = unseen_data ;
            end
            
            % plot goal and goal zone
            g = W.goal ;
            
            if ~check_if_plot_is_available(W,'goal')
                goal_pos = make_box(W.goal_radius*2);
                goal_data = patch(goal_pos(1,:)+g(1),goal_pos(2,:)+g(2),[1 0 0]) ;
                W.plot_data.goal = goal_data ;
            end

            % plot bounds
            B = W.bounds_as_polyline ;
            
            if ~check_if_plot_is_available(W,'bounds')
                bounds_data = plot(B(1,:),B(2,:),'Color','w','LineWidth',5) ;
                W.plot_data.bounds = bounds_data ;
            end
            
            if hold_check
                hold off ;
            end
        end
        
        function [X, Y] = convert_data_for_plots(W,x,y)
            idn=find(isnan(x));
            Sz=diff(idn)-1;
            Nmax=max(Sz);
            N=numel(Sz);
            X=zeros(Nmax,N);
            Y=X;
            for i=1:N
                X(1:Sz(i),i)=x(idn(i)+1:idn(i+1)-1);
                Y(1:Sz(i),i)=y(idn(i)+1:idn(i+1)-1);
            end
        end
    end
    
end