classdef GenParamClass < handle
    properties
        A3_
        AH_
        bounds_
        obstacles_
        world_obstacles_
        frs_
        frs_low_
        frs_dir_info_
        frs_lan_info_
        gen_param_pub_
        gen_param_msg_
        agent_state_
        x_des_
        K_
        K_next_
        lock_output_
        curr_path_
        obs_origin_
        obstacle_pub_
        obstacle_pub_msg_
        predicted_agent_state_stored_
        agent_state_stored_
        x_des_stored_
        obstacles_stored_
    end
    methods
        function obj = GenParamClass(frs, frs_low, frs_dir_info, frs_lan_info)
            % Defaults
            obj.agent_state_ = [-490;6;0;0.8;0;0;0.8];
            obj.x_des_ = [0;0];
            obj.K_ = [0; 0; 0; 0];
            obj.K_next_ = [0; 0; 0; 0];
            obj.obs_origin_ = [0; 0];
            obj.curr_path_ = [
    %corner in the back of the room
16.2899 -38.8251 -0.0104;
21.4259 -38.7495 -0.0761; 
23.3016 -38.8870 -0.1339; 
24.7201 -39.2633 -0.3837;
25.4002 -39.6549 -0.6576;
25.8645 -40.2692 -1.1036;
26.0486 -41.7327 -1.6739;
25.8345 -44.2227 -1.6973;
% elevator corner
%    -4.3107 -6.1747 -3.9167-0.757;
%    -4.3661 -4.6681 -3.8790-0.757;
%    -4.3171 -2.8413 2.2412-0.757;
%     -3.8599 -1.2245 1.8296-0.757;
%    -2.9550 -0.3843 1.4387-0.757;    
%    -1.6945 0.0058 1.0032-0.757; 
%     0.0915 0.2038 0.8176-0.757;
%     3.1124 0.2924 0.7570-0.757;
]';
            
            obj.frs_ = frs;
            obj.frs_low_ = frs_low;
            obj.frs_dir_info_ = frs_dir_info;
            obj.frs_lan_info_ = frs_lan_info;
            [obj.gen_param_pub_, obj.gen_param_msg_] = rospublisher('/gen_param_out', 'rover_control_msgs/GenParamInfo');
            %[obj.obstacle_pub_, obj.obstacle_pub_msg_] = rospublisher('/zonotope_visualization', 'jsk_recognition_msgs/PolygonArray');
            %obj.obstacle_pub_msg_
            [obj.A3_, obj.AH_, obj.bounds_, obj.obstacles_] = build_helpers(obj.frs_, obj.frs_low_, obj.frs_dir_info_, obj.frs_lan_info_);
            obj.AH_.parent_pointer = obj;
            obj.lock_output_ = false;
            %obj.curr_path_ = [];
            obs_w1 = 0.16;
            obs_r1 = obs_w1/2.0;
            obs_x1 =  2.0580;
            obs_y1 = -0.2948;

            obs_r2 = obs_r1;
            obs_x2 = 4.1485;
            obs_y2 = 0.6018;
            
            obj.obstacles_ = [
                gen_obs(20,  -20, 0.2, 0.5),...
                %gen_obs(40,  20, 1.5, 3),...
                %gen_obs(20,  -1.2987 - 3, 25, 3),...
                %gen_obs(20,  0.7728 + 3, 25, 3),...
                %gen_obs(-0.4345, -0.7661, 0.08),...
                %gen_obs(3.35, 0.53, 0.47, 0.40),...
                %gen_obs(6.7251, 0.1901, 0.1, 2.50),...
                %gen_obs(5.31,1.51, 0.2,0.3)
                %gen_obs(1.2318 + 150.0, 2.4470 + 150.0, 0.08)
                %gen_obs(obs_x1, obs_y1, obs_r1), ...
                %gen_obs(obs_x2, obs_y2, obs_r2),  
            ];
            obj.world_obstacles_ = [
                gen_obs(-19.5,19.944,0.25,18.375),...
                gen_obs(-16.892,19.944,0.25,18.375),...
                gen_obs(-3.442,37.994,13.7,0.25),...
                gen_obs(-3.442,40.444,13.7,0.25),...
                gen_obs(10.008,19.569,0.25,18.625),...
                gen_obs(12.258,19.569,0.25,18.625),...
                gen_obs(-3.442,1.194,13.7,0.25),...                
                gen_obs(-3.867,-1.106,13.625,0.25)
                ];
                
                
        end
        
        function gen_param_trigger(obj, sub, msg)
            disp('gen param triggered');
            obj.gen_params(msg.Data);
        end
        
        function store_vals(obj)
            obj.agent_state_stored_ = obj.agent_state_;
            obj.predicted_agent_state_stored_ = obj.agent_state_stored_;
            obj.x_des_stored_ = obj.x_des_;
            obj.obstacles_stored_ = obj.obstacles_;
        end
        
        function gen_params(obj, is_first_run)
            tic;
            disp('GenParams');
            disp(obj.agent_state_)
            disp(obj.K_next_)
            disp(obj.K_)
            x_offset = 3.0;
            y_offset = -4.0;
            obj.x_des_ = [obj.agent_state_(1) + x_offset * cos(obj.agent_state_(3));
                          obj.agent_state_(2) + y_offset * sin(obj.agent_state_(3))];
%             obj.x_des_ = [7.07; 3.155];
            if ~isempty(obj.curr_path_)
               obj.x_des_ = get_waypoint_from_path_var_lookahead(obj.agent_state_, obj.curr_path_, 5);
%                obj.x_des_ = [0.37; -0.08];
%                disp('dist to xdes')
%                norm_to_x_des = norm(obj.x_des_ - (obj.agent_state_(1:2)));
            else
                disp('xdes');
                %obj.x_des_ = [obj.agent_state_(1,1); obj.agent_state_(2,1)];
                %obj.x_des_ = [6.5256; 0.2889];
            end
            if is_first_run
                obj.lock_output_ = false;
                obj.AH_.cur_t0_idx = 1;
                obj.AH_.prev_action = -1;
                disp('knext')
                tic;
                obj.store_vals();
                [obj.K_, ~] = gen_parameter_standalone(obj.AH_, ...
                                                       obj.bounds_, ...
                                                       obj.obstacles_stored_, ...
                                                       obj.agent_state_stored_, ...
                                                       obj.x_des_stored_);
                disp('GEN PARAM TOC')
                toc
                if size(obj.K_, 1) < 4 || size(obj.K_, 2) < 1
                    return;
                end
                obj.send_params(obj.K_);
            else
                obj.K_ = obj.K_next_;
            end
            
            u_cur = obj.agent_state_(4) ;
            t_plan = 0.75;
            [Au, Ay, t0_offset, type_manu] = obj.get_param_info(obj.K_);
            load_const
            if type_manu == 3
                [T,U,Z] = gaussian_T_parameterized_traj_with_brake(t0_offset,Ay,Au,u_cur,[],0,1);
            else
                [T,U,Z] = gaussian_one_hump_parameterized_traj_with_brake(t0_offset,Ay,Au,u_cur,[],0,1);
            end
            zcur = obj.agent_state_;
            [tout,zout] = obj.A3_.integrator(@(t,z) obj.A3_.dynamics(t,z,T,U,Z),...
                                       [0 t_plan], zcur) ;
            state_next = zout(:,end)
            %obj.lock_output_ = norm(state_next(1:2) - obj.x_des_) < 1.0;
            tic;
            obj.store_vals();
            obj.predicted_agent_state_stored_ = state_next;
            [obj.K_next_, ~] = gen_parameter_standalone(obj.AH_, ...
                                       obj.bounds_, ...
                                       obj.obstacles_stored_, ...
                                       obj.predicted_agent_state_stored_, ...
                                       obj.x_des_stored_);
            disp('GEN PARAM TOC')
            toc
            if ~obj.lock_output_
                obj.send_params(obj.K_next_);
            end
        end
        function callback_obstacle(obj, src,msg)
%             disp('callback obstacles')
            %tic;
            obj.obstacles_ = zeros(2, 6*length(msg.Polygons));
            for i = 1:length(msg.Polygons)
                x = cell2mat({msg.Polygons(i).Polygon.Points(:).X});
                y = cell2mat({msg.Polygons(i).Polygon.Points(:).Y});
                x = x; % + obj.obs_origin_(1,1);
                y = y; % + obj.obs_origin_(2,1);
                obj.obstacles_(:,i*6-5:i*6) = double([x x(1) nan;y y(1) nan]);
            end
            obj.obstacles_ = [obj.obstacles_ obj.world_obstacles_];
            %toc
        end
        function [au, ay, t0_offset, manu_type] = get_param_info(obj, k_params)
            au = k_params(1,1);
            ay = k_params(2,1);
            manu_type = k_params(4,1);
            if manu_type == 3
                t0_offset_per_idx = obj.frs_lan_info_.t0_dt;
            else
                t0_offset_per_idx = obj.frs_dir_info_.t0_dt;
            end
            t0_offset = (k_params(3,1) - 1) * t0_offset_per_idx;
        end
        function send_params(obj, k_params)
            if ~isempty(k_params)
                [au, ay, t0_offset, manu_type] = obj.get_param_info(k_params);
                fprintf("kparams(3,1): %.2f, t0_offset: %.2f\n", k_params(3,1), t0_offset);
                obj.gen_param_msg_.Au =       au;
                obj.gen_param_msg_.Ay =       ay;
                obj.gen_param_msg_.T0Offset = t0_offset;
                obj.gen_param_msg_.ManuType = manu_type;
                send(obj.gen_param_pub_, obj.gen_param_msg_);
            end
        end
        function update_state(obj, sub, msg)
            %disp('update_state')
            obj.agent_state_ = [msg.X; msg.Y; msg.H; %  [485; 6; 0; %
                                msg.U; msg.V; msg.R; msg.W];
            agent_x = obj.agent_state_(1,1);
            agent_y = obj.agent_state_(2,1);
            %obj.x_des_ =[5.0; 0.8]; %[agent_x + 3.0; agent_y + 2.0]; %
        end
        function callback_path(obj, sub, msg)
%            showdetails(msg)
           num_data_points = size(msg.Poses, 1);

           obj.curr_path_ = zeros(3, 0);
           for i=1:num_data_points
               
                quad_x = msg.Poses(i).Pose.Orientation.X;
                quad_y = msg.Poses(i).Pose.Orientation.Y;
                quad_z = msg.Poses(i).Pose.Orientation.Z;
                quad_w = msg.Poses(i).Pose.Orientation.W;
                [yawangle, pitch,roll]=quat2angle([quad_w, quad_x, quad_y, quad_z]);
%                 [yawangle,pitch,roll]
                desired_yaw = wrapToPi(yawangle);
               obj.curr_path_(:,end+1) = [msg.Poses(i).Pose.Position.X; ...
                                          msg.Poses(i).Pose.Position.Y;...
                                          desired_yaw];
           end
        end
    end
end
function [wps] = get_waypoint_from_path_var_lookahead(vehicle_pos, path, max_lookahead)
    [d,~,~,p_idx] = dist_point_on_polyline_simon(vehicle_pos(1:2),path(1:2,:)) ;
    d_best = dist_polyline_cumulative(path(1:2,:));
    reduce_curve_lookahead = 7;
    if (p_idx + round(reduce_curve_lookahead*20))< length(path(1,:))%20 points = 1m 
        vert = path(1:2,p_idx:p_idx+round(reduce_curve_lookahead*20))';
        k=LineCurvature2D(vert)/20;
        N=LineNormals2D(vert);
        figure(1);
%         plot([vert(:,1) vert(:,1)+k.*N(:,1)]',[vert(:,2) vert(:,2)+k.*N(:,2)]','m','LineWidth',3);
%         plot([vert(Lines(:,1),1) vert(Lines(:,2),1)]',[vert(Lines(:,1),2) Vertices(Lines(:,2),2)]','b');
        k = max(abs(LineCurvature2D(path(1:2,p_idx:p_idx+round(reduce_curve_lookahead*20))')));
        if k > 0.2
            lookahead_distance = 1.5*1;% 1.5 s peak velocity, 0.55 m/s
%         lookahead_distance = lookahead_distance_potential+min_lookahead;
        else
            lookahead_distance = 1.5*2;% 1.5 s peak velocity, 2m/s
        end 
    else
        lookahead_distance = 1;
%         "max lookahead"
%         lookahead_distance = 1;
    end

%     lookahead_distance
    heading_lookahead = lookahead_distance + 1;
    d_lkhd = min(d_best(end), d + lookahead_distance);
    wp = match_trajectories(d_lkhd,d_best,path);
    d_lkhd1 = min(d_best(end), d + lookahead_distance/3);
    wp1 = match_trajectories(d_lkhd1,d_best,path);
    d_lkhd2 = min(d_best(end), d + lookahead_distance/3*2);
    wp2 = match_trajectories(d_lkhd2,d_best,path);


    d_lkhd = min(d_best(end), d + heading_lookahead);
    wp_h = match_trajectories(d_lkhd,d_best,path);
    d_lkhd1 = min(d_best(end), d + heading_lookahead/3);
    wp_h1 = match_trajectories(d_lkhd1,d_best,path);
    d_lkhd2 = min(d_best(end), d + heading_lookahead/3*2);
    wp_h2 = match_trajectories(d_lkhd2,d_best,path);

    wp(3) = wp_h(3);
    wp1(3) = wp_h1(3);
    wp2(3) = wp_h2(3);
    wps = [wp wp1 wp2];
end 
            

function wp = get_waypoint_from_path(vehicle_pos, path, lookahead_distance)
    d = dist_point_on_polyline(vehicle_pos(1:2),path(1:2,:)) ;
    d_best = dist_polyline_cumulative(path(1:2,:));
    d_lkhd = min(d_best(end), d + lookahead_distance);
    wp = match_trajectories(d_lkhd,d_best,path);
    
end

function [A3, AH, bounds, obstacles] = build_helpers(frs, frs_low, frs_dir_info, frs_lan_info)
    verbose_level = 0;
    bounds = [-500,0,-0.7,12.7] ;
    %goal_radius = 6;
    %world_buffer = 1; % this is used to make start and goal locations that are not too close to the robot or boundary
    %HLP_buffer = 0.1;
    %RTD_HLP_grow_tree_mode ='seed';
    % planner
    %buffer = 0.3 ; % [m] distance used to buffer obstacles for planning, must be larger than the agent's footprint
    t_move =0.75;
    t_plan =0.75;
    t_failsafe_move = 0.75;
    %Ts = t_move;
    %Tf = Ts*150;
    % automated from here
    A3 = highway_cruising_7_state_agent('use_rover_controller',false);
    A3.integrator_type= 'ode45';
    A3.desired_initial_condition = [bounds(1)+10; 6; 0;9; 0; 0];
    
    %W = dynamic_car_world('bounds',bounds,'buffer',world_buffer,...
    %    'verbose',verbose_level,'goal_radius',goal_radius,'num_cars',12) ;%1 is ego , 2 means 1 obs
    HLP = highway_HLP('max_spd',15);%RRT_star_HLP('timeout',0.5,'bounds',bounds,'grow_tree_mode',RTD_HLP_grow_tree_mode,'verbose',verbose_level);
    AH_debug_flag = false;
    u0_vec = frs_dir_info.u0_vec;
    if exist('frs_low', 'var') && ~isempty(frs_low)
        u0_vec = horzcat(0:0.1:0.5, u0_vec);
        frs.M_mega = horzcat(frs_low.M_mega, frs.M_mega);
    end
    AH = highwayAgentHelper(A3,frs,HLP,'t_plan',t_plan,'t_move',t_move,'t_failsafe_move',t_failsafe_move,...
        'eps',0.001,'verbose',verbose_level,'plot_flag',AH_debug_flag);
    AH.v_array = u0_vec;
    obstacles = [];
end

function obs_out = gen_obs(x1, y1, w, h)
    obs_out = [
                x1-w y1-h;
                x1-w y1+h;
                x1+w y1+h;
                x1+w y1-h;
                x1-w y1-h;
                nan nan;
    ]';
end


