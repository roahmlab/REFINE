classdef JL_highwayAgentHelper < agentHelper
    %% properties
    properties
        % hard reference bounds for parameters
        upper_bd = 10;% y
        lower_bd = 2;% y
        spdlb = 5;% vx
        spdub = 30;%vx
        
        % epsilon for boundary check
        eps = 0.001;
        
        num_py_lane;
        num_py_dir;
        
        HLP;
        % reference for a single sampling time, needs this so there is a
        % old reference when a new reference comes in
        vx_des = 5; % default to 1m/s
        y_des;
        
        % from FRS, to determine initial condition and desired condition
        u0_array
        plot_flag = 1;
        pause_flag = 0;
        draw_subplots = 0;
        %get rid of this eventually
        
        
        % reference data for plot
        ref_Z = [];
        FRS_plotting_param = [];
        proposed_ref_Z = [];
        t_real_start = [];
        t_proposed_start = [];
        

        
        prev_action = -1;
        cur_t0_idx = 1;
        Ay_idx = 0;

        t_plan=1.5;
        t_timeout = 600;
        
        fminconopt = [];
        cont_opt = 1;
        constrain_force = 0;
        
        plot_ax;
        
        S 
        
        FRS_helper_handle = struct;
        
        dynamic_obs_plot; %figure 2 dynamics stuff;
        
        %         tmiddle_label
        truncating_factor;
%         zono_dt ;
        
        FRS_cost_function_idx = 150;
        parent_pointer;
        num_times_called = 0;
        saved_K = [];
        
        waypt_hist = [];
        K_hist = [];
        FRS_hist = {};
        mirror_hist = [];
        state_hist = [];
        type_manu_hist = [];
        time_hist = [];
        solve_time_hist = [];

    end
    %% methods
    methods
        function AH = JL_highwayAgentHelper(A,FRS_obj,HLP,varargin)
            AH@agentHelper(A,FRS_obj,varargin{:});
            AH.HLP = HLP;
            info_file_dir = load('dir_change_Ay_info.mat');
            AH.num_py_dir = info_file_dir.num_Ay;
            info_file_lan = load('lane_change_Ay_info.mat');
            AH.num_py_lane = info_file_lan.num_Ay;
            
            %             AH.y_des = AH.fA.state(2,end);
            %             AH.h_array = AH.zono_full.h_range; % array of range of acceptable parameter values
            %             AH.y_array = AH.zono_full.y_range;
            %             %             AH.v_array = AH.zono_full.v_range;
            AH.u0_array = info_file_dir.u0_vec; % have to start at 5, edit examine_FRS if you don't want to
            % start at 5
            
            %             AH.zono_full.v_range = AH.zono_full.v_range;
            %             AH.del_array = AH.zono_full.del_range;
            %             AH = parse_args(AH,varargin{:}) ;
            %             AH.A = A;
            %             AH.zono_full = load(FRS_path);%'zono_full_7.13_1spd.mat');
            
            o = optimoptions('fmincon') ;
            o.OptimalityTolerance = 1e-3;
            o.MaxIterations = 100000 ;
            o.MaxFunctionEvaluations = 10000 ;
            o.SpecifyConstraintGradient = true ;
            o.SpecifyObjectiveGradient = true;
            o.Display = 'off';
            o.CheckGradients = false;
            o.UseParallel = false;
            %             o.Algorithm = 'trust-region-reflective';
            AH.fminconopt = o;
%             AH.plot_ax = figure(3);
%             subplot(3,1,1);cla; hold on; axis equal;
%             subplot(3,1,2);cla; hold on; axis equal;
%             subplot(3,1,3);cla; hold on; axis equal;
            
            %             figure(1);subplot(4,4,[3,7,11,15])
            %             For ford only
            %AH.master_plot_handle_frs_0 = plot([0],[0],'-','Color',[242 255 151]/255,'Parent',AH.A.master_ax);%unsliced %not showing this, too much clutter
            %AH.master_plot_handle_frs_1 = plot([0],[0],'-y','Color',[255 178 102]/255,'Parent',AH.A.master_ax);%sliced with ini
            %AH.master_plot_handle_frs_2 = plot([0],[0],'-g','Parent',AH.A.master_ax);%sliced with ini and des
            
            AH.truncating_factor = 1;
%             AH.zono_dt = 0.02*AH.truncating_factor;%decides how fast dynamic obstacles move
%             AH.FRS_cost_function_idx = 75;% deprecated, this var depends on maneuver type now
            
            %             dir_predict = load('dir_change_direct_predict_model.mat');
            %             AH.px_dir = dir_predict.px;
            %             AH.py_dir = dir_predict.py;
            %             AH.dot_px_dir;
            %             syms x1 x2 x_des_only_x
            %             px_dir_fun = AH.px_dir.Coefficients * (x1.^AH.px_dir.ModelTerms(:,1) .* x2.^AH.px_dir.ModelTerms(:,2));
            %             px_cost_fun = @(x1, x2, x_des_only_x) (px_dir_fun - x_des_only_x)^2;
            %             px_cost_fun_deriv = jacobian(px_cost_fun, x2);
            %             AH.px_dir_fun_jacobian = @(a, b) subs(px_cost_fun_deriv, [x1 x_des_only_x], [a b]);
            %             py_dir_fun = AH.py_dir.Coefficients * (x1.^AH.py_dir.ModelTerms(:,1) .* x2.^AH.py_dir.ModelTerms(:,2));
            %             AH.py_dir_fun_jacobian = jacobian(AH.py_dir_fun, [x1 x2]);
            %             AH.predict_eval_fun = [px_dir_fun; py_dir_fun]
        end
        
        
        
        function [K,tout] = gen_parameter_standalone(AH, world_info, agent_state,waypts)
            % takes a user input parameter and obstacles in world info,
            % find a nearest action that is safe.(may be itself), also
            % return the distance in parameter space and return a replace
            % flag based on if a proper replace action is found
            x_des = waypts(:,1);
            
            if (AH.cur_t0_idx > 1 && AH.prev_action == 2) || (AH.cur_t0_idx > 2 && AH.prev_action == 3)|| AH.prev_action  == 1
                AH.prev_action = -1;
                AH.cur_t0_idx = 1;
            end

            if AH.prev_action ~= -1 %since t move 1.5s, this must be lane change
                K = [AH.saved_K(1); AH.saved_K(2); AH.cur_t0_idx ;AH.prev_action];
                AH.cur_t0_idx = AH.cur_t0_idx + 1;
                return
            end

            x_des_mex = x_des;
            dyn_obs_mex = get_obs_mex(world_info.dyn_obstacles, world_info.bounds);
            agent_state_mex = agent_state(1:6);
            tic
            k_mex = TEST_MEX(agent_state_mex, x_des_mex, dyn_obs_mex); 
            t_temp = toc;
            AH.solve_time_hist = [AH.solve_time_hist, t_temp];
            indices = k_mex(5:6)+1;
            k_mex = k_mex(1:4);
            K = k_mex;
            if K(end) == -1
                K = [];
            else % JL modified: visualize FRS and update for the agent
                type_manu = K(4);
                t0_idx = K(3);
                multiplier = 1;
                mirror_flag = 0;

                type_manu_all = ["Au","dir","lan"];
                type_text = type_manu_all(type_manu);
                [~,idxu0] = min(abs(AH.u0_array - agent_state(4)));
                M = AH.zono_full.M_mega{idxu0};
                
                if type_manu == 1
                    k = K(1);
                    FRS = M(type_text);
                    AH.prev_action = -1; 
                    AH.cur_t0_idx = 1;
                else
                    k = K(2);
                    if k<0
                        multiplier = -1;
                        mirror_flag = 1;
                    end
                    FRS = M(type_text); 
                    AH.prev_action = type_manu;
                    AH.cur_t0_idx = 2;
                    AH.saved_K = K;
                end
                if size(FRS,1) == 1
                    FRS = FRS';
                end
                FRS = FRS{indices(1),indices(2)};
                AH.plot_selected_parameter_FRS(k,type_manu,FRS,mirror_flag,agent_state,multiplier);
                
                AH.waypt_hist = [AH.waypt_hist x_des];
                AH.K_hist = [AH.K_hist k];
                AH.FRS_hist{end+1} = FRS;
                AH.mirror_hist = [AH.mirror_hist mirror_flag];
                AH.type_manu_hist = [AH.type_manu_hist type_manu];
                AH.state_hist = [AH.state_hist agent_state];
                AH.time_hist = [AH.time_hist AH.A.time(end)];
            end
            tout = 0;
%             K
            return
            
            %     AH.cur_t0_idx = 1;
            if (AH.cur_t0_idx > 1 && AH.prev_action == 2) || (AH.cur_t0_idx > 2 && AH.prev_action == 3)|| AH.prev_action  == 1
                AH.prev_action = -1;
                AH.cur_t0_idx = 1;
            end
            
            
%             if agent_state(4) > 1.34
%                 agent_state(4) = 1.34;
%             elseif agent_state(4)<0.55
%                 agent_state(4) = 0.551;
%             end
            
            O_all = world_info.obstacles;
%             dyn_O.time  = world_info.dyn_obstacles_time;
            dyn_O.obs   = world_info.dyn_obstacles;
            
            % find the index correesponding to the initial condition (v_ini, h_ini, delta_ini)
            % paramter range and desired parameter range(y_desired, and vd).
            
            % inefficient to generate bounds obstacle everytime
            bounds = world_info.bounds;
            xlo = bounds(1) ; xhi = bounds(2) ;
            ylo = bounds(3) ; yhi = bounds(4) ;
            
            Blower = [xlo, xhi, xhi, xlo, xlo ; ylo, ylo, ylo-1, ylo-1, ylo] ;
            Bupper = [xlo, xhi, xhi, xlo, xlo ; yhi, yhi, yhi+1, yhi+1, yhi] ;
            B = [Blower, nan(2,1), Bupper, nan(2,1)] ;% make top and bottom bounds into obstacles
            
            O = [O_all B] ;
            
            %find the action to replace starting from current zonotope
            %index
            %% rough TRajecotry Optimization
            load_const
            % REMOVED: ~exist x_des: generated x_des from HLP
            %find all options
            [~,idxu0] = min(abs(AH.u0_array - agent_state(4)));
%             if agent_state(4) >= 0.55
%                 [~,idxAuu0] = min(abs([0.7 1.0 1.3 1.6 1.9] - agent_state(4)));
%                 idxAuu0 = idxAuu0 * 3 + 5;
%             else
                idxAuu0 = idxu0;
%             en.d      
            M = AH.zono_full.M_mega{idxu0};
            MAu = AH.zono_full.M_mega{idxAuu0};
            tbAu = MAu('Autb');                                          %Ay x y h v v r r
            tbAu_garbage_idx = find(abs(tbAu(1,:) - 5.75)<0.01);
            tbdir = M('dirtb'); tbdir = repmat(tbdir,[1,2,1]);
            tblan = M('lantb'); tblan([2,9,12],:) = tblan([2,9,12],:)/2;
            tblan = repmat(tblan,[1,2,1]);
            if size(tbdir,1) > 4
                tbdir(logical([1 0 1 1 1 1 1 1 0 1 1 0 1 1]),AH.num_py_dir+1:end,:) = -tbdir(logical([1 0 1 1 1 1 1 1 0 1 1 0 1 1]),AH.num_py_dir+1:end,:);
            else
                tbdir(logical([1 0 1 1 ]),AH.num_py_dir+1:end,:) = -tbdir(logical([1 0 1 1 ]),AH.num_py_dir+1:end,:);
            end
            if size(tblan,1) > 4
                tblan(logical([1 0 1 1 1 1 1 1 0 1 1 0 1 1]),AH.num_py_lane+1:end,:) = -tblan(logical([1 0 1 1 1 1 1 1 0 1 1 0 1 1]),AH.num_py_lane+1:end,:);
            else
                tblan(logical([1 0 1 1]),AH.num_py_lane+1:end,:) = -tblan(logical([1 0 1 1]),AH.num_py_lane+1:end,:);
            end
            all_tb = {tbAu, tbdir, tblan};
            
            %% debug plots for maneuvers
            figure(7);clf;hold;
            quiver(waypts(1,1),waypts(2,1),0.1*cos(waypts(3,1)),0.1*sin(waypts(3,1)),'g-','AutoScale','on', 'AutoScaleFactor', 15);
            quiver(waypts(1,2),waypts(2,2),0.1*cos(waypts(3,2)),0.1*sin(waypts(3,2)),'g-','AutoScale','on', 'AutoScaleFactor',15);
            quiver(waypts(1,3),waypts(2,3),0.1*cos(waypts(3,3)),0.1*sin(waypts(3,3)),'g-','AutoScale','on', 'AutoScaleFactor', 15);
            tb_dir_world = local_to_world(agent_state(1:3), tbdir(2:4,:,1));
            % TODO use cur_t0_idx_print here for more accurate arrows, but
            % also will be out of bound for the not chosen maneuvers
            tb_Au_world = local_to_world(agent_state(1:3), tbAu(2:4,:,1));
            
            tb_lan_world = local_to_world(agent_state(1:3), tblan(2:4,:,1));
            if size(tbdir,1)>8
            tb_dir_world1 = local_to_world(agent_state(1:3), tbdir(9:11,:,1));
            tb_dir_world2 = local_to_world(agent_state(1:3), tbdir(12:14,:,1));
            tb_Au_world1 = local_to_world(agent_state(1:3), tbAu(9:11,:,1));
            tb_Au_world2 = local_to_world(agent_state(1:3), tbAu(12:14,:,1));
            tb_lan_world1 = local_to_world(agent_state(1:3), tblan(9:11,:,1));
            tb_lan_world2 = local_to_world(agent_state(1:3), tblan(12:14,:,1));
            quiver(tb_Au_world1(1,:),tb_Au_world1(2,:),cos(tb_Au_world1(3,:)),sin(tb_Au_world1(3,:)),'r-');
            quiver(tb_Au_world2(1,:),tb_Au_world2(2,:),cos(tb_Au_world2(3,:)),sin(tb_Au_world2(3,:)),'r-');
            quiver(tb_dir_world1(1,:),tb_dir_world1(2,:),cos(tb_dir_world1(3,:)),sin(tb_dir_world1(3,:)),'m-');
            quiver(tb_dir_world2(1,:),tb_dir_world2(2,:),cos(tb_dir_world2(3,:)),sin(tb_dir_world2(3,:)),'m-');
            quiver(tb_lan_world1(1,:),tb_lan_world1(2,:),cos(tb_lan_world1(3,:)),sin(tb_lan_world1(3,:)),'b-');
            quiver(tb_lan_world2(1,:),tb_lan_world2(2,:),cos(tb_lan_world2(3,:)),sin(tb_lan_world2(3,:)),'b-');
            end
            quiver(tb_Au_world(1,:),tb_Au_world(2,:),cos(tb_Au_world(3,:)),sin(tb_Au_world(3,:)),'r-');
            quiver(tb_dir_world(1,:),tb_dir_world(2,:),cos(tb_dir_world(3,:)),sin(tb_dir_world(3,:)),'m-');
            quiver(tb_lan_world(1,:),tb_lan_world(2,:),cos(tb_lan_world(3,:)),sin(tb_lan_world(3,:)),'b-');
            quiver(agent_state(1),agent_state(2),cos(agent_state(3)),sin(agent_state(3)),'k-');
            AH.num_times_called = AH.num_times_called+1;
            axis equal
            %                 scatter(AH.parent_pointer.x_des_stored_(1),AH.parent_pointer.x_des_stored_(2),50,[0.92 0.69 0.12],'Filled');
            %                 text(AH.parent_pointer.x_des_stored_(1),AH.parent_pointer.x_des_stored_(2),num2str(AH.num_times_called));
%             AH.num_times_called=AH.num_times_called+1;
            
            K= [];type_manu_all = ["Au","dir","lan"];
            tsolvetic = tic;
            
            if AH.prev_action ~= -1 %since t move 1.5s, this must be lane change
%                 type_text = type_manu_all(AH.prev_action);
%                 FRS = M(type_text);
%                 K = AH.highway_sampling_opt( x_des, FRS,all_tb{AH.prev_action},AH.Ay_idx,O, agent_state, AH.prev_action,AH.cur_t0_idx,traffic_data,waypts);
%                 if isempty(K)
%                     AH.prev_action = -1;
%                     warning('Continuing previous action failed')
%                     %                     AH.cur_t0_idx = 1;
%                     %                     AH.Ay_idx = 0;
%                 else
                    K = [AH.saved_K(1); AH.saved_K(2); AH.cur_t0_idx ;AH.prev_action];
                    AH.cur_t0_idx = AH.cur_t0_idx + 1;
%                 end
            end
            if AH.prev_action == -1
                AH.cur_t0_idx = 1; % table here has extra cols for mirroring
                [dAu] = rough_manu_cost_estimate(tbAu, agent_state,waypts,AH.cur_t0_idx);
                [ddir] = rough_manu_cost_estimate(tbdir,agent_state,waypts,AH.cur_t0_idx);
                [dlan] = rough_manu_cost_estimate(tblan,agent_state,waypts,AH.cur_t0_idx);
                %                 ddir= 1000;% here we can toggle what actions we want
                dAu = dAu + abs(tbAu(1,:) - agent_state(4));
                dAu(tbAu_garbage_idx) = 1000;
                  ddir([2,4]) = 1000;% limit the large lateral dir changes, don't need those
%                 dAu (end-10:end) = 1000;
                if abs(agent_state(3))>0.01
                    ddir = ddir - 100;%prefer to straighten when heading too large
                end
                if min(abs(agent_state(2)-[0,3.7,3.7*2])) >0.2
                    dlan = dlan - 100;
                end

                cannot_move_lateral_speed = 6;
                if agent_state(4) < cannot_move_lateral_speed
                    ddir = 1000;
                    dlan = 1000;
                    

                end
%                 if (agent_state(1) > 6 && agent_state(2) < 6) || (agent_state(1) > 6 && agent_state(2) > 32) ||(agent_state(1) < -12 && agent_state(2) > 32) ||...
%                         (agent_state(1) < -12 && agent_state(2) < 6) 
%                     dlan = 1000;
% %                     dAu = 1000;
%                 end
                %                 dlan = 1000;
%                 dAu = 1000;
                
                %          dAu  = 1000;
                %         if length(dAu) >1
                %             v_array_original = 0.6:0.1:1.3;
                %             dAu(v_array_original < cannot_move_lateral_speed +0.1) = 1000;
                % %             dAu(end-1:end) = 1000;
                %         end
                %                 dAu = 1000;
                %                 ddir= 1000;
                
                all_dist = {dAu, ddir, dlan};
%                 cnt = 0;
                while any(all_dist{1}<1000) || any(all_dist{2}<1000) || any(all_dist{3}<1000)
%                     cnt = cnt + 1
                    [minAud, Auidx] = min(all_dist{1});
                    [mindird, diridx] = min(all_dist{2});
                    [minland, lanidx] = min(all_dist{3});
                    idx_arr = [Auidx diridx lanidx];
                    [~,type_manu]=min([minAud mindird minland]);
                    %                     if type_manu == 1
                    %                         mirror_flag = -1;
                    %                     end
                    %             if type_manu == 1 && tbAu(1,idx_arr(type_manu)) == 5 % this is to skip things that go to 5
                    %                 all_dist{type_manu}(idx_arr(type_manu)) = 1000;
                    %                 continue;
                    %             end
                    
                    type_text = type_manu_all(type_manu);
                    if type_manu == 1
                        FRS = MAu(type_text);
                    else
                        FRS = M(type_text);                     %here is original table
                    end
                    %highway_sample_opt should not handle
                    K = AH.highway_sampling_opt( x_des, FRS, all_tb{type_manu}, idx_arr(type_manu), O,dyn_O, agent_state, type_manu, AH.cur_t0_idx, traffic_data,waypts);
                    if isempty(K)
                        all_dist{type_manu}(idx_arr(type_manu)) = 1000;
                    else
                        if type_manu == 1
                            %Ku, Ky, t0_idx, manu_type
                            K = [K; 0;1;type_manu];
                            AH.prev_action = -1;
                            AH.cur_t0_idx = 1;
                        else
                            K = [agent_state(4);K;AH.cur_t0_idx;type_manu];
                            AH.prev_action = type_manu;
                            AH.cur_t0_idx = 2;
                            AH.Ay_idx = idx_arr(type_manu);
                            AH.saved_K = K;
                        end
                        break;
                    end
                end
            end
            K
            tout = toc(tsolvetic);
        end
        
        function [K]= highway_sampling_opt(AH, x_des, FRS,tbdir,diridx_with_mirror,O, dyn_O, agent_state, type_manu,t0_idx, traffic_data,wayypts)
            %x_des, FRS,all_tb{type_manu},idx_arr(type_manu),O, agent_state, type_manu,t0_idx
            K =[];
            %             if norm(x_des(1:2) - agent_state(1:2)) < 0.5
            %                 return;
            %             end
            if (diridx_with_mirror > AH.num_py_dir  && type_manu == 2) || (diridx_with_mirror > AH.num_py_lane && type_manu ==  3)
                mirror_flag = 1;
                if (diridx_with_mirror > AH.num_py_dir  && type_manu == 2)
                    diridx = diridx_with_mirror - AH.num_py_dir;
                else
                     diridx = diridx_with_mirror - AH.num_py_lane;
                end
                agent_state(5:6) = -agent_state(5:6);
                %                 agent_state(3) = -agent_state(3);
                multiplier = -1;
                
            else
                mirror_flag = 0;
                diridx = diridx_with_mirror;
                multiplier = 1;
            end
            %enable initial check condition r and v if table is extended
            if size(tbdir,1) > 4
                %                 warning('inital condition check disabled')
                
                
                v_lb = tbdir(5,diridx,t0_idx);
                v_ub = tbdir(6,diridx,t0_idx);
                v_eps = abs(v_ub - v_lb) / 10.0;
                if agent_state(5) < v_lb || agent_state(5) > v_ub
                    warning('v0 out of bound')
                    return
                end
                % if previous returned, the following code won't run
                if agent_state(5) < v_lb
                    agent_state(5) = v_lb + v_eps;
                elseif agent_state(5) > v_ub
                    agent_state(5) = v_ub - v_eps;
                end
                
                r_lb = tbdir(7,diridx,t0_idx);
                r_ub = tbdir(8,diridx,t0_idx);
                r_eps = abs(r_ub - r_lb) / 10.0;
                if agent_state(6) < r_lb || agent_state(6) > r_ub
                    warning('r0 out of bound')
                    return
                end
                if agent_state(6) < r_lb
                    agent_state(6) = r_lb + r_eps;
                elseif agent_state(6) > r_ub
                    agent_state(6) = r_ub - r_eps;
                end
            end
            
            %             x_des
            x_des = world_to_local_mirror(agent_state,x_des, mirror_flag);
            wayypts = world_to_local_mirror(agent_state,wayypts, mirror_flag);
            %             x_des(2) = multiplier * x_des(2);%is this correct?
            %Above line really not sure
            dummy_state = agent_state;
            if ~isa(AH, 'highwaySOSAgentHelper')
%                 dummy_state(3) =  0;%for zono, rotation is done when geerating obs
                %!! We no longer use zero heading. we can get proper
                %generate for obstacles in inner loop now
                %for sos, do rotation here;
            end
            if isempty(O)
                warning('empty obstacles!')
                return
            end
            
            %% filter out out of bound obs: turn off for now
%             Otest= reshape(O,2,6,[]);
%             for obs_idx = size(Otest,3):-1:1
%                 if all(Otest(2,1:5,obs_idx)< -40)
%                     Otest(:,:,obs_idx) = [] ;
%                 end
%                 
%             end
            
%             O = reshape(Otest,2,[]);
            O(1,min(O(1,:))==O(1,:)) = dummy_state(1);
            O(1,max(O(1,:))==O(1,:)) = dummy_state(1)+100;% change x value of the boundary obs 
            O = world_to_local_mirror(dummy_state,O,mirror_flag );
%             for obs_time_idx = 1:length(dyn_O.obs)
            dyn_O.obs{1} = world_to_local_mirror(dummy_state,dyn_O.obs{1},mirror_flag );
%             end
            
            %             Otest= reshape(O,2,6,[]);
            %             for obs_idx = size(Otest,3):-1:1
            %                 if all(Otest(1,:,obs_idx)> 8)  || all(Otest(2,:,obs_idx) > 3) || all(Otest(2,:,obs_idx) <-3)
            %                     Otest(:,:,obs_idx) = [] ;
            %                 end
            %
            %             end
            %
            %             O = reshape(Otest,2,[]);
            %             size(O)
            %             if Otest
            %             delete_idx_obs = [];
            %             for obs_idx = 1:size(O,2)/6
            %                 if O(1,(obs_idx-1)*6+1) < -0.5
            %                     delete_idx_obs = [delete_idx_obs obs_idx];
            %                 end
            %             end
            %             O(:,delete_idx_obs)
%             if size(traffic_data, 1) >= 2
%                 traffic_data(1:2,:) = world_to_local_mirror(dummy_state,traffic_data(1:2,:), mirror_flag);
%             end
            
            
            if size(FRS,1) == 1
                FRS = FRS';
            end
            FRS = FRS{diridx,t0_idx};
            if AH.cont_opt                                                                    %center value
                if size(tbdir,2) == 1
                    param_gen = 0.05;
                else
                    param_gen = abs((tbdir(1,2,t0_idx)-tbdir(1,1,t0_idx))/2);
                end
                [K] = AH.find_optimal_cont(O,agent_state,FRS,mirror_flag,type_manu,tbdir(1,diridx,t0_idx),param_gen,x_des, dyn_O);
                K = multiplier * K;
            else
                
                %below all in postive y change frame
                [safeK] = AH.find_safe_actionset(O,dyn_O,agent_state,FRS,mirror_flag,type_manu);
                if ~isempty(safeK)
                    cost_arr = zeros(length(safeK),1);
                    if size(tbdir,2) == 1
                        gensize = 0.05;
                    else
                        gensize = abs((tbdir(1,2,t0_idx)-tbdir(1,1,t0_idx))/2);
                    end
                    safeK =(safeK.*gensize+tbdir(1,diridx,t0_idx));
                    if type_manu == 1
                        slice_idx = 11;
                    else
                        slice_idx = 12;
                    end
                    for i = 1:length(safeK)
%                         if type_manu == 1
%                             FRS_cost_function_idx = 156;% these are the numbers obtained from examin frs during offline
%                         else
%                             FRS_cost_function_idx = 77;
%                         end
                        c = center(zonotope_slice(FRS.vehRS_save{round(FRS.brake_idx1)}, [7;8;9;slice_idx], [agent_state(4);agent_state(5);agent_state(6);safeK(i)]));
                        c1 = center(zonotope_slice(FRS.vehRS_save{round(FRS.brake_idx1/3)}, [7;8;9;slice_idx], [agent_state(4);agent_state(5);agent_state(6);safeK(i)]));
                        c2 = center(zonotope_slice(FRS.vehRS_save{round(FRS.brake_idx1/3*2)}, [7;8;9;slice_idx], [agent_state(4);agent_state(5);agent_state(6);safeK(i)]));
                        cost_arr(i) = highway_cost_fun(c(1:3),wayypts(:,1)) + highway_cost_fun(c1(1:3),wayypts(:,2)) + highway_cost_fun(c2(1:3),wayypts(:,3));
                    end
                    [~,Kidx] = min(cost_arr);
                    K = safeK(Kidx);
                    %now go back to unmirrored frame
                    K = multiplier *K;
                else
                    K = [];
                end
            end
            %             if AH.plot_flag
            if ~isempty(K)
                figure(1);
                %                 figure(2);hold on;
                %                 dummy_state = agent_state(1:3);
                %                 dummy_state(3) = -agent_state(3);
                %                 O_world = local_to_world(dummy_state,O);
                
                AH.plot_selected_parameter_FRS(K,type_manu,FRS,mirror_flag,agent_state,multiplier);
                AH.waypt_hist = [AH.waypt_hist reshape(wayypts,[9,1])];
                AH.K_hist = [AH.K_hist K];
                AH.FRS_hist{end+1} = FRS;
                AH.mirror_hist = [AH.mirror_hist mirror_flag];
                AH.type_manu_hist = [AH.type_manu_hist type_manu];
                AH.state_hist = [AH.state_hist agent_state];
                AH.time_hist = [AH.time_hist AH.A.time(end)];
                %                  AH.parent_pointer.obstacles_
                %                  plot(AH.parent_pointer.obstacles_stored_(1,:),AH.parent_pointer.obstacles_stored_(2,:),'r-');
                %                     plot(AH.parent_pointer.curr_path_(1,:),AH.parent_pointer.curr_path_(2,:),'k-');
                %                  scatter(AH.parent_pointer.predicted_agent_state_stored_(1),AH.parent_pointer.predicted_agent_state_stored_(2),50,'r','Filled');
                %                  scatter(AH.parent_pointer.agent_state_stored_(1),AH.parent_pointer.agent_state_stored_(2),50,'r');
                %                  text(AH.parent_pointer.agent_state_stored_(1),AH.parent_pointer.agent_state_stored_(2),num2str(AH.num_times_called));
                %                  text(AH.parent_pointer.predicted_agent_state_stored_(1),AH.parent_pointer.predicted_agent_state_stored_(2),num2str(AH.num_times_called));
                
                %                 axis equal
                drawnow
                pause(0.1);
            end
            %             end
        end
        function plot_selected_parameter_FRS(AH,K,type_manu,FRS,mirror_flag,agent_state,multiplier)
            if ~isempty(K)
                %clear data and then plot
                AH.FRS_helper_handle.XData = cell(3,1);
                AH.FRS_helper_handle.YData = cell(3,1);
                if type_manu == 1
                    %                     AH.plot_zono_collide_sliced(FRS,mirror_flag,agent_state,[K; 0],[0 0 1],0);
                    %                     AH.plot_zono_collide_sliced(FRS,mirror_flag,agent_state,[K; 0],[0 0 1],1);
                    AH.plot_zono_collide_sliced(FRS,mirror_flag,agent_state,[K; 0],[0 0 1],2);
                else
                    %                     AH.plot_zono_collide_sliced(FRS,mirror_flag,agent_state,[agent_state(4);K *multiplier],[0 1 0],0);
                    %                     AH.plot_zono_collide_sliced(FRS,mirror_flag,agent_state,[agent_state(4);K *multiplier],[0 1 0],1);
                    AH.plot_zono_collide_sliced(FRS,mirror_flag,agent_state,[agent_state(4);K *multiplier],[0 1 0],2);
                end
            end
        end
        function [c, ceq, gc, gceq] = eval_zono_highway_cons(AH,K,A_con,b_con,s_con,c_k, g_k,start_tic,timeout)
            c = [];
            ceq = [];
            gc = [];
            gceq = [];
            lambdas = (K - c_k)./g_k; % given a parameter, get coefficients on k_slc_G generators
            for k = 1:length(A_con)
                Zk_tmp = A_con{k}*lambdas - b_con{k}; % A*lambda - b <= 0 means inside unsafe set
                [min_c, min_i]=min(-Zk_tmp(1:s_con{k}(1)));
                c = [c min_c]; % this and below can be combined, but lazy so leave them here
                gc= [gc -A_con{k}(min_i,:)'];
                for m = 1: length(s_con{k})-1
                    [min_c, min_i] = min(-Zk_tmp(s_con{k}(m)+1:s_con{k}(m+1)));
                    c = [c min_c];
                    gc = [gc -A_con{k}(min_i+s_con{k}(m),:)'];
                end
            end
            %             figure(3); histogram(c); drawnow;figure(2); max(c)
            %             lambdas = lambdas_arr(lamidx); % given a parameter, get coefficients on k_slc_G generators
            %                 for k = 1:length(A_con)
            %                     Zk_tmp = A_con{k}*lambdas - b_con{k}; % A*lambda - b <= 0 means inside unsafe set
            %                     safe = max(Zk_tmp(1:s_con{k}(1))) > 0;
            %                     for m = 1: length(s_con{k})-1
            %                         if ~safe
            %                             break %break the m loop for obstacle;
            %                         end
            %                         safe = max(Zk_tmp(s_con{k}(m)+1:s_con{k}(m+1))) > 0;
            %                     end
            %                     if ~safe
            %                         break % %break the k loop for time idx;
            %                     end
            %                 end
            %                 if safe
            if toc(start_tic) > timeout
                %                 AH.timeout_err_counter = AH.timeout_err_counter+1;
                %                 fprintf("timed out Ay/Au = %.1f \n",c_k);
                error('Timed out while evaluating constraint function!')
            end
            %we need max(Ax -b)> 0, since fmincon requires nonlin<=0
            %we specify constraint as min(b-Ax)<=0
        end
        function [avaliable_action_set] = find_optimal_cont(AH,O,agent_state,FRS,mirror_flag,AuAyflag, zono_c, zono_g,x_des,dyn_O)
            %             for i = 1:length(FRS.delta_force)
            %                 FRS.delta_force{i} =  deleteAligned(FRS.delta_force{i});
            %             end
            start_tic = tic;
            [A_con,b_con,s_con] = AH.generate_constraints(agent_state,O,FRS.vehRS_save,mirror_flag,AuAyflag,dyn_O);
            if AH.constrain_force
                %Here choose FRS based on when is the one at 3 seconds, you
                %since each is 0.02 sec, the brake one lives at #150
                [A_other,b_other,s_other] = AH.generate_other_constraints(agent_state,AH.FRS_cost_function_idx,FRS.delta_force,AuAyflag);
                A_con =[A_con A_other]; b_con = [b_con b_other]; s_con = [s_con s_other];
            end
            timeout_t_pk = AH.t_timeout;
            
            if AuAyflag == 1
                slice_idx = 11;
            else
                slice_idx = 12;
            end
            
            cons = @(K) AH.eval_zono_highway_cons(K, A_con, b_con, s_con,zono_c,zono_g,start_tic, timeout_t_pk) ;
            cost = @(K) highway_fmincon_cost_fun(K,agent_state, slice_idx,FRS.vehRS_save{AH.FRS_cost_function_idx},x_des);
            %             cost = @(K) highway_fmincon_cost_fun(K, x0, x_des);
            lb =[zono_c - zono_g];
            ub =[zono_c + zono_g];
            
            initial_guess = 0.5*(ub+lb);% - AH.eps;
            
            try
                [k,fval,exitflag,~] = fmincon(cost, initial_guess, [], [],...
                    [], [], lb, ub, cons,AH.fminconopt) ;
            catch
                exitflag = -1;%timeout
                warning ('optimization times out')
            end
            %
            if exitflag == 1 || exitflag == 2
                avaliable_action_set = k;
                toc(start_tic)
            else
                avaliable_action_set = [];
            end
        end

        
        
        
        function K = convert_action_to_parameter(AH,action,discrete_flag)
            % called in parent class. differnet for each helper
            delta_struct = struct;
            if discrete_flag == true
                if any(newAct == [0 4 8])%M
                    delta_struct.vx_des = 0;
                elseif any(newAct == [1])%A
                    delta_struct.vx_des = 2;
                elseif any(newAct == [2])%B
                    delta_struct.vx_des = -2;
                elseif any(newAct == [3])%H
                    delta_struct.vx_des = -4;
                else
                end
                
                if any(newAct == [4])
                    delta_struct.y_des = 1;
                elseif any(newAct == [8])
                    delta_struct.y_des = -1;
                else
                    delta_struct.y_des = 0;
                end
            else
                delta_struct.vx_des = (action(1)+1)/2*6-4;   %%%%%%  vx
                delta_struct.y_des = action(2)*0.59;%+1)/2*8+2;    %%%%%%  y
            end
            
            if delta_struct.vx_des > 2
                delta_struct.vx_des = 2;
            elseif delta_struct.vx_des < -4
                delta_struct.vx_des = -4;
            end
            
            if delta_struct.y_des > 1
                delta_struct.y_des = 1;
            elseif delta_struct.y_des < -1
                delta_struct.y_des = -1;
            end
            
            %above is for delta vx and delta y, the following is for actual
            %parameters,      : vx and delta y
            K = AH.update_desired_parameters(delta_struct);
            K(2) = delta_struct.y_des; %skipping the previous step y change check!!
            
        end
        function [T, U, Z]=gen_ref(AH, K, real_reference_flag,agent_state, ref_time)
            % generate reference based on parameter and states
            if ~exist('agent_state','var')
                agent_info = AH.get_agent_info();
                agent_state = agent_info.state(:,end);
            end
            if ~exist('ref_time','var')
                ref_time = AH.A.time(end);
            end
            if ~exist('real_reference_flag','var')
                real_reference_flag = 1;
            end
            u_cur = agent_state(4) ;
            %             h_cur = agent_state(3) ;
            y_cur = agent_state(2) ;
            x_cur = agent_state(1) ;
            Au = K(1);%K = [agent_state(4); K;t0_idx;type_manu];
            Ay = K(2);
            t0_idx = K(3);
            
            t0 = (t0_idx-1)*AH.t_move;
            type_manu = K(4);
            load_const
            %not symbolic mode time argument will be ignored
            %del_y,Au,u0,t,symbolic_flag, scale_Ay_flag                t0
            if type_manu == 3
                [T, U,Z] = gaussian_T_parameterized_traj_with_brake(t0,Ay,Au,u_cur,[],0,1);
            else
                [T, U,Z] = sin_one_hump_parameterized_traj_with_brake(t0,Ay,Au,u_cur,[],0,1);
            end
            
            if real_reference_flag
                AH.ref_Z=[AH.ref_Z;x_cur+Z(1,:);y_cur+Z(2,:)];% for plotting
                AH.t_real_start = [AH.t_real_start;ref_time];
            else
                AH.proposed_ref_Z=[AH.proposed_ref_Z;x_cur+Z(1,:);y_cur+Z(2,:)];% for plotting
                AH.t_proposed_start = [AH.t_proposed_start;ref_time];
            end
            
            
        end
        
        %% helper functions
        function parameters=update_desired_parameters(AH,delta_struct)
            parameters = zeros(2,1);
            agent_info = AH.get_agent_info();
            agent_state = agent_info.state(:,end);
            v_cur = agent_state(4) ;
            %             h_cur = agent_state(3) ;
            y_cur = agent_state(2) ;
            %             x_cur = agent_state(1) ;
            %             replaced_flag = 0;
            
            if abs(delta_struct.vx_des)> AH.eps
                AH.vx_des = round(v_cur,1) + delta_struct.vx_des;
            end
            if  AH.vx_des > AH.spdub
                AH.vx_des= AH.spdub;
            end
            if AH.vx_des < AH.spdlb
                AH.vx_des = AH.spdlb;
            end
            if abs(delta_struct.y_des)> AH.eps
                AH.y_des = y_cur + delta_struct.y_des;    % vy_cur
            end
            if  AH.y_des > AH.upper_bd
                AH.y_des= AH.upper_bd;
            end
            if AH.y_des < AH.lower_bd
                AH.y_des = AH.lower_bd;
            end
            parameters(1) = AH.vx_des;
            parameters(2) = AH.y_des-y_cur;
        end
        function plot(AH)
            hold_check = false ;
            if ~ishold
                hold_check = true ;
                hold on
            end
                if ~isempty(AH.planned_path)
                   plot(AH.planned_path(1,:),AH.planned_path(2,:),'k-','LineWidth',1);
%                             plot(AH.proposed_ref_Z(end-1,:),AH.proposed_ref_Z(end,:),'Color','y','LineWidth',3,'LineStyle','--');
                end
            %                 %                 xlim([AH.ref_Z(1,1)-20,AH.ref_Z(1,1)+30]);
            %             end
            %             if ~isempty(AH.ref_Z)
            %                 plot(AH.ref_Z(end-1,:),AH.ref_Z(end,:),'Color',[0 0 0],'LineStyle','-','LineWidth',3);
            %                 %                 plot(AH.ref_Z(end-1,:),AH.ref_Z(end,:),'g--','LineWidth',2);
            %
            %                 %                 xlim([AH.ref_Z(1,1)-20,AH.ref_Z(1,1)+30]);
            %             end
            text(-250,15,"u="+num2str(AH.A.state(4,end))+"m/s",'Color','red','FontSize',15)
            %             text(-250,-20,"v="+num2str(AH.A.state(5,end))+"m/s",'Color','red','FontSize',13)
            
            if hold_check
                hold off ;
            end
            
        end
        

        
        function reset(AH,flags,eps_seed)
            if ~exist('eps_seed','var')
                AH.A.reset();
            else
                rng(eps_seed)
                AH.A.reset();
            end
            AH.y_des = AH.A.state(2,end);
            AH.vx_des = AH.A.state(4,end);
            AH.flags = flags;
            AH.ref_Z = [];
            AH.proposed_ref_Z = [];
            AH.t_real_start = [];
            AH.t_proposed_start = [];
            AH.K_hist = [];
            AH.waypt_hist = [];
            AH.FRS_hist = {};
            AH.mirror_hist = [];
            AH.state_hist = [];
            AH.type_manu_hist = [];
            AH.time_hist = [];
            if ~isempty(AH.HLP)
                AH.HLP.reset();
                if ~isempty(AH.HLP.plot_data.waypoints)
                    AH.HLP.plot_data.waypoints.XData = [];
                    AH.HLP.plot_data.waypoints.YData = [];
                end
                %                 AH.HLP.plot_data.current_waypoint = [];
            end
        end
        
        
        function plot_zono_collide_sliced(AH,FRS,mirror_flag,agent_state,K,color,slice_level)
            %slice level 0, don't slice
            % 1, slice initial condition
            % 2, slice initial and desired
            if K(2) == 0
                slice_dim = 11;
                k_slice = K(1);
            else
                slice_dim = 12;
                k_slice = K(2);
            end
            if AH.draw_subplots
%                 if mirror_flag
%                     figure(2);subplot(3,1,3);
%                 else
%                     figure(2);subplot(3,1,2);
%                 end
                for t_idx = 1:10/AH.truncating_factor:length(FRS.vehRS_save)
                    zono_one = zonotope_slice(FRS.vehRS_save{t_idx}, [7;8;9;slice_dim], [agent_state(4);agent_state(5);agent_state(6);k_slice]);
                    h = plot(zono_one,[1,2],'Color',color);
                end
                
%                 figure(2); subplot(3,1,1);
            end
            
            for t_idx = 1:5:length(FRS.vehRS_save) % 1:10/AH.truncating_factor:length(FRS.vehRS_save)
                if slice_level == 0
                    zono_one = FRS.vehRS_save{t_idx};
                elseif slice_level == 1
                    zono_one = zonotope_slice(FRS.vehRS_save{t_idx}, [7;8;9], [agent_state(4);agent_state(5);agent_state(6)]);
                elseif slice_level == 2
                    zono_one = zonotope_slice(FRS.vehRS_save{t_idx}, [7;8;9;slice_dim], [agent_state(4);agent_state(5);agent_state(6);k_slice]);
                else
                    error('unknown slice_level in plot selected zono');
                end
                h = plot(zono_one,[1,2],'Color',color);
                if mirror_flag
                    h.YData = - h.YData;
                end
                XY = [h.XData(:) h.YData(:)];                                     % Create Matrix Of Vectors
                theta = agent_state(3);
                R=[cos(theta) -sin(theta); sin(theta) cos(theta)]; %CREATE THE MATRIX
                rotXY=XY*R'; %MULTIPLY VECTORS BY THE ROT MATRIX
                Xqr = reshape(rotXY(:,1), size(h.XData,1), []);
                Yqr = reshape(rotXY(:,2), size(h.YData,1), []);
                %SHIFTING
                h.XData = Xqr+agent_state(1);
                h.YData = Yqr+agent_state(2);
                
                AH.FRS_helper_handle.XData{slice_level+1} = [h.YData nan AH.FRS_helper_handle.XData{slice_level+1}];
                AH.FRS_helper_handle.YData{slice_level+1} = [h.XData nan AH.FRS_helper_handle.YData{slice_level+1}];
            end
            
            if AH.constrain_force
                figure(2);
                for t_idx = 4:2:AH.FRS_cost_function_idx
                    zono_one = zonotope_slice(FRS.delta_force{t_idx}, [7;8;9;slice_dim], [agent_state(4);agent_state(5);agent_state(6);k_slice]);
                    plot(zono_one,[1,2],'g');
                end
            end
            
%             figure(2);
        end
        
        
        function [A_con,b_con,s_con] = generate_constraints(AH, agent_state,O_pts,FRS, mirror_flag, AuAyflag,dyn_O)
            if ~exist('mirror_flag','var')
                mirror_flag = 0;
            end
            zono_peak_mid_stop  = FRS;
            n = length(zono_peak_mid_stop);
            
            
            
            O_pts = O_pts';
            obs_dim = [1; 2]; % note that the obstacle exists in the x-y space (not theta or v)
            if AuAyflag == 1
                k_dim = 11;
                plotc = [0.8,0.8,1];
            else
                if AuAyflag == 2
                    plotc = [0.8,1,0.8];
                else
                    plotc = [1,0.8,0.8];
                end
                k_dim = 12;
            end
            buffer_dist = 0; % assume no buffer.
            %
            A_con = {};
            b_con = {};
            s_con = {};
            if mirror_flag
                multipler = -1;
            else
                multipler = 1;
            end
            if AH.plot_flag
                if AH.draw_subplots
                    if mirror_flag
                        figure(2);subplot(3,1,3)
                    else
                        figure(2);subplot(3,1,2)
                    end
                    AH.dynamic_obs_plot = plot([0],[0],'m');
                end
            end
            
            for t_idx = 1:5:n
                if AH.plot_flag && AH.draw_subplots
                    if mod(t_idx,ceil(10/AH.truncating_factor)) == 0
                        zono_one = zonotope_slice(zono_peak_mid_stop{t_idx}, [7;8;9], [agent_state(4);agent_state(5);agent_state(6)]);
                        plot(zono_one,[1,2],'Color',plotc);
                        AH.dynamic_obs_plot.XData = 0;
                        AH.dynamic_obs_plot.YData = 0;
                    end
                end
                zono_one = zono_peak_mid_stop{t_idx};
                
                % bound the values so slicing won't have problem, max
                % values determined by simulating beforehand
                %                 y_des_soft = bound_values(AH.y_des - agent_state(2),AH.zono_full.y_range(y_des_idx)-AH.zono_full.kyg+AH.eps, AH.zono_full.y_range(y_des_idx)+AH.zono_full.kyg-AH.eps);% can't slice on edge
                %                 vx_des_soft =bound_values(AH.vx_des,AH.zono_full.v_range(vd_idx)-AH.zono_full.kvdg+AH.eps, AH.zono_full.v_range(vd_idx)+AH.zono_full.kvdg-AH.eps);
                %                 vi_soft =bound_values(agent_state(4),AH.zono_full.v_range(v_ini_idx)-AH.zono_full.kvig+AH.eps, AH.zono_full.v_range(v_ini_idx)+AH.zono_full.kvig-AH.eps);
                %                 h_soft = bound_values(agent_state(3),AH.zono_full.h_range(h_ini_idx)-AH.zono_full.h_ini_range+0.001,AH.zono_full.h_range(h_ini_idx)+AH.zono_full.h_ini_range-0.001);
                zono_one = zonotope_slice(zono_one, [7;8;9], [agent_state(4);agent_state(5);agent_state(6)]);
                
                % get error data from table
                %                 center_bb = error_data_cur(1:2,t_idx);
                %                 h =  error_data_cur(3,t_idx);
                %                 gen = error_data_cur(4:5,t_idx);
                %                 len = gen(1);
                %                 width = gen(2);
                %                 ego_gen = [[cos(h)*len; sin(h)*len], [sin(-h)*width; cos(-h)*width]];
                %                 gen_err = zeros(7,2);gen_err(1:2,1:2) = ego_gen;
                %                 err_zono  = zonotope([[center_bb(1);center_bb(2);0;0;0;0;0], gen_err]);
                
                %
                %
                Z = zono_one.Z;
                A_obs_array= []; b_obs_array = [];size_array=[]; size_idx = 0;
                c = Z(obs_dim, 1);
                G = Z(:, 2:end);
                for k_idx = 1:length(k_dim)
                    [~, k_col(k_idx)] = find(G(k_dim(k_idx), :) ~= 0); % find "k-sliceable" generators
                end
                k_slc_G = G(obs_dim, k_col);
                k_no_slc_G = G(obs_dim, :);
                k_no_slc_G(:, k_col) = [];
                agent_heading = multipler * agent_state(3);

                %consider each obstacle as a halfspace
                for obs_idx = 1:(size(O_pts,1)+1)/6
                    one_obs = O_pts((obs_idx-1)*6+1:obs_idx*6-1,:);
                    
                    
                    if max(one_obs(:,1)) < -20 || min(one_obs(:,1)) > 250
                        %                     if(max(one_obs(:,1)) - min(one_obs(:,1)) ) < 100 || max(one_obs(:,2))>0
                        
                        continue;
                    end
                    
                    obs_zono = local_to_zono_with_h(one_obs,agent_heading);
                    if AH.plot_flag
                        if AH.draw_subplots
                            if t_idx ==1
                                
                                plot(obs_zono,[1 2],'r');
                            end
                        end
                    end
                    %                     [A_obs, b_obs]=zono_to_Ab(obs_zono,Z);
                    %Slice, Do it now! 6 , 7 can be sliced later, taking values vx_des_soft;y_des_soft
                    %Need to get rid of the extra zonos that have negative velocity
                    obstacle = obs_zono.Z;
                    buff_obstacle_c = [obstacle(:, 1) - c];
                    buff_obstacle_G = [obstacle(:, 2:end), k_no_slc_G, buffer_dist*eye(2)]; % obstacle is "buffered" by non-k-sliceable part of FRS
                    buff_obstacle_G(:, ~any(buff_obstacle_G)) = []; % delete zero columns of G
                    buff_obstacle = [buff_obstacle_c, buff_obstacle_G];
                    [A_obs, b_obs] = polytope_PH(buff_obstacle); % turn zonotope into polytope
                    A_obs_array = [A_obs_array;A_obs];
                    b_obs_array = [b_obs_array;b_obs];
                    size_idx = size_idx + length(b_obs);
                    size_array = [size_array;size_idx];
                end
                
                
                %% Dynamics Obstacles
                %                 A_obs_dyn_array= [];
%                 if t_idx <= AH.FRS_cost_function_idx

                % get time for this zonotope
                zonoint = interval(zono_one);
                tint = zonoint(length(zonoint));
                tlb = infimum(tint);
                tub = supremum(tint);


                    for traffic_idx = 1:length(dyn_O.obs{2}) % each car
                        car = [ mean(dyn_O.obs{1}(:, (traffic_idx-1)*6+1: (traffic_idx-1)*6+4),2) ;0;dyn_O.obs{2}(traffic_idx);...
                              4.8]; % x, y, heading, velocity, length
                        % account for the distance that this car travels within our time interval
                        car(5) = car(5) + car(4)*(tub-tlb);

                        % ignore a car if it is behind the ego or 200m away ahead
                        if car(1)<=agent_state(1) || car(1)>=agent_state(1)+200
                            continue
                        end

                        obs_box = make_box([car(5);2.2],car(1:2));
                        obs_box(1,:) = obs_box(1,:) + car(4)*sign(cos(car(3)))*0.5*(tlb+tub);
                        obs_zono_new = local_to_zono_with_h(obs_box',agent_heading);
                        %  local =(world coord            )-(ego)
                        %                     new_c =  center(obs_zono) + car(1:2)-dummy_state(1:2);
                        if AH.plot_flag
                            if AH.draw_subplots
                                if mod(t_idx,ceil(10/AH.truncating_factor)) == 0
                                    %                             plot(obs_zono_new,[1 2],'r');
                                    poly_vert = polygon(obs_zono_new);
                                    %                                 if min(poly_vert(1,:))>-100
                                    AH.dynamic_obs_plot.XData = [poly_vert(1,:) nan(1,1) AH.dynamic_obs_plot.XData] ;
                                    AH.dynamic_obs_plot.YData = [poly_vert(2,:) nan(1,1) AH.dynamic_obs_plot.YData] ;
                                end
                            end
                        end

                        obstacle = obs_zono_new.Z;
                        buff_obstacle_c = [obstacle(:, 1) - c];
                        buff_obstacle_G = [obstacle(:, 2:end), k_no_slc_G, buffer_dist*eye(2)]; % obstacle is "buffered" by non-k-sliceable part of FRS
                        buff_obstacle_G(:, ~any(buff_obstacle_G)) = []; % delete zero columns of G
                        buff_obstacle = [buff_obstacle_c, buff_obstacle_G];
                        [A_obs, b_obs] = polytope_PH(buff_obstacle); % turn zonotope into polytope
                        A_obs_array = [A_obs_array;A_obs];
                        b_obs_array = [b_obs_array;b_obs];
                        size_idx = size_idx + length(b_obs);
                        size_array = [size_array;size_idx];
                        %                         end
                    end
%                 end
                if AH.draw_subplots
                    if mod(t_idx,ceil(10/AH.truncating_factor)) == 0
                        drawnow
                    end
                end
                A_con{end+1} = A_obs_array*k_slc_G; % now polytope is over coefficients of k_slc_G generators
                b_con{end+1} = b_obs_array;
                s_con{end+1} = size_array;
            end
        end

    end
end

function obj_mex = get_obs_mex(dyn_obs, bounds)
    all_pts = dyn_obs{1};
    all_vels = dyn_obs{2};
    obj_mex = [];
    n_obs = length(all_vels);
    for dyn_obs_idx = 1:n_obs
        dyn_obs_pts_start_idx = ((dyn_obs_idx-1) * 6) + 1;
        curr_pts = all_pts(:, ...
            dyn_obs_pts_start_idx:dyn_obs_pts_start_idx+3);
        deltas = max(curr_pts,[],2) - min(curr_pts,[],2);
        means = mean(curr_pts, 2);
        dyn_c_x = means(1);
        dyn_c_y = means(2);
        dyn_length = deltas(1);
        dyn_width = deltas(2);
        dyn_velocity = all_vels(dyn_obs_idx);
        dyn_heading_rad = 0;
        obj_mex(:,end+1) = [dyn_c_x; 
                            dyn_c_y; 
                            dyn_heading_rad;
                            dyn_velocity;
                            dyn_length;
                            dyn_width];
    end
    xlo = bounds(1) ; xhi = bounds(2) ;
    ylo = bounds(3) ; yhi = bounds(4) ;
    Blower = [xlo, xhi, xhi,   xlo,   xlo ; 
              ylo, ylo, ylo-1, ylo-1, ylo] ;
    Bupper = [xlo, xhi, xhi,   xlo,   xlo ; 
              yhi, yhi, yhi+1, yhi+1, yhi] ;
    dx = xhi - xlo;
    dy = yhi - ylo;
    x_c = mean([xlo, xhi]);
    y_c = mean([ylo, yhi]);
    b_thick = 0.01;
    b_half_thick = b_thick / 2.0;

    % Top
    obj_mex(:,end+1) = [x_c; yhi+b_half_thick; 0; 0; dx; b_thick];
    % Bottom
    obj_mex(:,end+1) = [x_c; ylo-b_half_thick; 0; 0; dx; b_thick];
    % Right
    obj_mex(:,end+1) = [xhi+b_half_thick; y_c; 0; 0; b_thick; dy];
    % Left
    obj_mex(:,end+1) = [xlo-b_half_thick; y_c; 0; 0; b_thick; dy];
end
