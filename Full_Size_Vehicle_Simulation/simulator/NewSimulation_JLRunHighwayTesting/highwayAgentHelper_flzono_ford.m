classdef highwayAgentHelper_flzono_ford < agentHelper
    %% properties
    properties
        % hard reference bounds for parameters
        upper_bd = 10;% y
        lower_bd = 2;% y
        spdlb = 5;% vx
        spdub = 30;%vx
        
        kAug = 1;
        kAyg = 0.1;
        kAygfine = 0.05;
        % epsilon for boundary check
        eps = 0.001;
        
        HLP;
        % reference for a single sampling time, needs this so there is a
        % old reference when a new reference comes in
        vx_des = 5; % default to 1m/s
        y_des;
        
        % from FRS, to determine initial condition and desired condition
        h_array
        y_array
        v_array
        del_array
        
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
        
        zono_spd; %for other t0 zonotopes
        
        prev_action = -1;
        cur_t0_idx = 1;
        Ay_idx = 0;
        
        t_plan=0.75;% don't change this, crucial in determining reference start t0
        t_timeout = 1000; %0.75; % JL change this
        
        fminconopt = [];
        cont_opt = 1;
        constrain_force = 0;
        
        plot_ax;
        
        S %put pointer of S here so can do more things
        
        master_plot_handle_frs_2; %figure 1 FRS;
        master_plot_handle_frs_1; %figure 1 FRS;
        master_plot_handle_frs_0; %figure 1 FRS;
        FRS_helper_handle = struct;
        
        dynamic_obs_plot; %figure 2 dynamics stuff;
        
%         tmiddle_label
        truncating_factor;
        zono_dt ;
        
        FRS_cost_function_idx;
               
    end
    %% methods
    methods
        function AH = highwayAgentHelper_flzono_ford(A,FRS_obj,HLP,varargin)
            AH@agentHelper(A,FRS_obj,varargin{:});
            AH.HLP = HLP;
            %             AH.y_des = AH.A.state(2,end);
            %             AH.h_array = AH.zono_full.h_range; % array of range of acceptable parameter values
            %             AH.y_array = AH.zono_full.y_range;
            %             %             AH.v_array = AH.zono_full.v_range;
            AH.v_array = 5:2:27; % change this to accomodate blacklisted velocities.
            % have to start at 5, edit examine_FRS if you don't want to
            % start at 5
            
            %             AH.zono_full.v_range = AH.zono_full.v_range;
            %             AH.del_array = AH.zono_full.del_range;
            %             AH = parse_args(AH,varargin{:}) ;
            %             AH.A = A;
            %             AH.zono_full = load(FRS_path);%'zono_full_7.13_1spd.mat');
            
            o = optimoptions('fmincon') ;
            o.OptimalityTolerance = 1e-3 ;
            o.MaxIterations = 100000 ;
            o.MaxFunctionEvaluations = 10000 ;
            o.SpecifyConstraintGradient = true ;
            o.SpecifyObjectiveGradient = false;
            o.Display = 'off';
            o.CheckGradients = false;
            o.UseParallel = false;
            %             o.Algorithm = 'trust-region-reflective';
            AH.fminconopt = o;
            AH.plot_ax = figure(3);
            subplot(3,1,1);cla; hold on; axis equal;
            subplot(3,1,2);cla; hold on; axis equal;
            subplot(3,1,3);cla; hold on; axis equal;
            
%             figure(1);subplot(4,4,[3,7,11,15])
%             For ford only
            AH.master_plot_handle_frs_0 = plot([0],[0],'-','Color',[242 255 151]/255,'Parent',AH.A.master_ax);%unsliced %not showing this, too much clutter
            AH.master_plot_handle_frs_1 = plot([0],[0],'-y','Color',[255 178 102]/255,'Parent',AH.A.master_ax);%sliced with ini
            AH.master_plot_handle_frs_2 = plot([0],[0],'-g','Parent',AH.A.master_ax);%sliced with ini and des
            
            AH.truncating_factor = 10;
            AH.zono_dt = 0.02*AH.truncating_factor;%decides how fast dynamic obstacles move
            AH.FRS_cost_function_idx = 150/AH.truncating_factor;% decide which one to use for rough opt
        end
        
        %% methods called in parent class
        
        function [K,tout] = gen_parameter(AH,world_info, agent_state,x_des,traffic_data)
            % takes a user input parameter and obstacles in world info,
            % find a nearest action that is safe.(may be itself), also
            % return the distance in parameter space and return a replace
            % flag based on if a proper replace action is found
            %             AH.cur_t0_idx = 1; AH.prev_action = -1;
%             if AH.dont_plan 
%                 K = [];
%                 return
%             end
            
            %%DEBUG!!!!! DELETE ABOVE LATER
%             AH.master_plot_handle.XData = [];
%             AH.master_plot_handle.YData = [];
            if ~exist('traffic_data','var')
                % third parameter does not exist, so default it to something
                traffic_data = [];
            end
            
            if (AH.cur_t0_idx > 4 && AH.prev_action == 2) || (AH.cur_t0_idx > 8 && AH.prev_action == 3)
                AH.prev_action = -1;
                AH.cur_t0_idx = 1;
            end
            
            if ~exist('agent_state','var')
                agent_info = AH.get_agent_info();
                agent_state = agent_info.state(:,end);
            end
            
            O_all = world_info.obstacles;
            
            if isa(AH, 'highwaySOSAgentHelper')
                AH.cur_t0_idx = 1; AH.prev_action = -1;
                if agent_state(4) < 6
                    pause(0.3);
                    if agent_state(4) > 2
                        K = [agent_state(4)-4; 0;AH.cur_t0_idx ;AH.prev_action];
                    else
                        K = [0; 0;AH.cur_t0_idx ;AH.prev_action];
                    end
                    tout = NaN;
                    return
                end
            end
            
            %             [~,y_des_idx] =min (abs(AH.y_array - (K_user(2))));
            
            
            %             [~,h_ini_idx] = min(abs(AH.h_array - agent_state(3)));
            %             [~,vd_idx] = min(abs(K_user(1) - AH.v_array));
            %             [~,del_idx] = min(abs(AH.del_array - agent_state(5)));
            % find the index correesponding to the initial condition (v_ini, h_ini, delta_ini)
            % paramter range and desired parameter range(y_desired, and vd).
            
            % inefficient to generate bounds obstacle everytime
            bounds = world_info.bounds;
            if size(bounds,1) == 1
                xlo = bounds(1) ; xhi = bounds(2) ;
                ylo = bounds(3) ; yhi = bounds(4) ;
                
                Blower = [xlo, xhi, xhi, xlo, xlo ; ylo, ylo, ylo-1, ylo-1, ylo] ;
                Bupper = [xlo, xhi, xhi, xlo, xlo ; yhi, yhi, yhi+1, yhi+1, yhi] ;
                B = [Blower, nan(2,1), Bupper] ;% make top and bottom bounds into obstacles
            else
                B = world_info.bounds; %accomdate ford stuff
            end
            
            O = [O_all, B] ;
            
            %find the action to replace starting from current zonotope
            %index
            %% rough TRajecotry Optimization
            load_const
            if ~exist('x_des','var')
                d = zeros(3,1);
                [d(3),~,~,~] = AH.S.W.getCar_abs(2,agent_info);%top lane
                [d(2),~,~,~] = AH.S.W.getCar_abs(4,agent_info);% middle lane
                [d(1),~,~,~] = AH.S.W.getCar_abs(6,agent_info);%bottom lane
                x_des = AH.HLP.get_waypoint(agent_info,world_info,d) ;
            end
            % local coordinate desired
            %find all options
            if ~isa(AH,'highwaySOSAgentHelper') && agent_state(4) < 6
                K = []; % this is the max amount of decel accpeted in problem formulation
                tout = NaN;
                return
            end             %5:2:27
            [~,idxu0] = min(abs(AH.v_array - agent_state(4)));
            
            M = AH.zono_full.M_mega{idxu0};
            tbAu = M('Autb');                                          %Ay x y h v v r r
            tbdir = M('dirtb'); tbdir = repmat(tbdir,[1,2,1]);
            tblan = M('lantb'); tblan = repmat(tblan,[1,2,1]);
            if size(tbdir,1) > 4
                tbdir(logical([1 0 1 1 1 1 1 1]),5:end,:) = -tbdir(logical([1 0 1 1 1 1 1 1]),5:end,:);
            else
                tbdir(logical([1 0 1 1 ]),5:end,:) = -tbdir(logical([1 0 1 1 ]),5:end,:);
            end
            if size(tblan,1) > 4
                tblan(logical([1 0 1 1 1 1 1 1]),5:end,:) = -tblan(logical([1 0 1 1 1 1 1 1]),5:end,:);
            else
                tblan(logical([1 0 1 1]),5:end,:) = -tblan(logical([1 0 1 1]),5:end,:);
            end
            all_tb = {tbAu, tbdir, tblan};
            
            K= [];type_manu_all = ["Au","dir","lan"];
            if AH.plot_flag
                figure(2);
                if AH.draw_subplots
                    subplot(3,1,2);cla;hold on;axis equal;xlim([-20 200])
                    text(150,10,"Previous action="+num2str(AH.prev_action),'FontSize',15);
                    text(150,5,"u="+num2str(agent_state(4)),'FontSize',15);
                    text(150,0,"t0idx="+num2str(AH.cur_t0_idx),'FontSize',15);
                    subplot(3,1,3);cla;hold on;axis equal; xlim([-20 200])
                    text(150,10,"Previous action="+num2str(AH.prev_action),'FontSize',15);
                    text(150,5,"u="+num2str(agent_state(4)),'FontSize',15);
                    text(150,0,"t0idx="+num2str(AH.cur_t0_idx),'FontSize',15);
                end
            end
            tsolvetic = tic;
            if AH.prev_action ~= -1 %prev action = 2 3
                type_text = type_manu_all(AH.prev_action);
                FRS = M(type_text);
                K = AH.highway_sampling_opt( x_des, FRS,all_tb{AH.prev_action},AH.Ay_idx,O, agent_state, AH.prev_action,AH.cur_t0_idx,traffic_data);
                if isempty(K)
                    AH.prev_action = -1;
                    warning('Continuing previous action failed')
                    %                     AH.cur_t0_idx = 1;
                    %                     AH.Ay_idx = 0;
                else
                    K = [round(agent_state(4)); K;AH.cur_t0_idx ;AH.prev_action];
                    AH.cur_t0_idx = AH.cur_t0_idx + 1;
                end
                %                         AH.plot_zono_collide_sliced(FRS_plot,mirror_flag,agent_state,K)
                %                         FRS_plot = FRS{idx_arr(type_manu)};
                %                         AH.plot_zono_collide_sliced(FRS_plot,mirror_flag,agent_state,K)
            end
            if AH.prev_action == -1
                AH.cur_t0_idx = 1; % table here has extra cols for mirroring
                if isa(AH, 'highwaySOSAgentHelper')
                    dAu = 1000;
                else
                    [dAu] = rough_manu_cost_estimate(tbAu, agent_state,x_des,AH.cur_t0_idx);
                end
                [ddir] = rough_manu_cost_estimate(tbdir,agent_state,x_des,AH.cur_t0_idx);
                [dlan] = rough_manu_cost_estimate(tblan,agent_state,x_des,AH.cur_t0_idx);
%                 %                 ddir= 1000;% here we can toggle what actions we want
%                 dlan = 1000;
%                 dAu = 1000;
%                 ddir= 1000;
                all_dist = {dAu, ddir, dlan};
                

                % JL compute all candidate
                %%% spd change
                all_cost_val = [];
                all_K = [];
                all_idx = [];
                all_save_pkg = [];
%                 type_manu = 1;
%                 for idx = 1:length(dAu)
%                     if dAu(idx)>=1000
%                         continue
%                     end
%                     type_text = type_manu_all(type_manu);
%                     FRS = M(type_text);
%                     [K,cost_val,save_pkg] = AH.highway_sampling_opt( x_des, FRS, all_tb{type_manu}, idx, O, agent_state, type_manu, AH.cur_t0_idx, traffic_data);
%                     if ~isempty(K)
%                         all_K = [all_K, [K; 0;1;type_manu]];
%                         all_cost_val = [all_cost_val,cost_val];
%                         all_idx = [all_idx,idx];
%                         all_save_pkg = [all_save_pkg, save_pkg];
%                     end
%                 end

                if abs(x_des(2) - agent_state(2))>0.8
                    %%% dir change
                    type_manu = 2;
                    for idx = 1:length(ddir)
                        if ddir(idx)>=1000
                            continue
                        end
                        type_text = type_manu_all(type_manu);
                        FRS = M(type_text);
                        [K,cost_val,save_pkg] = AH.highway_sampling_opt( x_des, FRS, all_tb{type_manu}, idx, O, agent_state, type_manu, AH.cur_t0_idx, traffic_data);
                        if ~isempty(K)
                            all_K = [all_K, [agent_state(4);K;AH.cur_t0_idx;type_manu]];
                            all_cost_val = [all_cost_val,cost_val];
                            all_idx = [all_idx,idx];
                            all_save_pkg = [all_save_pkg, save_pkg];
                        end
                    end
    
                    %%% lan change
                    type_manu = 3;
                    for idx = 1:length(dlan)
                        if dlan(idx)>=1000
                            continue
                        end
                        type_text = type_manu_all(type_manu);
                        FRS = M(type_text);
                        [K,cost_val,save_pkg] = AH.highway_sampling_opt( x_des, FRS, all_tb{type_manu}, idx, O, agent_state, type_manu, AH.cur_t0_idx, traffic_data);
                        if ~isempty(K)
                            all_K = [all_K, [agent_state(4);K;AH.cur_t0_idx;type_manu]];
                            all_cost_val = [all_cost_val,cost_val];
                            all_idx = [all_idx,idx];
                            all_save_pkg = [all_save_pkg, save_pkg];
                        end
                    end
                end

                %%% pick the best K
                if ~isempty(all_cost_val)
                    all_cost_val = [1,2]*all_cost_val + 7*(1- (all_K(4,:)==3)); % JL prefer to choose lane change
                    [~,idx] = min(all_cost_val);
                    K = all_K(:,idx);
                    if K(4) == 1 % type_manu = 1
                        AH.prev_action = -1;
                        AH.cur_t0_idx = 1;
                    else
                        AH.prev_action = all_K(4,idx);
                        AH.cur_t0_idx = 2;
                        AH.Ay_idx = all_idx(idx);
                    end

                    %draw
                    figure(1);
                    save_pkg = all_save_pkg(idx);
                    AH.plot_selected_parameter_FRS(save_pkg.K,save_pkg.type_manu,save_pkg.FRS,save_pkg.mirror_flag,save_pkg.agent_state,save_pkg.multiplier);
%                     AH.waypt_hist = [AH.waypt_hist save_pkg.waypt];
%                     AH.K_hist = [AH.K_hist save_pkg.K];
%                     AH.FRS_hist{end+1} = save_pkg.FRS;
%                     AH.mirror_hist = [AH.mirror_hist save_pkg.mirror_flag];
%                     AH.type_manu_hist = [AH.type_manu_hist save_pkg.type_manu];
%                     AH.state_hist = [AH.state_hist save_pkg.agent_state];
%                     AH.time_hist = [AH.time_hist save_pkg.time];
                    drawnow
                    pause(0.1);

                end               
            end
            K
            tout = toc(tsolvetic);
            if ~isempty(K)% not empty so keep current plan
%                 AH.master_plot_handle_frs_0.XData =
%                 AH.FRS_helper_handle.XData{1};%this is a little too
%                 cluttered, so don't plot this
%                 AH.master_plot_handle_frs_0.YData = AH.FRS_helper_handle.YData{1};
                AH.master_plot_handle_frs_1.XData = AH.FRS_helper_handle.XData{2};
                AH.master_plot_handle_frs_1.YData = AH.FRS_helper_handle.YData{2};
                AH.master_plot_handle_frs_2.XData = AH.FRS_helper_handle.XData{3};
                AH.master_plot_handle_frs_2.YData = AH.FRS_helper_handle.YData{3};
            end
            if isa(AH, 'highwaySOSAgentHelper') && isempty(K)
                if agent_state(4) > 2
                    K = [agent_state(4)-4; 0;AH.cur_t0_idx ;AH.prev_action];
                else
                    K = [0; 0;AH.cur_t0_idx ;AH.prev_action];
                end
            end
            
            drawnow;
            if AH.pause_flag
                pause(0.5);
            end
            

            
            
            
        end
        function [K,cost_val, save_pkg]= highway_sampling_opt(AH, x_des, FRS,tbdir,diridx_with_mirror,O, agent_state, type_manu,t0_idx, traffic_data)
            %x_des, FRS,all_tb{type_manu},idx_arr(type_manu),O, agent_state, type_manu,t0_idx
            K =[]; cost_val = inf;
            if diridx_with_mirror >4 && (type_manu == 2 || type_manu ==  3 )
                mirror_flag = 1;
                diridx = diridx_with_mirror - 4;
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
                if agent_state(5) < tbdir(5,diridx,t0_idx) || agent_state(5) > tbdir(6,diridx,t0_idx)
                    warning('v0 out of bound')
                    return
                end
                if agent_state(6) < tbdir(7,diridx,t0_idx) || agent_state(6) > tbdir(8,diridx,t0_idx)
                    warning('r0 out of bound')
                    return
                end
            end
            
            
            x_des = world_to_local_mirror(agent_state,x_des, mirror_flag);
            %             x_des(2) = multiplier * x_des(2);%is this correct?
            %Above line really not sure
            dummy_state = agent_state;
            if ~isa(AH, 'highwaySOSAgentHelper')
                dummy_state(3) =  0;%for zono, rotation is done when geerating obs
                %for sos, do rotation here;
            end
            O = world_to_local_mirror(dummy_state,O,mirror_flag );
            if size(traffic_data, 1) >= 2
                traffic_data(1:2,:) = world_to_local_mirror(dummy_state,traffic_data(1:2,:), mirror_flag);
            end

            
            if size(FRS,1) == 1
                FRS = FRS';
            end
            FRS = FRS{diridx,t0_idx};
            if AH.cont_opt                                                                    %center value
                [K, cost_val] = AH.find_optimal_cont(O,agent_state,FRS,mirror_flag,type_manu,tbdir(1,diridx,t0_idx),abs((tbdir(1,2,t0_idx)-tbdir(1,1,t0_idx))/2),x_des, traffic_data);
                K = multiplier * K;
            else
                %below all in postive y change frame
                [safeK] = AH.find_safe_actionset(O,agent_state,FRS,mirror_flag,type_manu);
                
                if ~isempty(safeK)
                    cost_arr = zeros(length(safeK),1);
                    cost_y = cost_arr;
                    gensize = abs((tbdir(1,2,t0_idx)-tbdir(1,1,t0_idx))/2);
                    safeK =(safeK.*gensize+tbdir(1,diridx,t0_idx));
                    if type_manu == 1
                        slice_idx = 11;
                    else
                        slice_idx = 12;
                    end
                    for i = 1:length(safeK)
                        c = center(zonotope_slice(FRS.vehRS_save{AH.FRS_cost_function_idx}, [7;8;9;slice_idx], [agent_state(4);agent_state(5);agent_state(6);safeK(i)]));
                        cost_arr(i) = highway_cost_fun(c(1:3),x_des);
                        cost_y(i) = abs(c(2) - x_des(2,1));
                    end
                    [cost_val,Kidx] = min(cost_arr);
                    cost_val = [cost_val; cost_y(Kidx)];
                    K = safeK(Kidx);
                    %now go back to unmirrored frame
                    K = multiplier *K;
                else
                    K = [];
                end
            end
%             if AH.plot_flag
%                 AH.plot_selected_parameter_FRS(K,type_manu,FRS,mirror_flag,agent_state,multiplier)
%             end
            save_pkg = [];
            if ~isempty(K)
                save_pkg.K = K;
                save_pkg.type_manu = type_manu;
                save_pkg.FRS = FRS;
                save_pkg.mirror_flag = mirror_flag;
                save_pkg.agent_state = agent_state;
                save_pkg.multiplier = multiplier;
                save_pkg.waypt = x_des;
            end
        end
        function plot_selected_parameter_FRS(AH,K,type_manu,FRS,mirror_flag,agent_state,multiplier)
            if ~isempty(K)
                %clear data and then plot
                AH.FRS_helper_handle.XData = cell(3,1);
                AH.FRS_helper_handle.YData = cell(3,1);
                if type_manu == 1
                    AH.plot_zono_collide_sliced(FRS,mirror_flag,agent_state,[K; 0],[0 0 1],0);
                    AH.plot_zono_collide_sliced(FRS,mirror_flag,agent_state,[K; 0],[0 0 1],1);
                    AH.plot_zono_collide_sliced(FRS,mirror_flag,agent_state,[K; 0],[0 0 1],2);
                else
                    AH.plot_zono_collide_sliced(FRS,mirror_flag,agent_state,[agent_state(4);K *multiplier],[0 1 0],0);
                    AH.plot_zono_collide_sliced(FRS,mirror_flag,agent_state,[agent_state(4);K *multiplier],[0 1 0],1);
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
                error('Timed out while evaluating constraint function!')
            end
            %we need max(Ax -b)> 0, since fmincon requires nonlin<=0
            %we specify constraint as min(b-Ax)<=0
        end
        function [avaliable_action_set, costval] = find_optimal_cont(AH,O,agent_state,FRS,mirror_flag,AuAyflag, zono_c, zono_g,x_des,traffic_data)
            %             for i = 1:length(FRS.delta_force)
            %                 FRS.delta_force{i} =  deleteAligned(FRS.delta_force{i});
            %             end
            costval = inf;
            costval_y = inf;
            start_tic = tic;
            [A_con,b_con,s_con] = AH.generate_constraints(agent_state,O,FRS.vehRS_save,mirror_flag,AuAyflag,traffic_data);
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
            lb =[zono_c - zono_g];
            ub =[zono_c + zono_g];
            
            initial_guess = (lb + ub)/2;
            
            % JL plot
            figure(10), clf
            for i = 1:length(FRS.vehRS_save)
                plot(FRS.vehRS_save{i});
                hold on
            end
            axis equal


            try
            [k,fval,exitflag,~] = fmincon(cost, initial_guess, [], [],...
                [], [], lb, ub, cons,AH.fminconopt) ;
            catch
                exitflag = -1;%timeout
                warning ('optimization times out')
            end
            
            if exitflag == 1 || exitflag == 2
                avaliable_action_set = k;
                costval = cost(k);
                c = center(zonotope_slice(FRS.vehRS_save{AH.FRS_cost_function_idx}, [7;8;9;slice_idx], [agent_state(4);agent_state(5);agent_state(6);k]));
                costval = [costval; abs(c(2) - x_des(2))];
                toc(start_tic)
            else
                avaliable_action_set = [];
            end
        end
        function [avaliable_action_set] = find_safe_actionset(AH,O,agent_state,FRS,mirror_flag,AuAyflag)
            % replace action based on obstacle check. takes intial
            % condition index and desired condition index, output the
            % replaced parameter and if it is replaced. return [[],true]
            % if no replacement is found.
            [A_con,b_con,s_con] = AH.generate_constraints(agent_state,O,FRS.vehRS_save,mirror_flag,AuAyflag);
            
            
            lambdas_arr = linspace(-0.99,0.99,35);
            avaliable_action_set=[];
            
            for lamidx = 1:length(lambdas_arr)%*length(k2_sample)
                % was using a sprial search to make algorithm faster.
                % However, the first safe index that the sprial search
                % finds is not the closest one. So instead, just sprial
                % search the whole parameter space and record the avaliable
                % ones with distance
                
                %final parameter check
                %                 if K(1) > AH.spdub || K(1) < AH.spdlb || K(2) + agent_state(2) > AH.upper_bd || K(2) + agent_state(2) < AH.lower_bd
                %                     continue;
                %                 end
                
                
                lambdas = lambdas_arr(lamidx); % given a parameter, get coefficients on k_slc_G generators
                for k = 1:length(A_con)
                    Zk_tmp = A_con{k}*lambdas - b_con{k}; % A*lambda - b <= 0 means inside unsafe set
                    safe = max(Zk_tmp(1:s_con{k}(1))) > 0;
                    for m = 1: length(s_con{k})-1
                        if ~safe
                            break %break the m loop for obstacle;
                        end
                        safe = max(Zk_tmp(s_con{k}(m)+1:s_con{k}(m+1))) > 0;
                    end
                    if ~safe
                        break % %break the k loop for time idx;
                    end
                end
                if safe
                    %                     if AH.plot_flag
                    %                         h1.XData= [h1.XData K(1)];
                    %                         h1.YData= [h1.YData K(2)];
                    %                     end
                    %save avaliable parameters in a set
                    avaliable_action_set = [avaliable_action_set; lambdas];
                    %                     if replace_idx == 0
                    %                         break;% just exit if first selected action already safe
                    %                     end
                end
                
            end
            
            
            
        end
        function [avaliable_action_set] = find_Au_replace_action(AH,O,agent_state,FRS,mirror_flag,AuAyflag)
            % replace action based on obstacle check. takes intial
            % condition index and desired condition index, output the
            % replaced parameter and if it is replaced. return [[],true]
            % if no replacement is found.
            [A_con,b_con,s_con] = AH.generate_constraints(agent_state,O,FRS.vehRS_save,mirror_flag,AuAyflag);
            
            lambdas_arr = linspace(-0.99,0.99,10);
            avaliable_action_set=[];
            
            for lamidx = 1:length(lambdas_arr)%*length(k2_sample)
                % was using a sprial search to make algorithm faster.
                % However, the first safe index that the sprial search
                % finds is not the closest one. So instead, just sprial
                % search the whole parameter space and record the avaliable
                % ones with distance
                
                %final parameter check
                %                 if K(1) > AH.spdub || K(1) < AH.spdlb || K(2) + agent_state(2) > AH.upper_bd || K(2) + agent_state(2) < AH.lower_bd
                %                     continue;
                %                 end
                
                
                lambdas = lambdas_arr(lamidx); % given a parameter, get coefficients on k_slc_G generators
                for k = 1:length(A_con)
                    Zk_tmp = A_con{k}*lambdas - b_con{k}; % A*lambda - b <= 0 means inside unsafe set
                    safe = max(Zk_tmp(1:s_con{k}(1))) > 0;
                    for m = 1: length(s_con{k})-1
                        if ~safe
                            break %break the m loop for obstacle;
                        end
                        safe = max(Zk_tmp(s_con{k}(m)+1:s_con{k}(m+1))) > 0;
                    end
                    if ~safe
                        break % %break the k loop for time idx;
                    end
                end
                if safe
                    %                     if AH.plot_flag
                    %                         h1.XData= [h1.XData K(1)];
                    %                         h1.YData= [h1.YData K(2)];
                    %                     end
                    %save avaliable parameters in a set
                    avaliable_action_set = [avaliable_action_set; lambdas];
                    %                     if replace_idx == 0
                    %                         break;% just exit if first selected action already safe
                    %                     end
                end
                
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
            
            t0 = (t0_idx-1)*AH.t_plan;
            type_manu = K(4);
            load_const
            %not symbolic mode time argument will be ignored
            %del_y,Au,u0,t,symbolic_flag, scale_Ay_flag                t0
            if type_manu == 3
                [T, U,Z] = gaussian_T_parameterized_traj_with_brake(t0,Ay,Au,u_cur,[],0,1);
            else
                [T, U,Z] = gaussian_one_hump_parameterized_traj_with_brake(t0,Ay,Au,u_cur,[],0,1);
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
            %             if ~isempty(AH.proposed_ref_Z)
            %                 plot(AH.proposed_ref_Z(end-1,:),AH.proposed_ref_Z(end,:),'k-','LineWidth',3);
            %                 plot(AH.proposed_ref_Z(end-1,:),AH.proposed_ref_Z(end,:),'Color','y','LineWidth',3,'LineStyle','--');
            %
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
        
        function plot_adjust(AH)
            error('adjust plot not implemented')
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
            if ~isempty(AH.HLP)
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
                if mirror_flag
                    figure(2);subplot(3,1,3);
                else
                    figure(2);subplot(3,1,2);
                end
                for t_idx = 1:10/AH.truncating_factor:length(FRS.vehRS_save)
                    zono_one = zonotope_slice(FRS.vehRS_save{t_idx}, [7;8;9;slice_dim], [agent_state(4);agent_state(5);agent_state(6);k_slice]);
                    h = plot(zono_one,[1,2],'Color',color);
                end
                
                figure(2); subplot(3,1,1);
            end
            
            for t_idx = 1:10/AH.truncating_factor:length(FRS.vehRS_save)
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
            
            figure(2);
        end
        
        
        function [K, replaced] = find_replace_action_global(AH,O,agent_state,K_user)
            %for each combination of v_des_index and y_des_idx, try finding
            %the replacment action. start with the current index, if cannot
            %find coninue onto near ones.
            % Interestingly, there is only one combination now. v = [1 3]
            %y = [-1 1], v =[3 5] was disabled due to a bug, but that makes
            %everything simpler so the method is more clear.but maybe not
            %to its full potential.
            for replace_idx = 1:1%2*2*length(AH.zono_full.v_range)*length(AH.zono_full.y_range)% 2 times making sure all zonotopes are looked at
                [d1,d2] = sprial_index(replace_idx); % use a spiral indexing way to check nearby zonotopes
                %                 k1idx = vd_idx + d1;
                %                 k2idx = y_des_idx + d2;
                %                 if k1idx > length(AH.zono_full.v_range)  || k1idx < 1 || k2idx < 1 ||  k2idx > length(AH.zono_full.y_range)
                %                     %outside of selectable zonotopes, skip
                %                     continue;
                %                 end
                %                 if AH.plot_flag
                %                     AH.check_zono_collide_halfspace(O,agent_state,v_ini_idx,k2idx,h_ini_idx,del_idx,k1idx);% check for full space safety, for good visualization
                %                 end
                [K, inner_replaced_flag,~] = AH.find_replace_action(O,agent_state,K_user);% find a action that works
                if ~isempty(K)
                    %P.check_zono_collide_sliced(O,agent_state,v_ini_idx,k2idx,h_ini_idx,del_idx,k1idx,K);
                    break
                end
            end
            if replace_idx == 1 && inner_replaced_flag == 0
                replaced = 0;
            else
                replaced = 1;
            end
        end
        function [A_con,b_con,s_con] = generate_other_constraints(AH, agent_state,stop_idx,FRS, AuAyflag)
            zono_peak_mid_stop  = FRS;
            n = length(zono_peak_mid_stop);
            load_const
            max_tire_f = m * 9.81 * mu_gnd;
            W_box = 2*sin(pi/8)*max_tire_f;
            L_box = max_tire_f - cos(pi/8)*max_tire_f;
            
            
            O_pts = make_box([L_box W_box],[max_tire_f-W_box/2 0])';
            
            %in2 states:Fxf Fyf delta u, v, r,u0,v0, r0,t0 ,Au,   Ay,   t
            %            1  2    3    4  5  6  7  8  9  10  11    12    13
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
            
            if AH.plot_flag
                figure(2);clf;hold on;
                
                for t_idx = 4:2:stop_idx
                    zono_one = zonotope_slice(zono_peak_mid_stop{t_idx}, [7;8;9], [agent_state(4);agent_state(5);agent_state(6)]);
                    plot(zono_one,[1,2],'Color',plotc);
                end
            end
            
            for t_idx = 4:stop_idx
                
                zono_one = zono_peak_mid_stop{t_idx};
                
                % bound the values so slicing won't have problem, max
                % values determined by simulating beforehand
                %                 y_des_soft = bound_values(AH.y_des - agent_state(2),AH.zono_full.y_range(y_des_idx)-AH.zono_full.kyg+AH.eps, AH.zono_full.y_range(y_des_idx)+AH.zono_full.kyg-AH.eps);% can't slice on edge
                %                 vx_des_soft =bound_values(AH.vx_des,AH.zono_full.v_range(vd_idx)-AH.zono_full.kvdg+AH.eps, AH.zono_full.v_range(vd_idx)+AH.zono_full.kvdg-AH.eps);
                %                 vi_soft =bound_values(agent_state(4),AH.zono_full.v_range(v_ini_idx)-AH.zono_full.kvig+AH.eps, AH.zono_full.v_range(v_ini_idx)+AH.zono_full.kvig-AH.eps);
                %                 h_soft = bound_values(agent_state(3),AH.zono_full.h_range(h_ini_idx)-AH.zono_full.h_ini_range+0.001,AH.zono_full.h_range(h_ini_idx)+AH.zono_full.h_ini_range-0.001);
                zono_one = deleteAligned(zonotope_slice(zono_one, [7;8;9], [agent_state(4);agent_state(5);agent_state(6)]));
                
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
                
                %consider each obstacle as a halfspace
                for obs_idx = 1:8
                    one_obs = O_pts;
                    
                    
                    
                    %                         if P.plot_flag
                    %                             figure(2)
                    %                             h=(O(1,(obs_idx-1)*6+1:obs_idx*6-1),O(2,(obs_idx-1)*6+1:obs_idx*6-1),'r');
                    %                              pause(0.2);
                    %                               delete(h);
                    %                         end
                    agent_heading = pi/4*(obs_idx-1);
                    obs_zono = local_to_zono_with_h(one_obs,agent_heading);
                    if AH.plot_flag
                        if t_idx ==4
                            
                            plot(obs_zono,[1 2],'r');
                        end
                        
                    end
                    obstacle = obs_zono.Z;
                    
                    %Slice, Do it now! 6 , 7 can be sliced later, taking values vx_des_soft;y_des_soft
                    %Need to get rid of the extra zonos that have negative velocity
                    
                    c = Z(obs_dim, 1);
                    G = Z(:, 2:end);
                    
                    for k_idx = 1:length(k_dim)
                        [~, k_col(k_idx)] = find(G(k_dim(k_idx), :) ~= 0); % find "k-sliceable" generators
                    end
                    k_slc_G = G(obs_dim, k_col);
                    k_no_slc_G = G(obs_dim, :);
                    k_no_slc_G(:, k_col) = [];
                    
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
                A_con{end+1} = A_obs_array*k_slc_G; % now polytope is over coefficients of k_slc_G generators
                b_con{end+1} = b_obs_array;
                s_con{end+1} = size_array;
            end
        end
        function [A_con,b_con,s_con] = generate_constraints(AH, agent_state,O_pts,FRS, mirror_flag, AuAyflag,traffic_data)
            if ~exist('mirror_flag','var')
                mirror_flag = 0;
            end
            zono_peak_mid_stop  = FRS;
            n = length(zono_peak_mid_stop);
            
            
            
            O_pts = O_pts';
            %in2 states: x, y, psi, u, v, r,u0,v0, r0,t0 ,Au,   Ay,   t
            %            %           1  2  3    4  5  6  7  8  9  10  11    12    13
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
            
            for t_idx = 1:n
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
                %consider each obstacle as a halfspace
                for obs_idx = 1:(size(O_pts,1)+1)/6
                    one_obs = O_pts((obs_idx-1)*6+1:obs_idx*6-1,:);
                    
                    
                    if max(one_obs(:,1)) < -20 || min(one_obs(:,1)) > 250
                        %                     if(max(one_obs(:,1)) - min(one_obs(:,1)) ) < 100 || max(one_obs(:,2))>0
                        
                        continue;
                    end
                    
                    agent_heading = multipler * agent_state(3);
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

                    for traffic_idx = 1:size(traffic_data, 2) % each car
                        car = traffic_data(:,traffic_idx);
                        %                         [~,idxu0] = min(abs(this.u0_arr_spd - car(3)));
                        %                         u0_spd = this.u0_arr_spd(idxu0);
                        %                         Au_zono_all = this.zono_spd.M_spd(char("u0="+num2str(u0_spd)+"_Au"));
                        %                         for action_idx = ceil(length(Au_zono_all)/2)% use middle choice right now
                        obs_box = make_box([car(4);2.2],car(1:2));
                        obs_box(1,:) = obs_box(1,:) + car(3)* (t_idx-1)*AH.zono_dt;
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
        function collision_array = check_zono_collide_halfspace(AH,O,agent_state,v_ini_idx,y_des_idx,h_ini_idx,del_idx,vd_idx)
            % same as find_replace_action execpt check the full space, for
            % plotting purposes only
            %             if AH.plot_flag
            % %                 figure(2);clf;hold on;axis equal;
            %             end
            zono_peak_peak = AH.zono_full.res{v_ini_idx ,y_des_idx,h_ini_idx,del_idx,vd_idx,1};
            zono_peak_mid = AH.zono_full.res{v_ini_idx ,y_des_idx,h_ini_idx,del_idx,vd_idx,2};
            zono_peak_stop = AH.zono_full.res{v_ini_idx ,y_des_idx,h_ini_idx,del_idx,vd_idx,3};
            error_data_cur = AH.zono_full.error_table{v_ini_idx ,y_des_idx,h_ini_idx,del_idx,vd_idx};
            %             zono_one_all = [zono_peak_peak; zono_peak_mid;zono_peak_stop];
            
            zono_peak_one = [zono_peak_peak; zono_peak_mid;zono_peak_stop];
            %             zono_stop_one = P.zono_stop.res_stop{v_ini_idx ,y_des_idx,h_ini_idx,del_idx,vd_idx};
            n = length(zono_peak_one); %+length(zono_stop_one);
            
            collision_array = zeros(n, 1);
            if n == 0
                return
            end
            
            
            %             AH.current_obstacles = O ;
            %             [O_pts] = compute_zonotope_obs(O,...
            %                 agent_state,P.buffer,P.point_spacing) ;
            dummy_state = agent_state;
            dummy_state(3) = 0; % since the zonotope here doesn't need to be rotated.
            %             [O_pts] = compute_zonotope_obs(O,...
            %                 dummy_state,P.buffer,P.point_spacing) ;
            O_pts = world_to_local(dummy_state,O);
            % filter out points that are too far away to be reached
            %     O_FRS = crop_points_outside_region(0,0,O_FRS,1) ;
            O_pts = O_pts';
            
            
            obs_dim = [1; 2]; % note that the obstacle exists in the x-y space (not theta or v)
            k_dim = [3; 6]; % note that the parameters k are in the 5th and 6th rows of the zonotopes
            buffer_dist = 0; % assume no buffer.
            %
            A_con = {};
            b_con = {};
            s_con = {};
            for t_idx = 1: n
                %                     if t_idx <= length(zono_peak_one)
                zono_one = zono_peak_one{t_idx};
                % %                     else
                %                         zono_one = zono_stop_one{t_idx- length(zono_peak_one)}{1};
                %                         color = [0 1 0];
                %                     end
                %                     zono_one = zono_one + P.footprint_buffer;
                if isempty(zono_one)
                    error("no such zonotope"+num2str(v_ini_idx)+num2str(y_des_idx)+num2str(h_ini_idx)+num2str(del_idx)+num2str(vd_idx));
                end
                y_des_soft = bound_values(AH.y_des - agent_state(2),AH.zono_full.y_range(y_des_idx)-AH.zono_full.kyg+AH.eps, AH.zono_full.y_range(y_des_idx)+AH.zono_full.kyg-AH.eps);% can't slice on edge
                vx_des_soft =bound_values(AH.vx_des,AH.zono_full.v_range(vd_idx)-AH.zono_full.kvdg+AH.eps, AH.zono_full.v_range(vd_idx)+AH.zono_full.kvdg-AH.eps);
                vi_soft =bound_values(agent_state(4),AH.zono_full.v_range(v_ini_idx)-AH.zono_full.kvig+AH.eps, AH.zono_full.v_range(v_ini_idx)+AH.zono_full.kvig-AH.eps);
                heading_soft = bound_values(agent_state(3), AH.zono_full.h_range(h_ini_idx)-AH.zono_full.h_ini_range+AH.eps, AH.zono_full.h_range(h_ini_idx)+AH.zono_full.h_ini_range-AH.eps);
                zono_one = zonotope_slice(zono_one, [4;5], [vi_soft;heading_soft]);
                
                center_bb = error_data_cur(1:2,t_idx);
                h =  error_data_cur(3,t_idx);
                gen = error_data_cur(4:5,t_idx);
                len = gen(1);
                width = gen(2);
                ego_gen = [[cos(h)*len; sin(h)*len], [sin(-h)*width; cos(-h)*width]];
                gen = zeros(7,2);gen(1:2,1:2) = ego_gen;
                err_zono  = zonotope([[center_bb(1);center_bb(2);0;0;0;0;0], gen]);
                %                         = zonotope([[cur_x_cen(i);cur_y_cen(i)],diag([cur_x_gen(i);cur_y_gen(i)])]);
                %                             err_zono_clean{i} = zonotope([[0;0],diag(ft_print_gen)])+zonotope([[cur_x_cen_clean(i);cur_y_cen_clean(i)],diag([cur_x_gen_clean(i);cur_y_gen_clean(i)])]);
                
                
                %Slice, Do it now!
                zono_one = (err_zono+zono_one);
                %                 if P.plot_flag
                %                     figure(2);
                %                     plot(zonotope_slice(zono_one, [3;6], [2;-0.4]),[1;2]);
                %                 end
                %z_with_err_plot = project(zonotope_slice(zono_one, [3;6], [vx_des_soft;y_des_soft]), [1, 2])
                %                 if P.plot_flag
                %                     p4=plotFilled(zono_one,[1,2],'r');
                %                     p4.FaceAlpha = 0.02;
                %                 end
                %                 zono_plot = zonotope_slice(zono_one, ,);
                Z = zono_one.Z;
                A_obs_array= []; b_obs_array = [];size_array=[]; size_idx = 0;
                %                  P.vx_des,P.y_des - agent_state(2),agent_state(3),agent_state(4),agent_state(5)
                for obs_idx = 1:(size(O_pts,1)+1)/6
                    one_obs = O_pts((obs_idx-1)*6+1:obs_idx*6-1,:);
                    
                    if max(one_obs(:,1)) < -20 || min(one_obs(:,1)) > 50
                        %                     if(max(one_obs(:,1)) - min(one_obs(:,1)) ) < 100 || max(one_obs(:,2))>0
                        
                        continue;
                    end
                    %                         if P.plot_flag
                    %                             figure(2)
                    %                             h=(O(1,(obs_idx-1)*6+1:obs_idx*6-1),O(2,(obs_idx-1)*6+1:obs_idx*6-1),'r');
                    %                              pause(0.2);
                    %                               delete(h);
                    %                         end
                    obs_zono = local_to_zono(one_obs);
                    obstacle = obs_zono.Z;
                    %                     if AH.plot_flag
                    %                         if t_idx == 1
                    %                             p4=plotFilled(obs_zono,[1,2],'r');
                    %                             p4.FaceAlpha = 0.05; xlim([-30 70]);
                    %                         end
                    %                     end
                    %Slice, Do it now! 6 , 7 can be sliced later, taking values vx_des_soft;y_des_soft
                    %Need to get rid of the extra zonos that have negative velocity
                    
                    c = Z(obs_dim, 1);
                    G = Z(:, 2:end);
                    
                    for k_idx = 1:length(k_dim)
                        [~, k_col(k_idx)] = find(G(k_dim(k_idx), :) ~= 0); % find "k-sliceable" generators
                    end
                    k_slc_G = G(obs_dim, k_col);
                    k_no_slc_G = G(obs_dim, :);
                    k_no_slc_G(:, k_col) = [];
                    
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
                A_con{t_idx} = A_obs_array*k_slc_G; % now polytope is over coefficients of k_slc_G generators
                b_con{t_idx} = b_obs_array;
                s_con{t_idx} = size_array;
            end
            k1c = AH.v_array(vd_idx);k1g = AH.zono_full.kvdg;
            k2c = AH.y_array(y_des_idx);k2g = AH.zono_full.kyg;
            c_k = [k1c; k2c];
            g_k = [k1g; k2g];
            k1_user= vx_des_soft;k2_user=y_des_soft;
            lambdas = ([k1_user; k2_user] - c_k)./g_k;
            
            
            lims = [k1c - k1g, k1c - k1g, k1c + k1g, k1c + k1g, k1c - k1g; k2c - k2g, k2c + k2g, k2c + k2g, k2c - k2g, k2c - k2g];
            %             plot(lims(1, :)', lims(2, :)', 'k', 'LineWidth', 2);
            
            % grid over parameter space
            k1_sample = linspace(k1c - k1g, k1c + k1g, 20);
            k2_sample = linspace(k2c - k2g, k2c + k2g, 20);
            [Xk, Yk] = meshgrid(k1_sample, k2_sample);
            Zk = inf*ones(size(Xk));
            for i = 1:length(k1_sample)
                for j = 1:length(k2_sample)
                    K = [Xk(i, j); Yk(i, j)];
                    lambdas = (K - c_k)./g_k; % given a parameter, get coefficients on k_slc_G generators
                    for k = 1:length(A_con)
                        Zk_tmp = A_con{k}*lambdas - b_con{k}; % A*lambda - b <= 0 means inside unsafe set
                        safe = max(Zk_tmp(1:s_con{k}(1))) > 0;
                        for m = 1: length(s_con{k})-1
                            if ~safe
                                break
                            end
                            
                            safe = max(Zk_tmp(s_con{k}(m)+1:s_con{k}(m+1))) > 0;
                        end
                        if safe
                            Zk(i, j) = min(Zk(i, j), 1); % take smallest max. if it's <=0, then unsafe
                        else
                            Zk(i, j) =-1;
                        end
                    end
                end
            end
            if AH.plot_flag
                %                 % plot parameters and unsafe set
                %                 figure(9); clf;
                %                 set(gcf, 'Position', [1 1 1000 300]);
                %                 subplot(1,2,2);hold on;
                %                 %                 title('', 'FontSize', 15);
                %                 xlabel('$\kappa_{\mathrm{pk}} \mathrm{[m/s]}$', 'Interpreter', 'latex', 'FontSize', 15);
                %                 ylabel('$\kappa_{\mathrm{y}} \mathrm{[m]}$', 'Interpreter', 'latex', 'FontSize', 15);
                %
                %                 contourf(Xk, Yk, -Zk, [0, 0], 'FaceColor', [255 51 51]/255); % show zero level set contours
                %
                %                 %% have user select a point, see the corresponding slice of FRS:
                %
                %                 k1_user= vx_des_soft;k2_user=y_des_soft;
                %
                %                 scatter(k1_user, k2_user,200,'y','filled','MarkerEdgeColor',[0  0  0],'LineWidth',2);
                %                 figure(2)
                %
                
                %                     delete(h);
            end
        end
    end
end



