function [K,tout] = gen_parameter_standalone(AH, bounds, obstacles, agent_state,waypts)
    % takes a user input parameter and obstacles in world info,
    % find a nearest action that is safe.(may be itself), also
    % return the distance in parameter space and return a replace
    % flag based on if a proper replace action is found
    x_des = waypts(:,1);
    traffic_data = [];
     cannot_move_lateral_speed = 0.55;
%     AH.cur_t0_idx = 1;
    if (AH.cur_t0_idx > 2 && AH.prev_action == 2) || (AH.cur_t0_idx > 3 && AH.prev_action == 3)|| AH.prev_action  == 1
        AH.prev_action = -1;
        AH.cur_t0_idx = 1;
    end
    
    cur_t0_idx_print = AH.cur_t0_idx

%     AH.prev_action = -1;
%     AH.cur_t0_idx = 1;

    if agent_state(4) > 1.34
        agent_state(4) = 1.34;
    end
        %to prevent cases where in the middle of a maneuver, speed drops
        %below 0.55, then slicing is going to have trouble. So keep it
        %above 0.55 manually
    if AH.prev_action ~= -1 && agent_state (4) < 0.55
       agent_state (4)= 0.551;
    end
        
    O_all = obstacles;

    % find the index correesponding to the initial condition (v_ini, h_ini, delta_ini)
    % paramter range and desired parameter range(y_desired, and vd).

    % inefficient to generate bounds obstacle everytime
%     if size(bounds,1) == 1
%         xlo = bounds(1) ; xhi = bounds(2) ;
%         ylo = bounds(3) ; yhi = bounds(4) ;
% 
%         Blower = [xlo, xhi, xhi, xlo, xlo ; ylo, ylo, ylo-1, ylo-1, ylo] ;
%         Bupper = [xlo, xhi, xhi, xlo, xlo ; yhi, yhi, yhi+1, yhi+1, yhi] ;
%         B = [Blower, nan(2,1), Bupper] ;% make top and bottom bounds into obstacles
%     else
%         B = world_info.bounds; %accomdate ford stuff
%     end

    O = [O_all] ;

    %find the action to replace starting from current zonotope
    %index
    %% rough TRajecotry Optimization
    load_const
    % REMOVED: ~exist x_des: generated x_des from HLP
    %find all options
    [~,idxu0] = min(abs(AH.v_array - agent_state(4)));
    
    M = AH.zono_full.M_mega{idxu0};
    tbAu = M('Autb');                                          %Ay x y h v v r r
    tbdir = M('dirtb'); tbdir = repmat(tbdir,[1,2,1]);
    tblan = M('lantb'); tblan = repmat(tblan,[1,2,1]);
    if size(tbdir,1) > 4
        tbdir(logical([1 0 1 1 1 1 1 1]),4:end,:) = -tbdir(logical([1 0 1 1 1 1 1 1]),4:end,:);
    else
        tbdir(logical([1 0 1 1 ]),4:end,:) = -tbdir(logical([1 0 1 1 ]),4:end,:);
    end
    if size(tblan,1) > 4
        tblan(logical([1 0 1 1 1 1 1 1]),4:end,:) = -tblan(logical([1 0 1 1 1 1 1 1]),4:end,:);
    else
        tblan(logical([1 0 1 1]),4:end,:) = -tblan(logical([1 0 1 1]),4:end,:);
    end
    all_tb = {tbAu, tbdir, tblan};
    
    %% debug plots for maneuvers
%     figure(1);clf;hold;
%      scatter(x_des(1),x_des(2),40,'g','Filled');    
%      tb_dir_world = local_to_world(agent_state(1:3), tbdir(2:4,:,1));
%      tb_Au_world = local_to_world(agent_state(1:3), tbAu(2:4,:,1));
%      quiver(tb_Au_world(1,:),tb_Au_world(2,:),cos(tb_Au_world(3,:)),sin(tb_Au_world(3,:)));
%      quiver(tb_dir_world(1,:),tb_dir_world(2,:),cos(tb_dir_world(3,:)),sin(tb_dir_world(3,:)));
%      quiver(agent_state(1),agent_state(2),cos(agent_state(3)),sin(agent_state(3)));
%      AH.num_times_called = AH.num_times_called+1;
%     scatter(AH.parent_pointer.x_des_stored_(1),AH.parent_pointer.x_des_stored_(2),50,[0.92 0.69 0.12],'Filled');
%     text(AH.parent_pointer.x_des_stored_(1),AH.parent_pointer.x_des_stored_(2),num2str(AH.num_times_called));
AH.num_times_called=AH.num_times_called+1;
 
    K= [];type_manu_all = ["Au","dir","lan"];
    tsolvetic = tic;
    
    if AH.prev_action ~= -1 %prev action = 2 3
        type_text = type_manu_all(AH.prev_action);
        FRS = M(type_text);

        
        K = AH.highway_sampling_opt( x_des, FRS,all_tb{AH.prev_action},AH.Ay_idx,O, agent_state, AH.prev_action,AH.cur_t0_idx,traffic_data,waypts);
        if isempty(K)
            AH.prev_action = -1;
            warning('Continuing previous action failed')
            %                     AH.cur_t0_idx = 1;
            %                     AH.Ay_idx = 0;
        else
            K = [round(agent_state(4),1); K;AH.cur_t0_idx ;AH.prev_action];
            AH.cur_t0_idx = AH.cur_t0_idx + 1;
        end
    end    
    if AH.prev_action == -1
        AH.cur_t0_idx = 1; % table here has extra cols for mirroring
        [dAu] = rough_manu_cost_estimate(tbAu, agent_state,waypts,AH.cur_t0_idx);
        [ddir] = rough_manu_cost_estimate(tbdir,agent_state,waypts,AH.cur_t0_idx);
%         [dlan] = rough_manu_cost_estimate(tblan,agent_state,waypts,AH.cur_t0_idx);
%                 %                 ddir= 1000;% here we can toggle what actions we want
      
       
       if agent_state(4) <= cannot_move_lateral_speed
         ddir = 1000;
       else
%             ddir(2) = 1000;
%             ddir(5) = 1000;%reduce number of bins
%             useful_au_idx = (idxu0-6-1):(idxu0-6+1);
%             useful_au_idx = useful_au_idx(useful_au_idx>= 1);
%             all_idx = ones(8,1);
%             all_idx(useful_au_idx) = 0;
%             dAu(logical(all_idx)) = 1000;
       end
       
        dlan = 1000;
%          dAu  = 1000;
%         if length(dAu) >1
%             v_array_original = 0.6:0.1:1.3;
%             dAu(v_array_original < cannot_move_lateral_speed +0.1) = 1000;
% %             dAu(end-1:end) = 1000; 
%         end
%                 dAu = 1000;
%                 ddir= 1000;

        all_dist = {dAu, ddir, dlan};
        while any(all_dist{1}<1000) || any(all_dist{2}<1000) || any(all_dist{3}<1000)
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
            FRS = M(type_text);                     %here is original table
            %highway_sample_opt should not handle
            K = AH.highway_sampling_opt( x_des, FRS, all_tb{type_manu}, idx_arr(type_manu), O, agent_state, type_manu, AH.cur_t0_idx, traffic_data, waypts);
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
                end                        
                break;
            end
        end
    end
    K
    tout = toc(tsolvetic);            
end