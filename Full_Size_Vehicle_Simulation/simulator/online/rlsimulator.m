classdef rlsimulator < handle
    properties
        safety_layer = 'A'; % automatic waypoint 
        discrete_flag = 0;
        replace_action = 0; % 0 for original rl lib, 1 for replace with new action, 2 for with punishing reward b/w replaced and original
        plot_sim_flag = 1; %plot simulation figure 1
        plot_AH_flag;
        plot_adjust_flag = 0;% plot replacement process figure 2
        eval = 0; % used to set random seed, using the same seed generate same random environment
        AH
        W
        
        fig_num = 1
        epscur = 1;
        time_vec=[];
        wp_hist = [];

        save_result = false;
    end

    methods
        %% constructor
        function S = rlsimulator(AH,W,varargin)
            S = parse_args(S,varargin{:}) ;
            S.AH = AH;
            S.W  = W;
            figure(1);
        end
        
        %% step
        function [Observation,Reward,IsDone,LoggedSignals,varargout] = step(S,action)
            %step in time and check for done flags
            tic
            agent_info = S.AH.get_agent_info();
            world_info = S.W.get_world_info(agent_info);%where obstacles are
            % move 
            [action_replaced, replace_distance, stuck, k, wp] = S.AH.advanced_move(action,world_info);
            S.wp_hist{end+1} = wp;
            agent_info = S.AH.get_agent_info() ;
            agent_info.replace_distance = replace_distance;
            Observation =[]; %S.W.get_ob(agent_info);
            Reward = 0;%S.W.getRew(agent_info,Observation);
            S.plot();% this updates W.O_seen, which collision check needs
            collision = S.W.collision_check(agent_info);
            if collision
                S.plot();
                warning("A collision Happened!");
            end
            goal_check = S.W.goal_check(agent_info);
            IsDone = S.determine_isDone_flag(collision,action_replaced,stuck,goal_check);
            
            if S.eval &&( IsDone == 1 || IsDone == 3 ||IsDone == 4 || IsDone == 5) && S.save_result
                Filename = sprintf('sim_summary_IsDone_%s-%s.mat', [num2str(IsDone) strcat(num2str(S.epscur),"_",datestr(now,'HH-MM-SS.FFF'))]);
                ref_Z = S.AH.ref_Z;
                proposed_ref_Z = S.AH.proposed_ref_Z;
                T= S.AH.T;
                t_real_start_arr = S.AH.t_real_start;
                t_proposed_start_arr = S.AH.t_proposed_start;
                t_move =  S.AH.t_move;
                plotting_param = S.AH.FRS_plotting_param;
                envCars = S.W.envCars;
                hist_info.K_hist = S.AH.K_hist;
                hist_info.FRS_hist = S.AH.FRS_hist;
                hist_info.mirror_hist = S.AH.mirror_hist;
                hist_info.type_manu_hist = S.AH.type_manu_hist;
                hist_info.state_hist = S.AH.state_hist;
                hist_info.time_hist = S.AH.time_hist;
                hist_info.wp_hist = S.wp_hist;
                hist_info.solve_time_hist = S.AH.solve_time_hist;                
                save(Filename,'hist_info','agent_info','world_info','ref_Z','proposed_ref_Z','plotting_param','T','t_move','t_real_start_arr','t_proposed_start_arr','envCars')
            end
            
            drawnow;
            LoggedSignals = struct;
            
            % send output action if nargout > 4
            if nargout > 4
                varargout = {action_replaced,k} ;
            end
            t_step = toc;
            S.time_vec = [S.time_vec t_step];
        end
        
        %% reset
        function [iniOb, LoggedSignals] = reset(S)
            LoggedSignals = struct;
            flags = struct;
            flags.discrete_flag = S.discrete_flag;
            flags.replace_action = S.replace_action;
            flags.safety_layer = S.safety_layer;
            if S.eval
                S.W.setup(S.epscur);
                S.AH.reset(flags,S.epscur);
            else
                S.W.setup();
                S.AH.reset(flags);
            end
            iniOb =[];
            S.plot();
        end
        
        %% helper functions
        function plot(S)
            if S.plot_sim_flag
                figure(S.fig_num) ;
                cla ; hold on ; axis equal ;
                S.W.plot(S.AH.A.get_agent_info);
                
                xlabel('x [m]');
                ylabel('y [m]');
                set(gca,'FontSize',15)
                if  S.plot_AH_flag
                    S.AH.plot()
                    if S.plot_adjust_flag
                        S.AH.plot_adjust()
                    end
                end
                S.AH.plot_A();
                S.AH.HLP.plot();
                xlim([S.AH.A.state(1,end)-10 S.AH.A.state(1,end)+100]);
                
            end

            
        end
        
        function [IsDone] = determine_isDone_flag(S,collision,action_replaced,stuck,goal_check)
            if collision && action_replaced
                IsDone = 3;
            elseif collision
                IsDone = 1;
            elseif goal_check
                IsDone = 5;
            elseif stuck
                IsDone = 4;
            elseif action_replaced
                IsDone = 2;
            else
                IsDone = 0;
            end
        end
    end
end
