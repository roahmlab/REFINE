classdef agentHelper < handle
    %% properties
    properties
        A
        % FRS object
        zono_full
        
        %time info
        t_move
        t_failsafe_move

        T
        U
        Z
        
        T_hist
        U_hist
        Z_hist
        T_hist_failsafe
        U_hist_failsafe
        Z_hist_failsafe
        
        planned_path
        % flags to pass into children AH
        flags
        
        stuck_count = 0
        verbose = 0 ;
        plot_data
        name = 'agentHelper' ;
    end
    %% methods
    methods
        function AH = agentHelper(A,FRS_obj,varargin)
            if nargin > 0
                AH = parse_args(AH,varargin{:}) ;
                AH.A = A;
                AH.zono_full = FRS_obj;%'zono_full_7.13_1spd.mat');
            end
        end

        function [action_replaced, replace_distance, stop_sim, k, wp] = advanced_move(AH,action,world_info)
            stop_sim = 0;
            agent_info = AH.get_agent_info();
            % use heading to the waypoint as desired heading;
            wp = AH.HLP.get_waypoint(world_info,agent_info.state(:,end));
            wp1 = (wp - agent_info.state(1:3,end)) * 0.333 + agent_info.state(1:3,end);
            wp2 = (wp - agent_info.state(1:3,end)) * 0.667 + agent_info.state(1:3,end);
            quiver(wp(1),wp(2),cos(wp(3)),sin(wp(3)),0.5,'g-','LineWidth',3);
            quiver(wp1(1),wp1(2),cos(wp1(3)),sin(wp1(3)),0.5,'g-','LineWidth',3);
            quiver(wp2(1),wp2(2),cos(wp2(3)),sin(wp2(3)),0.5,'g-','LineWidth',3);
            k = AH.gen_parameter_standalone(world_info,agent_info.state(:,end),[wp,wp1,wp2]);
            replace_distance = 0;
            action_replaced = 1;
            no_replace_action = 0;
            if isempty(k)
                no_replace_action = 1;
            end

            if strcmp(AH.flags.safety_layer, 'A') && ~no_replace_action  
                AH.stuck_count = 0;
                [AH.T, AH.U, AH.Z] = AH.gen_ref(k);
                AH.Z(3,:) = AH.Z(3,:) + AH.A.state(3, end);
                AH.A.move(AH.t_move,AH.T, AH.U, AH.Z);
                idx_move = find(AH.t_move > AH.T,1,'last');
                AH.T_hist = [AH.T_hist AH.T(1:idx_move)+AH.A.time(end) - AH.t_move];
                AH.U_hist = [AH.U_hist AH.U(:,1:idx_move)];
                AH.Z_hist = [AH.Z_hist AH.Z(:,1:idx_move)];
            elseif strcmp(AH.flags.safety_layer, 'A')
                fprintf('no replacement action found\n');
                AH.stuck_count = AH.stuck_count + 1;

                    
                % match FRS time horizon and ref_traj horizon
                t_itv = interval(AH.FRS_hist{end}.vehRS_save{end}); tub = supremum(t_itv(20));
                AH.U(:,AH.T>tub) = [];
                AH.Z(:,AH.T>tub) = [];
                AH.T(AH.T>tub) = [];
                

                idx = find((AH.T-AH.t_move)>0);
                idx = idx(1)-1;
                AH.A.move(AH.T(end)-AH.T(idx),AH.T(idx:end)-AH.T(idx), AH.U(:,idx:end), AH.Z(:,idx:end));
                AH.T = AH.T(idx:end) - AH.T(idx);
                AH.U = AH.U(:,idx:end);
                AH.Z = AH.Z(:,idx:end);
                stop_sim = 1;
                fprintf('Terminating\n');

                idx_move = find(AH.t_move > AH.T,1,'last');
                AH.T_hist_failsafe = [AH.T_hist_failsafe AH.T(1:idx_move)+AH.A.time(end)-AH.t_move];
                AH.U_hist_failsafe = [AH.U_hist_failsafe AH.U(:,1:idx_move)];
                AH.Z_hist_failsafe = [AH.Z_hist_failsafe AH.Z(:,1:idx_move)];
            else
                action_replaced = 1;
                replace_distance = 0;
            end
            
        end
        
        function agent_info = get_agent_info(AH)
            agent_info = AH.A.get_agent_info();
        end
        function plot_A(AH)
            AH.A.plot()
        end

        function vdisp(AH,s,l)
            % Display a string s if the message's verbose level l is greater
            % than or equal to the planner's verbose level.
            if nargin < 3
                l = 1 ;
            end
            if AH.verbose >= l
                if ischar(s)
                    disp(['    AH: ',s])
                else
                    disp('    AH: String not provided!')
                end
            end
        end
    end
end
