classdef agentHelper < handle
    %% properties
    properties
        A
        % FRS object
        zono_full
        
        %time info
        t_move
        t_failsafe_move
        
        %saved TUZ for failsafe action when none can be found
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
            % AH = agentHelper(A,FRS_path,varargin) Construct by passing in
            % a agent and FRS file, file handled by children class 
            if nargin > 0
                AH = parse_args(AH,varargin{:}) ;
                AH.A = A;
                AH.zono_full = FRS_obj;%'zono_full_7.13_1spd.mat');
%                 frs_low = load('FRS_Rover_14-Sep-2021_low_spd.mat');
%                 AH.zono_full.M_mega = horzcat(frs_low.M_mega, AH.zono_full.M_mega);
            end
        end
        %% the only useful function in the parent class
        function [action_replaced, replace_distance, stop_sim, k, wp] = advanced_move(AH,action,world_info)
            % [action_replaced, replace_distance, stop_sim, k] = advanced_move(AH,action,world_info)
            % pass in a action from the main file with action range of [-1,1]
            % scale properly and move the agent accordingly, checks for
            % action safety if safety layer is on
%             if AH.A.state(4,end) < 0.3
%                 stop_sim = 1;
%                 action_replaced =1;replace_distance=0; k =[];
%                 return
%             end
            k = AH.convert_action_to_parameter(action,AH.flags.discrete_flag);
            stop_sim = 0;
            if strcmp(AH.flags.safety_layer, 'Z')
                [k, replace_distance, replaced_flag]= AH.adjust(k,world_info);
                if replaced_flag == 0
                    no_replace_action = 0;
                    action_replaced = 0;
                elseif replaced_flag == 1
                    no_replace_action = 0;
                    action_replaced = 1;
                elseif replaced_flag == 2
                    no_replace_action = 1;
                    action_replaced = 1;
                end
            elseif strcmp(AH.flags.safety_layer, 'N')
                replace_distance = 0;
                action_replaced = 0;
                no_replace_action = 0;
            elseif strcmp(AH.flags.safety_layer,'G')
                agent_info = AH.get_agent_info();
                wp = AH.HLP.get_waypoint(world_info,agent_info.state(:,end));
                [time_vec, delta, w_cmd,exit_status] = gen_control_inputs_gpops(AH, world_info, agent_info.state(:,end), wp);
                 AH.A.move(AH.t_move,time_vec', [delta';w_cmd'], [delta';w_cmd']);
                 if exit_status ~= 0
                    stop_sim = 1;
                 end
            elseif strcmp(AH.flags.safety_layer, 'A')
%                 try
%                     [k]= AH.gen_parameter(world_info);
%                 catch
                    agent_info = AH.get_agent_info();
%                   path = [-90 5 0;
%                      -75 5 0;
%                      -72.4 4.5 -0.2;
%                      -60 4 0;
%                      -20 4 0]';
                    % use heading to the waypoint as desired heading;
                    wp = AH.HLP.get_waypoint(world_info,agent_info.state(:,end));
                    wp1 = (wp - agent_info.state(1:3,end)) * 0.333 + agent_info.state(1:3,end);
                    wp2 = (wp - agent_info.state(1:3,end)) * 0.667 + agent_info.state(1:3,end);
%                     [wp,wp1,wp2] = get_waypoint_from_path_var_lookahead(agent_info.state(:,end), AH.planned_path, 4);
                    quiver(wp(1),wp(2),cos(wp(3)),sin(wp(3)),0.5,'g-','LineWidth',3);
                    quiver(wp1(1),wp1(2),cos(wp1(3)),sin(wp1(3)),0.5,'g-','LineWidth',3);
                    quiver(wp2(1),wp2(2),cos(wp2(3)),sin(wp2(3)),0.5,'g-','LineWidth',3);
%                     wp(3) = desired_heading ;
%                     quiver(wp(1),wp(2),cos(wp(3)),sin(wp(3)),'g');
                    
                    k = AH.gen_parameter_standalone(world_info,agent_info.state(:,end),[wp,wp1,wp2]);
                    
%                 end
                replace_distance = 0;
                action_replaced = 1;
                no_replace_action = 0;
                if isempty(k)
                    no_replace_action = 1;
                end
                
            end
            if strcmp(AH.flags.safety_layer, 'A') && ~no_replace_action  
                AH.stuck_count = 0;
                [AH.T, AH.U, AH.Z] = AH.gen_ref(k);
                AH.Z(3,:) = AH.Z(3,:) + AH.A.state(3, end);
                
                
                if AH.plot_flag % this is for highway only, may have bug for drone
%                     h1 = figure(1); 
%                     AH.plot();
%                     AH.A.plot();
%                     state=AH.A.state(:,end);
% %                     text(state(1)+20,2,[num2str(state(4)) 'm/s'],'Color','red','FontSize',15)
%                     figure(9);
%                     set(gca,'FontSize',15)
%                     axl=subplot(1,2,1);
%                     axis equal
%                     set(gca,'FontSize',15)
%                     copyobj(h1.Children.Children,axl)
%                     axl.XLim = h1.Children.XLim;axl.YLim = h1.Children.YLim;
%                     if axl.XLim(1)+15 > axl.XLim(2)-30
%                         axl.XLim = [axl.XLim(1) axl.XLim(2)];
%                     else
%                         axl.XLim = [axl.XLim(1)+15 axl.XLim(2)-30];
%                     end
%                         
%                     xlabel('x [m]');
%                     ylabel('y [m]');
                    
                end
                AH.A.move(AH.t_move,AH.T, AH.U, AH.Z);
                idx_move = find(AH.t_move > AH.T,1,'last');
                AH.T_hist = [AH.T_hist AH.T(1:idx_move)+AH.A.time(end) - AH.t_move];
                AH.U_hist = [AH.U_hist AH.U(:,1:idx_move)];
                AH.Z_hist = [AH.Z_hist AH.Z(:,1:idx_move)];
%                 xlim([AH.A.state(1,end)-18, AH.A.state(1,end)+10])
            elseif strcmp(AH.flags.safety_layer, 'A')
                fprintf('no replacement action found\n');
                AH.stuck_count = AH.stuck_count + 1;
%                 if AH.T(end)-AH.t_move > AH.t_failsafe_move
                    
                    % JL add to match FRS time horizon and ref_traj horizon
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
%                 elseif AH.stuck_count < 2
%                     AH.T = [0 AH.t_failsafe_move];
% %                     AH.U = repmat(AH.U(:,end),1,2);
%                     U_ref = AH.U(:,end)*0;
%                     AH.U = repmat(U_ref,1,2);
%                     
%                     AH.Z = repmat(AH.Z(:,end),1,2);
%                     AH.A.move(AH.t_failsafe_move,AH.T, AH.U, AH.Z);
%                 else
%                     stop_sim = 1;
%                     fprintf('Terminating\n');
%                 end
                idx_move = find(AH.t_move > AH.T,1,'last');
                AH.T_hist_failsafe = [AH.T_hist_failsafe AH.T(1:idx_move)+AH.A.time(end)-AH.t_move];
                AH.U_hist_failsafe = [AH.U_hist_failsafe AH.U(:,1:idx_move)];
                AH.Z_hist_failsafe = [AH.Z_hist_failsafe AH.Z(:,1:idx_move)];
            else
                action_replaced = 1;
                replace_distance = 0;
            end
            
        end
        %% Helper and unimplemented functions
        
        function agent_info = get_agent_info(AH)
            agent_info = AH.A.get_agent_info();
        end
        function reset(AH,flags,world_start)
            if nargin < 3
                AH.A.reset();
            else
                AH.A.reset(world_start);
            end
            AH.flags = flags;
            error('agent helper reset not implemented')
        end
        function K = convert_action_to_parameter(AH,action)
            error('convert_action_to_parameter not implemented')
        end
        function [K, replace_distance, replaced_flag] = adjust(AH,K,flags)
            error('adjust not implemented')
        end
        function [T, U, Z] = gen_ref(AH,K)
            error('gen_ref not implemented')
        end
        function [k] = gen_parameter(AH,world_info);
            error('gen_parameter not implemented')
        end
        function  plot(AH)
            error('plot not implemented')
        end
        function plot_adjust(AH)
            error('plot_adjust not implemented')
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
