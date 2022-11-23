classdef JL_highway_HLP_old < high_level_planner
    properties
        mode = 2; % mode 1 avoid min, mode 2 chase max, other for fixed point
        desired_y = 6;
        T = 3;
        lookahead = 60;
    end
methods
    function HLP = JL_highway_HLP_old(varargin)
        HLP@high_level_planner(varargin{:}) ;
    end
    function reset(HLP)
        HLP.waypoints = [];
        HLP.current_waypoint_index = 1;
    end
    
    function [waypoint,desired_lane] = get_waypoint(HLP,world_info,agent_state, O_dyn)
        % d is a 3x1 matrix that holds the front car info.
        if nargin <4 % nargin = 3 if sos is not in used
            O_dyn = world_info.dyn_obstacles{1};
            %reconstruct cars;
            car_info = zeros(2,0);
            for i = 1:size(O_dyn,2)/6
                idx = [(i-1)*6+1:i*6-2];
                car_info = [car_info [min(O_dyn(1,idx));mean(O_dyn(2,idx));world_info.dyn_obstacles{2}(i)]]; % x,y,spd
            end
        else % sos, also inputs are switched in UMplanner...
            temp = agent_state;
            agent_state = world_info;
            world_info = temp;
            obs = O_dyn;
            car_info = zeros(2,0);
            for i = 1:size(obs,2)
                car_info = [car_info [obs(1,i)-2.4; obs(2,i); obs(4,i)]];
            end
        end
        
        car_info = [car_info [99999;0;0] [99999;3.7;0] [99999;3.7*2;0]];
        car_relative_dist = car_info(1,:) - agent_state(1); 
        car_info = [car_relative_dist(:,car_relative_dist>0); car_info(2:3,car_relative_dist>0)];

        car_info(1,:) = car_info(1,:)+6*car_info(3,:); % JL: consider future position
        
        car_info_lane_1_logi = car_info(2,:) <= 1;
        car_info_lane_2_logi = car_info(2,:) <= 6 & car_info(2,:) >= 1;
        car_info_lane_3_logi = car_info(2,:) >= 6;
        [lane_1_score]=min(car_info(1,car_info_lane_1_logi));
        [lane_2_score]=min(car_info(1,car_info_lane_2_logi));
        [lane_3_score]=min(car_info(1,car_info_lane_3_logi));
        lane_scores = [lane_1_score lane_2_score lane_3_score];
        [~,desired_lane]=max(lane_scores);
        ego_lane = abs(agent_state(2)-[0,3.7,7.4]) < 2.5;
        if any(lane_scores(ego_lane) < 160) && (agent_state(1)>100)
            lookahead_d = 42; %50; %30;
        else
            lookahead_d = HLP.lookahead; %90; %50;
        end

        waypoint = [agent_state(1)+lookahead_d; 3.7*(desired_lane-1); 0];
        plot(waypoint(1), waypoint(2),'k*')

        % update current waypoints
        HLP.current_waypoint = waypoint ;
        HLP.waypoints = [HLP.waypoints, waypoint] ;
        HLP.current_waypoint_index = HLP.current_waypoint_index + 1 ;
    end
end
end