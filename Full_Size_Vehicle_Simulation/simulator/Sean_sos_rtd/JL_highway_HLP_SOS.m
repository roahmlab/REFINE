classdef JL_highway_HLP_SOS < high_level_planner
    properties
        mode = 2; % mode 1 avoid min, mode 2 chase max, other for fixed point
        desired_y = 6;
        T = 3;
    end
methods
    function HLP = JL_highway_HLP_SOS(varargin)
        HLP@high_level_planner(varargin{:}) ;
    end
    function reset(HLP)
        HLP.waypoints = [];
        HLP.current_waypoint_index = 1;
    end
    
    function [waypoint, desired_lane] = get_waypoint(HLP,agent_state,world_info,traffic_info)
        obs = traffic_info;
        car_info = zeros(2,0);
        for i = 1:size(obs,2)
            car_info = [car_info [traffic_info(1,i)-2.4; traffic_info(2,i)]];
        end
        car_info = [car_info [99999;0] [99999;3.7] [99999;3.7*2]];
        car_relative_dist = car_info(1,:) - agent_state(1);
        car_info = car_info(:,car_relative_dist>0);
        car_info_lane_1_logi = car_info(2,:) <= 1;
        car_info_lane_2_logi = car_info(2,:) <= 6 & car_info(2,:) >= 1;
        car_info_lane_3_logi = car_info(2,:) >= 6;
        [lane_1_score]=min(car_info(1,car_info_lane_1_logi));
        [lane_2_score]=min(car_info(1,car_info_lane_2_logi));
        [lane_3_score]=min(car_info(1,car_info_lane_3_logi));
        [~,desired_lane]=max([lane_1_score lane_2_score lane_3_score]);
        waypoint = [agent_state(1)+100; 3.7*(desired_lane-1); 0];

        % update current waypoints
        HLP.current_waypoint = waypoint ;
        HLP.waypoints = [HLP.waypoints, waypoint] ;
        HLP.current_waypoint_index = HLP.current_waypoint_index + 1 ;
    end
end
end