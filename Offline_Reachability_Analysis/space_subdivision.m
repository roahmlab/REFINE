% This script tries to find proper subdivision of the initial condition
% space and control parameter space for direction change and lane change.
% We aim to achieve enough lateral displacement during a direction or lane 
% change maneuver.  

clear, close all; clear
isSim = 1; % 1: full-size FWD vehicle for simulation. 0: AWD Rover for hardware experiments. 
if isSim
    full_size_const;
else
    rover_const;
end
save '../util/my_const.mat'
warning('off','ALL')


mode = 1; %mode 1 for dir change, mode 2 for lane change

x0 = 0;
y0 = 0;
h0 = 0;
vy0 = 0;
t0 = 0;
r0 = 0;
if isSim
    vx0_vec = 5.25:0.5:29.75; % partition on the initial condition space of vx
    t0_dt = 3; % because time duration of a lane change maneuver is as twice as that of a speed/direction change maneuver, t0_dt provides options to execute the lane change maneuver from the beginning and middle (not used in REFINE).
else
    vx0_vec = 0.6:0.1:2.0;
    t0_dt = 1.5;
end
vx0_gen = 0.5*(vx0_vec(2)-vx0_vec(1));

Ay_vec= zeros(length(vx0_vec),1); 

sim_end_idx = 5;
r0vy0_limit = zeros(length(vx0_vec),2);
if mode == 2
    if isSim
        y_ideal = 4; % lane change seeks to achieve 4m of lateral change in simulation.
    else
        y_ideal = 0.3;
    end
    num_Ay = 2;
elseif mode == 1
    if isSim
        y_ideal = 3; % direction change seeks to achieve 3m of lateral change in simulation.
    else
        y_ideal = 0.3;
    end
    num_Ay = 2;
end


if mode == 1
    end_idx = tpk_dir/t0_dt;
else
    end_idx = tpk/t0_dt;
end
r_value_vec = zeros(2,end_idx,num_Ay,length(vx0_vec)); %value of r at different time stage of a turn
vy_value_vec = zeros(2,end_idx,num_Ay,length(vx0_vec)); %value of vy at different time stage of a turn
for vx0idx = 1:length(vx0_vec)
    vx0 = vx0_vec(vx0idx) % initial longitudinal speed
    w0 = vx0; % initial wheel speed.
    %% Test system behavior over each maneuver via simulaion
    if isSim
        Agent = highway_cruising_10_state_agent;
    else
        Agent = highway_cruising_10_state_agent_awd;
    end

    z_0 = [x0;y0;h0;vx0;vy0;r0;w0;0;0;0] ;
    done_flag = 0;
    
    p_y_lower= 0;
    if isSim
        p_y_upper = 0.8; 
    else
        p_y_upper = 1.396;
    end

    while done_flag~= 1 
        p_y = mean([p_y_lower p_y_upper]);
        p_vx = vx0; 
        if mode == 1 
            t= [];
            [T_go,U_go,Z_go] = sin_one_hump_parameterized_traj_with_brake(t0,p_y,p_vx,vx0,t,0, 1); % T: time. U: control input. Z: state (ideal)
        else
            t= [];
            [T_go,U_go,Z_go] = gaussian_T_parameterized_traj_with_brake(t0,p_y,p_vx,vx0,t,0, 1, isSim);
        end
        Agent.reset(z_0) ;
        if mode == 1 
            Agent.move(tpk_dir,T_go,U_go,Z_go) ; 
        else
            Agent.move(tpk,T_go,U_go,Z_go) ;
        end
        
        
        X = Agent.state';
        T = Agent.time';
        delta_y = X(end, 2);

        if abs(delta_y-y_ideal) <0.01
            done_flag = 1; 

            Ay_vec(vx0idx) = p_y;
            del_y_arr = linspace(0,p_y,2*num_Ay+1);
            delta_y   = (del_y_arr(2)-del_y_arr(1));
            del_y_arr = del_y_arr(2:2:end);
            for Ay_idx = 1:num_Ay
                ratio_Ay = linspace(-1,1,sim_end_idx);
                for sim_idx = 1:sim_end_idx 
                    Agent.reset([0;0;0;vx0;0;0;w0;0;0;0]);
                    Ay_sim = del_y_arr(Ay_idx) + ratio_Ay(sim_idx)*delta_y;
                    if mode == 1 
                        [T_go,U_go,Z_go] = sin_one_hump_parameterized_traj_with_brake(0,Ay_sim,p_vx,vx0,t,0, 1);
                    else
                        [T_go,U_go,Z_go] = gaussian_T_parameterized_traj_with_brake(0,Ay_sim,p_vx,vx0,t,0, 1, isSim);
                    end
                    if mode == 1 
                        Agent.move(tpk_dir,T_go,U_go,Z_go) ;
                    else
                        Agent.move(tpk,T_go,U_go,Z_go) ;
                    end
                    
                    X = Agent.state';
                    T = Agent.time';
                    for t0_idx = 1:end_idx
                        t0_val = t0_dt*(t0_idx-1);
                        [~,t_idx_real] = min(abs(T-t0_val));
                        vy_value_vec(sim_idx,t0_idx,Ay_idx, vx0idx)  =  X(t_idx_real,5) ;% save vy and r to find their ranges
                        r_value_vec(sim_idx,t0_idx,Ay_idx, vx0idx)  =  X(t_idx_real,6);
                    end
                end
            end
        elseif delta_y < y_ideal
            p_y_lower = p_y;
        elseif delta_y > y_ideal
            p_y_upper = p_y;
        end
        
        if isSim
            r0vy0_limit(vx0idx,1) = deg2rad(4.50); %this is obtained by testing on all reference traj
            r0vy0_limit(vx0idx,2) = 0.013; %this is obtained by testing on all reference traj
        else
            r0vy0_limit(vx0idx,1) = deg2rad(3.7);%this data is obtained using experiment on all reference traj
            r0vy0_limit(vx0idx,2) = 0.013;%this data is obtained using experiment on all reference traj
        end
    end
end
%% post-process to get r0v0 limit
% using 3D array or 4D array to store subdivision information
r0_limit_gen = zeros(size(r_value_vec,2),size(r_value_vec,3),size(r_value_vec,4));
vy0_limit_gen = zeros(size(vy_value_vec,2),size(vy_value_vec,3),size(vy_value_vec,4));
r0_limit_c = permute(max(r_value_vec,[],1)+min(r_value_vec,[],1),[2 3 4 1])/2;
vy0_limit_c = permute(max(vy_value_vec,[],1)+min(vy_value_vec,[],1),[2 3 4 1])/2;
for i = 1:size(r0_limit_gen,3) 
    r0_min =  r_value_vec(:,:,:,i)-repmat(r0vy0_limit(i,1),[sim_end_idx, size(r0_limit_gen,1),size(r0_limit_gen,2)]);
    r0_max =  r_value_vec(:,:,:,i)+repmat(r0vy0_limit(i,1),[sim_end_idx, size(r0_limit_gen,1),size(r0_limit_gen,2)]);
    r0_extreme = cat(1,r0_min,r0_max); 
    r0_limit_gen(:,:,i) = squeeze(range(r0_extreme,1)/2);
    v0_min =  vy_value_vec(:,:,:,i)-repmat(r0vy0_limit(i,2),[sim_end_idx, size(vy0_limit_gen,1),size(vy0_limit_gen,2)]);
    v0_max =  vy_value_vec(:,:,:,i)+repmat(r0vy0_limit(i,2),[sim_end_idx, size(vy0_limit_gen,1),size(vy0_limit_gen,2)]);
    vy0_extreme = cat(1,v0_min,v0_max); 
    vy0_limit_gen(:,:,i) = squeeze(range(vy0_extreme,1)/2);
end
if mode == 1
    file_name = "dir_change_Ay_info.mat";
elseif mode == 2
    file_name = "lane_change_Ay_info.mat";
end
save(file_name,'Ay_vec','vx0_vec','vy_value_vec','r_value_vec','r0vy0_limit','r0_limit_c','vy0_limit_c','r0_limit_gen','vy0_limit_gen','num_Ay','t0_dt','vx0_gen');



