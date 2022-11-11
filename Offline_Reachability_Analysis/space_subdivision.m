% This script tries to find proper subdivision of the initial condition
% space and control parameter space for direction change and lane change.
% We aim to achieve enough lateral displacement during a direction or lane 
% change maneuver.  

clear, close all; clear
full_size_const; 
save my_const.mat
warning('off','ALL')


mode = 1; %mode 1 for dir change, mode 2 for lane change

x0 = 0;
y0 = 0;
h0 = 0;
u0_vec = 5.25:0.5:29.75; % partition on the initial condition space of u
u0_gen = 0.5*(u0_vec(2)-u0_vec(1));
v0 = 0;
t0 = 0;
r0 = 0;
t0_dt = 3; % because time duration of a lane change maneuver is as twice as that of a speed/direction change maneuver, we use t0_dt to indicate if we execute the lane change maneuver from the beginning or middle.


Ay_vec= zeros(length(u0_vec),1); 

sim_end_idx = 5;
r0v0_limit = zeros(length(u0_vec),2);
if mode == 2
    y_ideal = 4; % lane change seeks to achieve 4m of lateral change.
    num_Ay = 2;
elseif mode == 1
    y_ideal = 3; % lane change seeks to achieve 3m of lateral change.
    num_Ay = 2;
end

if mode == 1
    end_idx = tpk_dir/t0_dt;
else
    end_idx = tpk/t0_dt;
end
r_value_vec = zeros(2,end_idx,num_Ay,length(u0_vec)); %value of r at different time stage of a turn
v_value_vec = zeros(2,end_idx,num_Ay,length(u0_vec)); %value of v at different time stage of a turn
for u0idx = 21:length(u0_vec)
    u0 = u0_vec(u0idx) % initial longitudinal speed
    w0 = u0; % initial wheel speed.
    %% Test system behavior over each maneuver via simulaion
    A_go = highway_cruising_10_state_agent('use_rover_controller',false) ;
    
    z_0 = [x0;y0;h0;u0;v0;r0;w0;0;0;0] ;
    done_flag = 0;
    

    p_y_upper = 0.8; 
    p_y_lower= 0;
    
    while done_flag~= 1 
        p_y = mean([p_y_lower p_y_upper]);
        p_u = u0; 
        if mode == 1 
            t= [];
            [T_go,U_go,Z_go] = sin_one_hump_parameterized_traj_with_brake(t0,p_y,p_u,u0,t,0, 1); % T: time. U: control input. Z: state (ideal)
        else
            t= [];
            [T_go,U_go,Z_go] = gaussian_T_parameterized_traj_with_brake(t0,p_y,p_u,u0,t,0, 1);
        end
        A_go.reset(z_0) ;
        if mode == 1 
            A_go.move(tpk_dir,T_go,U_go,Z_go) ; 
        else
            A_go.move(tpk,T_go,U_go,Z_go) ;
        end
        
        
        X = A_go.state';
        T = A_go.time';
        delta_y = X(end, 2);

        if abs(delta_y-y_ideal) <0.01 % found the right Ay
            done_flag = 1; 
            Ay_vec(u0idx) = p_y;
            del_y_arr = linspace(0,p_y,2*num_Ay+1);
            delta_y   = (del_y_arr(2)-del_y_arr(1));
            del_y_arr = del_y_arr(2:2:end);
            for Ay_idx = 1:num_Ay
                ratio_Ay = linspace(-1,1,sim_end_idx);
                for sim_idx = 1:sim_end_idx 
                    A_go.reset([0;0;0;u0;0;0;w0;0;0;0]);
                    Ay_sim = del_y_arr(Ay_idx) + ratio_Ay(sim_idx)*delta_y;
                    if mode == 1 
                        [T_go,U_go,Z_go] = sin_one_hump_parameterized_traj_with_brake(0,Ay_sim,p_u,u0,t,0, 1);
                    else
                        [T_go,U_go,Z_go] = gaussian_T_parameterized_traj_with_brake(0,Ay_sim,p_u,u0,t,0, 1);
                    end
                    if mode == 1 
                        A_go.move(tpk_dir,T_go,U_go,Z_go) ;
                    else
                        A_go.move(tpk,T_go,U_go,Z_go) ;
                    end
                    
                    X = A_go.state';
                    T = A_go.time';
                    for t0_idx = 1:end_idx
                        t0_val = t0_dt*(t0_idx-1);
                        [~,t_idx_real] = min(abs(T-t0_val));
                        v_value_vec(sim_idx,t0_idx,Ay_idx, u0idx)  =  X(t_idx_real,5) ;% save v and r to find their ranges
                        r_value_vec(sim_idx,t0_idx,Ay_idx, u0idx)  =  X(t_idx_real,6);
                    end
                end
            end
        elseif delta_y < y_ideal
            p_y_lower = p_y;
        elseif delta_y > y_ideal
            p_y_upper = p_y;
        end
        

        r0v0_limit(u0idx,1) = deg2rad(4.50); %this is obtained by testing on all reference traj
        r0v0_limit(u0idx,2) = 0.013; %this is obtained by testing on all reference traj
    end
end
%% post-process to get r0v0 limit
% using 3D array or 4D array to store subdivision information
r0_limit_gen = zeros(size(r_value_vec,2),size(r_value_vec,3),size(r_value_vec,4));
v0_limit_gen = zeros(size(v_value_vec,2),size(v_value_vec,3),size(v_value_vec,4));
r0_limit_c = permute(max(r_value_vec,[],1)+min(r_value_vec,[],1),[2 3 4 1])/2;
v0_limit_c = permute(max(v_value_vec,[],1)+min(v_value_vec,[],1),[2 3 4 1])/2;
for i = 1:size(r0_limit_gen,3) 
    r0_min =  r_value_vec(:,:,:,i)-repmat(r0v0_limit(i,1),[sim_end_idx, size(r0_limit_gen,1),size(r0_limit_gen,2)]);
    r0_max =  r_value_vec(:,:,:,i)+repmat(r0v0_limit(i,1),[sim_end_idx, size(r0_limit_gen,1),size(r0_limit_gen,2)]);
    r0_extreme = cat(1,r0_min,r0_max); 
    r0_limit_gen(:,:,i) = squeeze(range(r0_extreme,1)/2);
    v0_min =  v_value_vec(:,:,:,i)-repmat(r0v0_limit(i,2),[sim_end_idx, size(v0_limit_gen,1),size(v0_limit_gen,2)]);
    v0_max =  v_value_vec(:,:,:,i)+repmat(r0v0_limit(i,2),[sim_end_idx, size(v0_limit_gen,1),size(v0_limit_gen,2)]);
    v0_extreme = cat(1,v0_min,v0_max); 
    v0_limit_gen(:,:,i) = squeeze(range(v0_extreme,1)/2);
end
if mode == 1
    file_name = "dir_change_Ay_info.mat";
elseif mode == 2
    file_name = "lane_change_Ay_info.mat";
end
save(file_name,'Ay_vec','u0_vec','v_value_vec','r_value_vec','r0v0_limit','r0_limit_c','v0_limit_c','r0_limit_gen','v0_limit_gen','num_Ay','t0_dt','u0_gen');



