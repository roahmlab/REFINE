%% description

% define reference, use binary search to find proper Ay to get to a proper lane change width; i.e 4m +- 0.05m
% above 28m/s there is a non-trivial phase offset of the v, and r achieved during simulation.
% therefore a different reference trajecotry is needed for that
close all; clear
full_size_const; 
save my_const.mat
warning('off','ALL')


mode = 1; %mode 1 for dir change, mode 2 for lane change

% real_car_scale = 10;

x0 = 0;
y0 = 0;
h0 = 0;
u0_vec = 5.25:0.5:29.75;
u0_gen = 0.5*(u0_vec(2)-u0_vec(1));
v0 = 0;
t0 = 0;
r0 = 0;
t0_dt = 3;


Ay_vec= zeros(length(u0_vec),1);

sim_end_idx = 5;
r0v0_limit = zeros(length(u0_vec),2);%r0 delta , and then v0 delta
if mode == 2
    y_ideal = 4; %both dir change and lane change achieve 4 m of change.
% this means dir change needs a higher Ay
    num_Ay = 2;
elseif mode == 1
    y_ideal = 3;
    num_Ay = 2;
end

if mode == 1
%     t_f = tpk_dir +tbrk; %3+4  %have new adaptive braking time, not 4 sec anymore 
    end_idx = tpk_dir/t0_dt;
else
%     t_f = tpk +tbrk; %6+4
    end_idx = tpk/t0_dt;
end
r_value_vec = zeros(2,end_idx,num_Ay,length(u0_vec));%value of r at different time stage of a turn
v_value_vec = zeros(2,end_idx,num_Ay,length(u0_vec));%value of v at different time stage of a turn
for u0idx = 1:length(u0_vec)% starting speed
    u0 = u0_vec(u0idx)
    w0 = u0;
    %% automated from here
    A_go = highway_cruising_10_state_agent('use_rover_controller',false) ;
    
    z_0 = [x0;y0;h0;u0;v0;r0;w0;0;0;0] ;
    syms_flag = 0;
    done_flag = 0;
    

    Ay_upper = 0.8;
    Ay_lower= 0;
    
    while done_flag~= 1 %binary search
        Ay = mean([Ay_lower Ay_upper]);
        
        % if set to one will make t symbolic, for dynamics generation
        % syms t Au u0 %Ay av0 au0 v0 r0
        Au = u0;  scale_Ay_flag = 1; % enable lane change manuver by setting this to 1
        if mode == 1 
            t= [];%linspace(0,tpk_dir,1001);
            [T_go,U_go,Z_go] = sin_one_hump_parameterized_traj_with_brake(t0,Ay,Au,u0,t,syms_flag, scale_Ay_flag);
        else
            t= [];%linspace(0,tpk,1000);
            %sin_two_hump_parameterized_traj_with_brake
            %gaussian_T_parameterized_traj_with_brake
            [T_go,U_go,Z_go] = gaussian_T_parameterized_traj_with_brake(t0,Ay,Au,u0,t,syms_flag, scale_Ay_flag);
        end
        %% run iterative find vd rd exercise
        A_go.reset(z_0) ;
        % figure(2); hold on; axis auto
        if syms_flag
            %     ezplot(U_go(1,1), [0,tpk]);
            ezplot(Z_go(1,1), [0,tpk]);
            ezplot(U_go(2,1), [0,tpk]);
            ezplot(U_go(3,1), [0,tpk]);
            ezplot(U_ga(3,1), [0,tpk]);
            ezplot(U_ga(5,1), [0,tpk]);
            
            legend('int(vd)','vd','rd','vgaussian','rgaussian')
        else
            %     plot(T_go, U_go)
            %     plot(Z_go(1,:), Z_go(2,:))
        end
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
            
            Ay_vec(u0idx) = Ay;
            del_y_arr = linspace(0,Ay,2*num_Ay+1);%    del_y_arr = -0.8:0.2:0; % look at the 4 bins to look at avtual v r values
            delta_y   = (del_y_arr(2)-del_y_arr(1));
            del_y_arr = del_y_arr(2:2:end);
            
            %             delyspd = 0.5*(del_y_arr(2) - del_y_arr(1));
            for Ay_idx = 1:num_Ay
                ratio_Ay = linspace(-1,1,sim_end_idx);
                for sim_idx = 1:sim_end_idx 
                    A_go.reset([0;0;0;u0;0;0;w0;0;0;0]);
                    Ay_sim = del_y_arr(Ay_idx) + ratio_Ay(sim_idx)*delta_y;% either plus or minus delta, it is repetitive i know
                    if mode == 1 
                        [T_go,U_go,Z_go] = sin_one_hump_parameterized_traj_with_brake(0,Ay_sim,Au,u0,t,syms_flag, scale_Ay_flag);
                    else
                        [T_go,U_go,Z_go] = gaussian_T_parameterized_traj_with_brake(0,Ay_sim,Au,u0,t,syms_flag, scale_Ay_flag);
                    end
                    if mode == 1 
                        A_go.move(tpk_dir,T_go,U_go,Z_go) ;
                    else
                        A_go.move(tpk,T_go,U_go,Z_go) ;
                    end
                    
                    X = A_go.state';
                    T = A_go.time';
                    for t0_idx = 1:end_idx
                        %here don't need to input t0idx, just take the piece of time you need
                        t0_val = t0_dt*(t0_idx-1);
                        [~,t_idx_real] = min(abs(T-t0_val));
                        v_value_vec(sim_idx,t0_idx,Ay_idx, u0idx)  =  X(t_idx_real,5);%save v and r to find it's range
                        r_value_vec(sim_idx,t0_idx,Ay_idx, u0idx)  =  X(t_idx_real,6);
                    end
                end
            end
        elseif delta_y < y_ideal
            Ay_lower = Ay;
        elseif delta_y > y_ideal
            Ay_upper = Ay;
        end
        
        
        %get tire forces
        %[dzdt, Fxwf, Fywf, Fywr ,delta ,w_cmd, v, r, Fxwr] 
        [~, ~, ~, ~ ,delta ,w_cmd, ~, ~, ~] = cellfun(@(t,x)  A_go.dynamics(t,x.',T_go,U_go,Z_go), num2cell(T), num2cell(X,[2]),'uni',0);
        r0v0_limit(u0idx,1) = deg2rad(4.50); %this data is obtained using experiment on all reference traj
        r0v0_limit(u0idx,2) = 0.013; %this data is obtained using experiment on all reference traj
    end
end
%% pose process to get r0v0 limit
%minmax t0 Ayidx u0idx
% r0 v0 should not require the tires to exert more than 30 percent of the peak force that was
% achieved during the bin. 
r0_limit_gen = zeros(size(r_value_vec,2),size(r_value_vec,3),size(r_value_vec,4));
v0_limit_gen = zeros(size(v_value_vec,2),size(v_value_vec,3),size(v_value_vec,4));
               % t0 idx                    %ay idx            u0_idx
r0_limit_c = permute(max(r_value_vec,[],1)+min(r_value_vec,[],1),[2 3 4 1])/2;
v0_limit_c = permute(max(v_value_vec,[],1)+min(v_value_vec,[],1),[2 3 4 1])/2;
for i = 1:size(r0_limit_gen,3) %populate 3d array with original 4d arrary data %num_t0    %num_Ay
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



