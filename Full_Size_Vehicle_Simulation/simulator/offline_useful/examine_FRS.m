clear;
dir_folder = "./data/frs_collected/";
files = dir(dir_folder+'*.mat');

calc_force_flag = false;
extended_r0v0_table = true;% see slides for help, this adds extra four rows into the table so we can check the initial state to see if it is a verified initial condition
dim = 14;

options.tensorParallel = 0;%turn this to 1 if want to use parallel, not working very well.
% set options for reachability analysis:
options.taylorTerms=15; % number of taylor terms for reachable sets
options.zonotopeOrder= 100; % zonotope order... increase this for more complicated systems.
options.maxError = 1e100*ones(dim, 1); % our zonotopes shouldn't be "splitting", so this term doesn't matter for now
options.verbose = 0;
options.uTrans = 0; % we won't be using any inputs, as traj. params specify trajectories
options.U = zonotope([0, 0]);
options.advancedLinErrorComp = 0;
options.tensorOrder = 1;
options.errorOrder = 50;
options.reductionInterval = inf;
options.reductionTechnique = 'girard';

options.tStart = 0;
options.tFinal = 1;
options.reachabilitySteps=1;% discrete 1 time step prop
options.timeStep=options.tFinal/options.reachabilitySteps;
lane_info = load('lane_change_Ay_info.mat');
dir_info = load('dir_change_Ay_info.mat');
predict_err_lat = load('lateral_prediction_error.mat');
predict_err_spd = load('spd_change_prediction_error.mat');
load_const
dt = lane_info.t0_dt;
t0_vec = 0:dt:tpk-dt;
% u0_vec = 5:2:27;
u0_vec = lane_info.u0_vec;
lane_time_bin_num = tpk/dt;
dir_time_bin_num = tpk_dir/dt;
dir_num = dir_info.num_Ay;
lan_num = lane_info.num_Ay;
M_mega = cell(0);
%dim 1 is Ay/Au , dX, dY, dtheta, v0_lower, v0_upper, r0_lower, r0_upper
%dim 2 is different Au/Ay
%dim 3 is t0_idx

for u0 = u0_vec
    M = containers.Map;
    M(char("Autb"))= [];  %keep this size loose since the number of actions may be different for each spd
    M(char("Au"))= cell(0);
    
    if extended_r0v0_table %uses the limits that taking forces into account from find_proper_reference_brute
        tb = -1*ones(8,dir_num,dir_time_bin_num); %see slides for dimension help
        u0_idx = find(dir_info.u0_vec == u0);
        for t0_idx = 1:dir_time_bin_num
            for Ay_idx = 1:dir_num
                tb(5,Ay_idx,t0_idx) = dir_info.v0_limit_c(t0_idx,Ay_idx,u0_idx)-dir_info.v0_limit_gen(t0_idx,Ay_idx,u0_idx);
                tb(6,Ay_idx,t0_idx) = dir_info.v0_limit_c(t0_idx,Ay_idx,u0_idx)+dir_info.v0_limit_gen(t0_idx,Ay_idx,u0_idx);
                tb(7,Ay_idx,t0_idx) = dir_info.r0_limit_c(t0_idx,Ay_idx,u0_idx)-dir_info.r0_limit_gen(t0_idx,Ay_idx,u0_idx);
                tb(8,Ay_idx,t0_idx) = dir_info.r0_limit_c(t0_idx,Ay_idx,u0_idx)+dir_info.r0_limit_gen(t0_idx,Ay_idx,u0_idx);
            end
        end
        M(char("dirtb"))= tb;
    else
        M(char("dirtb"))= -1*ones(4,dir_num,dir_time_bin_num); %see slides for dimention help
    end
    
    
    M(char("dir"))= cell(dir_num,dir_time_bin_num);
    
    if extended_r0v0_table
        tb =-1*ones(8,lan_num,lane_time_bin_num);
        u0_idx = find(lane_info.u0_vec == u0);
        for t0_idx = 1:lane_time_bin_num
            for Ay_idx = 1:lan_num
                tb(5,Ay_idx,t0_idx) = lane_info.v0_limit_c(t0_idx,Ay_idx,u0_idx)-lane_info.v0_limit_gen(t0_idx,Ay_idx,u0_idx);
                tb(6,Ay_idx,t0_idx) = lane_info.v0_limit_c(t0_idx,Ay_idx,u0_idx)+lane_info.v0_limit_gen(t0_idx,Ay_idx,u0_idx);
                tb(7,Ay_idx,t0_idx) = lane_info.r0_limit_c(t0_idx,Ay_idx,u0_idx)-lane_info.r0_limit_gen(t0_idx,Ay_idx,u0_idx);
                tb(8,Ay_idx,t0_idx) = lane_info.r0_limit_c(t0_idx,Ay_idx,u0_idx)+lane_info.r0_limit_gen(t0_idx,Ay_idx,u0_idx);
            end
        end
        M(char("lantb"))= tb;
    else
        M(char("lantb"))= -1*ones(4,lan_num,lane_time_bin_num);
    end
    
    
    M(char("lan"))= cell(lan_num,lane_time_bin_num);
    
    M_mega{end+1} = M;
    
end

for i=1:length(files)
    %     if ~contains(files(i).name,'spd_change')
    %         continue;
    %     end
    %     if contains(files(i).name,'_u=5_')
    display(files(i).name)
    FRS = load ([dir_folder+files(i).name]);
    peak_idx = 1.5*10; % so during online rough optimization there is something of same reference
    %1.5 seconds, each zonotope is 0.1 sec
    if contains(files(i).name,'lane_change')
        peak_idx = 3*10;
    end
    c = center(FRS.vehRS_save{peak_idx});
    dx = c(1);
    dy = c(2);
    dh = c(3);
    
    idx1 = strfind(files(i).name,'_u=');
    idx2 = strfind(files(i).name,'_A');
    u0 = str2double(files(i).name(idx1+3:idx2-1));
    
    mega_idx = find(u0 == u0_vec);
    M = M_mega{mega_idx};
    
    if calc_force_flag
        FRS.delta_force ={};
        if contains(files(i).name,'lane_change') %get the dynamics ready for force calc
            dyn_arr = {@dyn_y_change_forces,@dyn_y_brake_forces};
        elseif contains(files(i).name,'dir_change')
            dyn_arr = {@dyn_dir_change_forces,@dyn_dir_brake_forces};
        elseif contains(files(i).name,'spd_change')
            dyn_arr = {@dyn_u_change_forces,@dyn_u_brake_forces};
        end
    end
    
    
    for frs_idx = length(FRS.vehRS_save):-1:1
        head_zono = deleteAligned(project(FRS.vehRS_save{frs_idx},3));
        footprint_obs_zono = get_error_zono(head_zono, [car_length car_width] + obs_localize_err, dim);% do some cleaning and get footprint based on heading uncertainty
        closest_time = frs_idx/10;
        [~,error_idx ] = min(abs(predict_err_lat.error_time_vector - closest_time));
%% Need to get error for failsafe as well
        if contains(files(i).name,'lane_change') %get the dynamics ready for force calc
            lon_gen = max(predict_err_lat.lon_err_lan_max(error_idx), 0);
            lat_gen = max(abs(predict_err_lat.lat_err_lan_min(error_idx)), predict_err_lat.lat_err_lan_max(error_idx));
            head_gen = max(abs(predict_err_lat.head_err_lan_min(error_idx)), predict_err_lat.head_err_lan_max(error_idx));
        elseif contains(files(i).name,'dir_change')
            lon_gen = max(predict_err_lat.lon_err_dir_max(error_idx),0);
            lat_gen = max(abs(predict_err_lat.lat_err_dir_min(error_idx)), predict_err_lat.lat_err_dir_max(error_idx));
            head_gen = max(abs(predict_err_lat.head_err_dir_min(error_idx)), predict_err_lat.head_err_dir_max(error_idx));
        elseif contains(files(i).name,'spd_change')
%  For now this is same as dir, need to calculate this from closed loop
%  data
            lon_gen = max(predict_err_spd.lon_err_dir_max(error_idx),0);
            lat_gen = max(abs(predict_err_spd.lat_err_dir_min(error_idx)), predict_err_spd.lat_err_dir_max(error_idx));
            head_gen = max(abs(predict_err_spd.head_err_dir_min(error_idx)), predict_err_spd.head_err_dir_max(error_idx));
        end
        
        predict_err_zono = get_error_zono(zonotope([0,deg2rad(head_gen)]), [lon_gen lat_gen], dim);
        % = get_error_zono(head_zono, [0.2, 0.1], dim);% do some cleaning and get footprint based on heading uncertainty
        FRS.vehRS_save{frs_idx}= FRS.vehRS_save{frs_idx}+footprint_obs_zono + predict_err_zono;
        if calc_force_flag
            options.x0 = center(FRS.vehRS_save{frs_idx});
            options.R0 = deleteZeros(deleteAligned(FRS.vehRS_save{frs_idx}));
            if frs_idx <= FRS.brake_idx(1)
                dyn = dyn_arr{1};
            else
                dyn = dyn_arr{2};
            end
            sysDisc = nonlinearSysDT(dim,1,dyn,options);
            R = reach(sysDisc,options);
            FRS.delta_force{end+1} = deleteZeros(deleteAligned(R{2}));
            %         zonotope_slice(FRS.delta_force{end}, [7;8;9;12], [u0;0;0;-0.0148]);
        end
    end
    if contains(files(i).name,'lane_change') || contains(files(i).name,'dir_change')
        idx1 = strfind(files(i).name,'_Ay=');
        idx2 = strfind(files(i).name,',');
        idx3 = strfind(files(i).name,'.mat');
        Ay_idx = str2double(files(i).name(idx1+4:idx2-1));
        Ay     = str2double(files(i).name(idx2+1:idx3-1));
        idxt1 = strfind(files(i).name,'t0=');
        idxt2 = strfind(files(i).name,'_u=');
        t0 = str2double(files(i).name(idxt1+3:idxt2-1));
        t0_idx = t0/dt+1;
        
        if contains(files(i).name,'lane_change')
            lantb = M(char("lantb"));
            lantb(1:4,Ay_idx,t0_idx) = [Ay;dx;dy;dh];
            M(char("lantb")) = lantb;
            
            MAy = M(char("lan"));
            MAy{Ay_idx,t0_idx} = FRS;
            M("lan") = MAy;
            
            
        else %contains(files(i).name,'dir_change')
            dirtb = M(char("dirtb"));
            dirtb(1:4,Ay_idx,t0_idx) = [Ay;dx;dy;dh];
            M(char("dirtb")) = dirtb;
            
            MAy = M(char("dir"));
            MAy{Ay_idx,t0_idx} = FRS;
            M("dir") = MAy;
        end
    elseif contains(files(i).name,'spd_change')
        idx1 = strfind(files(i).name,'_Au=');
        idx2 = strfind(files(i).name,',');
        Au = str2double(files(i).name(idx1+4:idx2-1));%t0_idx
                                                %t0idx ayidx u0idx
                                                %previously generated with
                                                %low r0v0 values;
        r0v0_spd_limits = [lane_info.v0_limit_c(1,1,mega_idx)-lane_info.v0_limit_gen(1,1,mega_idx);
            lane_info.v0_limit_c(1,1,mega_idx)+lane_info.v0_limit_gen(1,1,mega_idx);
            lane_info.r0_limit_c(1,1,mega_idx)-lane_info.r0_limit_gen(1,1,mega_idx);
            lane_info.r0_limit_c(1,1,mega_idx)+lane_info.r0_limit_gen(1,1,mega_idx);];
        M(char("Autb")) =  [M(char("Autb")) [Au;dx;dy;0;r0v0_spd_limits]];%Au change shold allow uncertainty in r and v as well
        MAu = M(char("Au"));
        MAu{end+1} = FRS;
        M('Au') = MAu;
    end
end

save(dir_folder+"../FRS_Rover_"+string(datetime('today'))+"_no_force.mat",'M_mega')
