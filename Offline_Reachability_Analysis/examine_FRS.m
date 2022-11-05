clear;
dir_folder = "./FRSdata/";
files = dir(dir_folder+'*.mat');

dim = 20;

lane_info = load('lane_change_Ay_info.mat');
dir_info = load('dir_change_Ay_info.mat');
load my_const.mat
dt = dir_info.t0_dt;
t0_vec = 0:dt:tpk-dt;
u0_vec = dir_info.u0_vec;
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
    

    tb = nan*ones(8+6,dir_num,dir_time_bin_num); %see slides for dimension help
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

    
    
    M(char("dir"))= cell(dir_num,dir_time_bin_num);
    

    tb =nan*ones(8+6,lan_num,lane_time_bin_num);
    u0_idx = find(lane_info.u0_vec == u0);
    if ~isempty(u0_idx) 
        for t0_idx = 1:lane_time_bin_num
            for Ay_idx = 1:lan_num
                tb(5,Ay_idx,t0_idx) = lane_info.v0_limit_c(t0_idx,Ay_idx,u0_idx)-lane_info.v0_limit_gen(t0_idx,Ay_idx,u0_idx);
                tb(6,Ay_idx,t0_idx) = lane_info.v0_limit_c(t0_idx,Ay_idx,u0_idx)+lane_info.v0_limit_gen(t0_idx,Ay_idx,u0_idx);
                tb(7,Ay_idx,t0_idx) = lane_info.r0_limit_c(t0_idx,Ay_idx,u0_idx)-lane_info.r0_limit_gen(t0_idx,Ay_idx,u0_idx);
                tb(8,Ay_idx,t0_idx) = lane_info.r0_limit_c(t0_idx,Ay_idx,u0_idx)+lane_info.r0_limit_gen(t0_idx,Ay_idx,u0_idx);
            end
        end 
    else
        aaa = 1
    end
    M(char("lantb"))= tb;

    
    
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
    %% commented out peak idx choose
%     peak_idx = 1.5*10; % so during online rough optimization there is something of same reference
%     %1.5 seconds, each zonotope is 0.1 sec
%     if contains(files(i).name,'lane_change') && contains(files(i).name,'t0=0_')
%         peak_idx = 1.5*10;
%     end
%     peak_idx_1 = peak_idx/3;
%     peak_idx_2 = peak_idx/3*2;
%     if contains(files(i).name,'spd_change') || contains(files(i).name,'lane_change') 
%         peak_idx = 150;
%         peak_idx_1 = 50;
%         peak_idx_2 = 100;
%     else
        peak_idx = FRS.brake_idx1;
        peak_idx_1 = round(FRS.brake_idx1/3);
        peak_idx_2 = peak_idx_1*2;
%     end
    c = center(FRS.vehRS_save{peak_idx});
    dx = c(1);
    dy = c(2);
    dh = c(3);
    c1 = center(FRS.vehRS_save{peak_idx_1});
    c2 = center(FRS.vehRS_save{peak_idx_2});
    dx1 = c1(1);
    dy1 = c1(2);
    dh1 = c1(3);
    dx2 = c2(1);
    dy2 = c2(2);
    dh2 = c2(3);
    
    idx1 = strfind(files(i).name,'_u=');
    idx2 = strfind(files(i).name,'_A');
    u0 = str2double(files(i).name(idx1+3:idx2-1));
    
    mega_idx = find(abs(u0_vec - u0)<0.01);
    M = M_mega{mega_idx};
    
    dist_arr = [];
    
    for frs_idx = length(FRS.vehRS_save):-1:1
%         if mod(frs_idx, 10) ~= 0
%             FRS.vehRS_save(frs_idx) = [];
%             continue;
%         end
        zono_center = FRS.vehRS_save{frs_idx}.center;
        dist_arr = [abs(zono_center(end) -1.5) dist_arr];
        head_zono = deleteAligned(project(FRS.vehRS_save{frs_idx},3));
        % do not add localize error here, add in obstacle
        footprint_obs_zono = get_error_zono(head_zono, [car_length car_width], dim);% do some cleaning and get footprint based on heading uncertainty
%         closest_time = frs_idx/10;
%         [~,error_idx ] = min(abs(predict_err_lat.error_time_vector - closest_time));
% %% Need to get error for failsafe as well
%         if contains(files(i).name,'lane_change') %get the dynamics ready for force calc
%             lon_gen = max(predict_err_lat.lon_err_lan_max(error_idx), 0);
%             lat_gen = max(abs(predict_err_lat.lat_err_lan_min(error_idx)), predict_err_lat.lat_err_lan_max(error_idx));
%             head_gen = max(abs(predict_err_lat.head_err_lan_min(error_idx)), predict_err_lat.head_err_lan_max(error_idx));
%         elseif contains(files(i).name,'dir_change')
%             lon_gen = max(predict_err_lat.lon_err_dir_max(error_idx),0);
%             lat_gen = max(abs(predict_err_lat.lat_err_dir_min(error_idx)), predict_err_lat.lat_err_dir_max(error_idx));
%             head_gen = max(abs(predict_err_lat.head_err_dir_min(error_idx)), predict_err_lat.head_err_dir_max(error_idx));
%         elseif contains(files(i).name,'spd_change')
% %  For now this is same as dir, need to calculate this from closed loop
% %  data
%             lon_gen = max(predict_err_spd.lon_err_dir_max(error_idx),0);
%             lat_gen = max(abs(predict_err_spd.lat_err_dir_min(error_idx)), predict_err_spd.lat_err_dir_max(error_idx));
%             head_gen = max(abs(predict_err_spd.head_err_dir_head_zonomin(error_idx)), predict_err_spd.head_err_dir_max(error_idx));
%         end
%         
%         predict_err_zono = get_error_zono(zonotope([0,deg2rad(head_gen)]), [lon_gen lat_gen], dim);
        % = get_error_zono(head_zono, [0.2, 0.1], dim);% do some cleaning and get footprint based on heading uncertainty
        %We got rid of predict error using a robust controller, and putting
        %uncertainty and robustness into CORA
        FRS.vehRS_save{frs_idx}= FRS.vehRS_save{frs_idx} +footprint_obs_zono;
        
    end
    [time_dist_min, min_idx] = min(dist_arr);
    min_idx
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
            lantb(9:14,Ay_idx,t0_idx) = [dx1;dy1;dh1;dx2;dy2;dh2];
            M(char("lantb")) = lantb;
            
            MAy = M(char("lan"));
            MAy{Ay_idx,t0_idx} = FRS;
            M("lan") = MAy;
            
            
        else %contains(files(i).name,'dir_change')
            dirtb = M(char("dirtb"));
            dirtb(1:4,Ay_idx,t0_idx) = [Ay;dx;dy;dh];
            dirtb(9:14,Ay_idx,t0_idx) = [dx1;dy1;dh1;dx2;dy2;dh2];
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
        r0v0_spd_limits = [dir_info.v0_limit_c(1,1,mega_idx)-dir_info.v0_limit_gen(1,1,mega_idx);
            dir_info.v0_limit_c(1,1,mega_idx)+dir_info.v0_limit_gen(1,1,mega_idx);
            dir_info.r0_limit_c(1,1,mega_idx)-dir_info.r0_limit_gen(1,1,mega_idx);
            dir_info.r0_limit_c(1,1,mega_idx)+dir_info.r0_limit_gen(1,1,mega_idx);];
        M(char("Autb")) =  [M(char("Autb")) [Au;dx;dy;dh;r0v0_spd_limits;dx1;dy1;dh1;dx2;dy2;dh2]];%Au change shold allow uncertainty in r and v as well
        MAu = M(char("Au"));
        MAu{end+1} = FRS;
        M('Au') = MAu;
    end
end

save(dir_folder+"../FRS_Rover_"+string(datetime('today'))+"_no_force.mat",'M_mega','-v7.3','-nocompression')
