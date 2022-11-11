% this script wraps up all FRS into a map structure

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
%dim 1 is p_u or p_y , dX, dY, dtheta, v0_lower, v0_upper, r0_lower, r0_upper
%dim 2 is for different values of p_u or p_y
%dim 3 is for different values of t0_idx (in our computation t0_idx = 0 always)

for u0 = u0_vec
    M = containers.Map;
    M(char("Autb"))= [];  %keep this size loose since the number of actions may be different for each spd
    M(char("Au"))= cell(0);
    
    tb = nan*ones(8+6,dir_num,dir_time_bin_num); 
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
    end
    M(char("lantb"))= tb;
    M(char("lan"))= cell(lan_num,lane_time_bin_num);
    M_mega{end+1} = M;
end

for i=1:length(files)
    display(files(i).name)
    FRS = load ([dir_folder+files(i).name]);
    peak_idx = FRS.brake_idx1;
    peak_idx_1 = round(FRS.brake_idx1/3);
    peak_idx_2 = peak_idx_1*2;
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
        zono_center = FRS.vehRS_save{frs_idx}.center;
        dist_arr = [abs(zono_center(end) -1.5) dist_arr];
        head_zono = deleteAligned(project(FRS.vehRS_save{frs_idx},3));
        footprint_obs_zono = account_for_footprint(head_zono, [car_length car_width], dim);% get footprint based on heading uncertainty
        FRS.vehRS_save{frs_idx}= FRS.vehRS_save{frs_idx} +footprint_obs_zono;
        
    end
    [time_dist_min, min_idx] = min(dist_arr);
    min_idx
    if contains(files(i).name,'lane_change') || contains(files(i).name,'dir_change')
        idx1 = strfind(files(i).name,'_p_y=');
        idx2 = strfind(files(i).name,',');
        idx3 = strfind(files(i).name,'.mat');
        Ay_idx = str2double(files(i).name(idx1+4:idx2-1));
        Ay     = str2double(files(i).name(idx2+1:idx3-1));
        idxt1 = strfind(files(i).name,'t0=');
        idxt2 = strfind(files(i).name,'_u0=');
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
            
            
        else 
            dirtb = M(char("dirtb"));
            dirtb(1:4,Ay_idx,t0_idx) = [Ay;dx;dy;dh];
            dirtb(9:14,Ay_idx,t0_idx) = [dx1;dy1;dh1;dx2;dy2;dh2];
            M(char("dirtb")) = dirtb;
            
            MAy = M(char("dir"));
            MAy{Ay_idx,t0_idx} = FRS;
            M("dir") = MAy;
        end
    elseif contains(files(i).name,'spd_change')
        idx1 = strfind(files(i).name,'_p_u=');
        idx2 = strfind(files(i).name,',');
        Au = str2double(files(i).name(idx1+4:idx2-1));
        r0v0_spd_limits = [dir_info.v0_limit_c(1,1,mega_idx)-dir_info.v0_limit_gen(1,1,mega_idx);
                           dir_info.v0_limit_c(1,1,mega_idx)+dir_info.v0_limit_gen(1,1,mega_idx);
                           dir_info.r0_limit_c(1,1,mega_idx)-dir_info.r0_limit_gen(1,1,mega_idx);
                           dir_info.r0_limit_c(1,1,mega_idx)+dir_info.r0_limit_gen(1,1,mega_idx);];
        M(char("Autb")) =  [M(char("Autb")) [Au;dx;dy;dh;r0v0_spd_limits;dx1;dy1;dh1;dx2;dy2;dh2]];
        MAu = M(char("Au"));
        MAu{end+1} = FRS;
        M('Au') = MAu;
    end
end

save(dir_folder+"../car_frs.mat",'M_mega','-v7.3','-nocompression')
