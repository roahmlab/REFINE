% clc;clear
%% Dec 19
% lan_change_info_nov_8 = load('/home/nova/src/roahm/MATLAB_dependencies/lane_change_Ay_info.mat');
% dir_change_info_nov_8 = load('/home/nova/src/roahm/MATLAB_dependencies/dir_change_Ay_info.mat');
% frs_dec_19 = load('/home/nova/src/roahm/MATLAB_dependencies/FRS_Rover_19-Dec-2021_no_force.mat');
% write_frs(frs_dec_19, 'dec19_new_no_hull.txt', lan_change_info_nov_8, dir_change_info_nov_8, [], false);
%%
% % % % lan_change_info = load('/home/nova/downloads/frs-info-20211013/lane_change_Ay_info');
% % % % dir_change_info = load('/home/nova/downloads/frs-info-20211013/dir_change_Ay_info');
% % % % u0_vec_normal = lan_change_info.u0_vec;
% % % % bin_spacing_normal = diff(u0_vec_normal(1:2));
% % % % frs_min_spd_normal = u0_vec_normal(1) - (bin_spacing_normal / 2.0);
% % % % frs_max_spd_normal = u0_vec_normal(end) + (bin_spacing_normal / 2.0);
% % % % % frs = load('/home/nova/downloads/frs-info-20211013/FRS_Rover_13-Oct-2021_no_force_large_dir_1.8_lane_1.2.mat');


% jan_low = load('/home/roahm/Downloads/FRS_Rover_04-Jan-2022_low_spd.mat');
% u0_intervals_low = [0.00 0.05 0.15 0.25 0.35 0.45 0.55];
% u0_intervals_low = [u0_intervals_low(1:end-1);u0_intervals_low(2:end)];
% write_frs(jan_low, 'jan04low_int_new_no_hull___test.txt', [], [], u0_intervals_low, false);

% jan_hi = load('/home/roahm/Downloads/FRS_Rover_22-Jan-2022_no_force.mat');
% u0_intervals_hi = 5.5:0.5:20.0;
% u0_intervals_hi = [u0_intervals_hi-0.25;u0_intervals_hi+0.25];
% % u0_intervals_hi = [0.00 0.05 0.15 0.25 0.35 0.45 0.55];
% % u0_intervals_hi = [u0_intervals_hi(1:end-1);u0_intervals_hi(2:end)];
% write_frs(jan_hi, 'jan04hi_int_new_no_hull___test2.txt', [], [], u0_intervals_hi, false);

% frs_low = load('~/Desktop/Research/rover-rtd-su21/FRS_Rover_14-Sep-2021_low_spd.mat');
% low_max_center = 0.50;
% low_num = length(frs_low.M_mega);
% assert(low_num > 1) % Need to change below otherwise
% bin_spacing_low = low_max_center / (low_num-1);
% u0_vec_min_low = [-0.05  0.05  0.15  0.25  0.35  0.45];
% u0_vec_max_low = u0_vec_min_low + bin_spacing_low;
% u0_intervals_low = [u0_vec_min_low; u0_vec_max_low];
% write_frs(frs_low, 'jan14low_int.txt', [], [], u0_intervals_low, false);

% 
% frs_low = load('~/Desktop/Research/rover-rtd-su21/FRS_Rover_14-Sep-2021_low_spd.mat');
% low_max_center = 0.50;
% low_num = length(frs_low.M_mega);
% assert(low_num > 1) % Need to change below otherwise
% bin_spacing_low = low_max_center / (low_num-1);
% u0_vec_min_low = [-0.05  0.05  0.15  0.25  0.35  0.45];
% u0_vec_max_low = u0_vec_min_low + bin_spacing_low;
% u0_intervals_low = [u0_vec_min_low; u0_vec_max_low];
% write_frs(frs_low, 'sept14low_int.txt', [], [], u0_intervals_low, false);


% write_frs(frs, 'oct13.txt', lan_change_info, dir_change_info, [], true);

% write_info_struct(1, ' % .25f', 'LAN_CHANGE_INFO', lan_change_info);
% function write_info_struct(fid, float_fspec, name, info_struct)
%        fprintf(fid, "(%s)\n", name);
%        ay_vec = info_struct.Ay_vec;
%        ay_vec_sz = size(ay_vec, 1);
%        fprintf(fid, "AY_VEC %d\n", ay_vec_sz);
%        fprintf(fid, float_fspec, ay_vec);
%        
%        r0_limit_c = info_struct.r0_limit_c;
%        fprintf(fid, "NUM_AY %d\n", info_struct.num_Ay);
%        fprintf(fid, "r0_limit_c %d %d %d\n", size(r0_limit_c,1), ...
%            size(r0_limit_c,2), size(r0_limit_c,3));
%        fprintf(fid, float_fspec, r0_limit_c);
%        
%        r0_limit_gen = info_struct.r0_limit_gen;
%        fprintf(fid, "r0_limit_gen %d %d %d\n", size(r0_limit_gen,1), ...
%            size(r0_limit_gen, 2), size(r0_limit_gen, 3));
%        fprintf(fid, float_fspec, r0_limit_gen);
% end

write_frs(frs_full_data, 'test_frs_aug30_no_time.txt', [], [], u0_minmaxes_individ, false);

function write_frs(frs, out_fname, lan_change_info, dir_change_info, u0_intervals, alternating_au)
    has_info = ~isempty(lan_change_info) && ~isempty(dir_change_info);
    if has_info
        assert(all(lan_change_info.u0_vec == dir_change_info.u0_vec));
        u0_centers = lan_change_info.u0_vec;
        bin_spacing = diff(u0_centers(1:2));
        u0_mins = u0_centers - (bin_spacing / 2);
        u0_maxs = u0_mins + bin_spacing;
        u0_intervals = [u0_mins; u0_maxs];
    end
    frs_min_spd = u0_intervals(1, 1);
    frs_max_spd = u0_intervals(2, end);

    assert(exist('u0_intervals', 'var') && ~isempty(u0_intervals));
    float_fspec = '% .25f';
    fid = fopen(out_fname, 'W');
    minmax_fspec = "MINMAX_U0 " + float_fspec + " " + float_fspec + "\n";
    fprintf(fid, minmax_fspec, frs_min_spd, frs_max_spd);
    fprintf(fid, "ALTERNATING_AU %d\n", alternating_au);
    
    fprintf(fid, "NUM_MEGAS %d\n", length(frs.M_mega));
    for mega_idx = 1:length(frs.M_mega)
        fprintf(fid, "M_MEGA %d\n", mega_idx);
        frs_individ = frs.M_mega{mega_idx};
        
        curr_autb = try_load(frs_individ, 'Autb');
        curr_dirtb = try_load(frs_individ, 'dirtb');
        curr_lantb = try_load(frs_individ, 'lantb');
        au_individ = try_load(frs_individ, 'Au');
        lan_individ = try_load(frs_individ, 'lan');
        dir_individ = try_load(frs_individ, 'dir');

        fprintf(fid, "(MEGA_%d) AU %d\n", mega_idx, length(au_individ));
        if ~isempty(au_individ)
            for au_vehrs_idx = 1:length(au_individ)
                curr_au = au_individ(au_vehrs_idx);
                curr_vehrs = curr_au{1}.vehRS_save;
                curr_k_rng = get_k_rng(curr_autb, 1);
                print_vehrs_save(fid, float_fspec, curr_vehrs, 'AU', au_vehrs_idx, curr_k_rng);
            end
        end

        fprintf(fid, "(MEGA_%d) AUTB %d %d\n", mega_idx, size(curr_autb, 1), size(curr_autb, 2));
        for autb_row = 1:size(curr_autb, 1)
            for autb_col = 1:size(curr_autb, 2)
                fprintf(fid, float_fspec + " ", curr_autb(autb_row, autb_col));
            end
            fprintf(fid, "\n");
        end
    
        dir_len2 = 0;
        for j = 1:size(dir_individ, 2)
            if isempty(dir_individ{1, j})
                break
            end
            dir_len2 = dir_len2 + 1;
        end
        fprintf(fid, "(MEGA_%d) DIR %d %d\n", mega_idx, size(dir_individ, 1), dir_len2);
        for dir_row = 1:size(dir_individ, 1)
            for dir_col = 1:dir_len2
                curr_dir = dir_individ(dir_row, dir_col);
                assert(~isempty(curr_dir));
                curr_vehrs = curr_dir{1}.vehRS_save;
                curr_k_rng = get_k_rng(curr_dirtb, dir_col);
                print_vehrs_save(fid, float_fspec, curr_vehrs, 'DIR', [dir_row dir_col], curr_k_rng);
            end
        end
        
        fprintf(fid, "(MEGA_%d) DIRTB %d %d %d\n", mega_idx, size(curr_dirtb, 1), size(curr_dirtb, 2), dir_len2);
        for i = 1:size(curr_dirtb,1)
            for j = 1:size(curr_dirtb,2)
                for k = 1:dir_len2
                    fprintf(fid, float_fspec + " ", curr_dirtb(i, j, k));
                end
            end
        end
        fprintf(fid, "\n");

        
        lan_len2 = 0;
        for j = 1:size(lan_individ, 2)
            if isempty(lan_individ{1, j})
                break
            end
            lan_len2 = lan_len2 + 1;
        end

        fprintf(fid, "(MEGA_%d) LAN %d %d\n", mega_idx, size(lan_individ, 1), lan_len2);
        for lan_row = 1:size(lan_individ,1)
            for lan_col = 1:lan_len2
                assert(~isempty(lan_individ{lan_row, lan_col}));
                curr_vehrs = lan_individ{lan_row, lan_col}.vehRS_save;
                curr_k_rng = get_k_rng(curr_lantb, lan_col);
                print_vehrs_save(fid, float_fspec, curr_vehrs, 'LAN', [lan_row lan_col], curr_k_rng);
            end
        end

        fprintf(fid, "(MEGA_%d) LANTB %d %d %d\n", mega_idx, size(curr_lantb, 1), size(curr_lantb, 2), lan_len2);
        for i = 1:size(curr_lantb,1)
            for j = 1:size(curr_lantb,2)
                for k = 1:lan_len2
                    fprintf(fid, float_fspec + " ", curr_lantb(i, j, k));
                end
            end
        end
        fprintf(fid, "\n");
    end
    
    fprintf(fid, "u0_intervals %d %d\n", size(u0_intervals,1), size(u0_intervals,2));
    for i = 1:size(u0_intervals,1)
        for j = 1:size(u0_intervals,2)
            fprintf(fid, float_fspec + " ", u0_intervals(i,j));
        end
        fprintf(fid, "\n");
    end
    fclose(fid);
end

function ret = try_load(kv, key)
    try
        ret = kv(key);
    catch ...
        ret = [];
    end
end

function k_rng = get_k_rng(tb, t0_idx)
    assert(size(tb, 1) > 0);
    assert(size(tb, 2) > 0);
    if size(tb, 2) > 2
%         assert(...
%         all(diff(diff(tb(1,:,t0_idx))) < 1e-8)...
%         )
    end
    assert(t0_idx <= size(tb, 3));
%     if size(tb, 2) > 2
%         if any(diff(diff(tb(1,:,t0_idx))) >= 1e-8)
% %             bkpt = 1
%         end
%     end
    if size(tb, 2) > 1
        k_rng = abs(tb(1,2,t0_idx) - tb(1,1,t0_idx)) / 2;
    else
        k_rng = abs(tb(1,1,t0_idx));
    end
end

function print_vehrs_save(fid, float_fspec, vehrs_save, prefix, idxs, k_rng)
    t_eval_idx_c_style = find_t_eval_idx_c_style(vehrs_save);
    fprintf(fid, "(%s_%s) vehRS_save %d %d\n", prefix, join(string(idxs), "_"), length(vehrs_save), t_eval_idx_c_style);
    fprintf(fid, "K_RNG " + float_fspec + "\n", k_rng);
%     vehrs_convhull_cw = vehrs_convhull(vehrs_save);
%     fprintf(fid, "VEHRS_CONV_HULL %d %d\n", size(vehrs_convhull_cw,1), size(vehrs_convhull_cw,2));
%     for i = 1:size(vehrs_convhull_cw, 1)
%         for j = 1:size(vehrs_convhull_cw, 2)
%             fprintf(fid, float_fspec + " ", vehrs_convhull_cw(i, j));
%         end
%         fprintf(fid, "\n");
%     end
    for i = 1:length(vehrs_save)
        print_zonotope(fid, float_fspec, prefix, vehrs_save(i), i)
    end
end

function min_idx = find_t_eval_idx_c_style(vehrs_save)
    % Find the closest t value in the FRS to the desired time, 
    % and return the index, but 0-indexed
    t_desired = 1.5;
    min_found = inf;
    min_idx = 0;
    for i = 1:length(vehrs_save)
        dist_curr = abs(t_desired - vehrs_save{i}.Z(end,1));
        if dist_curr < min_found
            min_found = dist_curr;
            min_idx = i;
        end
    end
    assert(min_idx > 0);
    min_idx = min_idx - 1;
end

function print_zonotope(fid, float_fspec, manu_type_prefix, zono, zono_idx)
    Z = zono{1}.Z;
    % Get lower and upper bound on time for this zonotope
    [t_lb, t_ub] = get_zono_time_lb_ub(Z);

    % Store center value and then remove
    z_center = Z(:,1);
    Z(:,1) = [];

    % Store slice values and corresponding XYH values
    slc_dims = [7 8 9];
    
    % Slice dimensions depend on type of manuever
    if strcmp(manu_type_prefix, 'AU')
        slc_dims(end+1) = 11;
    else
        slc_dims(end+1) = 12;
    end
    
    % Get column indices of sliceable generators. There should be one
    % per slice dimension.
    slc_idxs = [];
    for slc_dim = slc_dims
        slc_idxs(end+1) = find(Z(slc_dim,:));
    end
    
    % Make sure that we don't store overlapping vectors
    assert(length(unique(slc_idxs)) == length(slc_dims));

    % Store slice generators
    slc_gens = {};
    for i = 1:length(slc_dims)
        slc_dim = slc_dims(i);
        slc_idx = slc_idxs(i);
        slc_gens{end+1} = Z(:,slc_idx);
    end
    
    % Remove slice generators
    Z(:,slc_idxs) = [];

    % Move the uvr slice generators to the end
    for i = 1:3
        Z(:,end+1) = slc_gens{1, i};
    end
    
    % Remove dimensions we don't want to store
    Z(4:end,:) = [];
    
    % Remove generators with xy components near zero
    epsilon_considered_zero = 1e-5;
    to_rem_zero = all(abs(Z(1:2,:)) < epsilon_considered_zero);
    Z(:,to_rem_zero) = [];

    % Print out header
    fprintf(fid, "(ZONO_%d) %d %d\n", zono_idx, size(Z, 1), size(Z, 2));
    % Store XYH (1:3) values for the remaining generators and center
    for c = 1:size(Z,2)
        for r = 1:size(Z, 1)
            fprintf(fid, float_fspec + " ", Z(r, c));
        end
        fprintf(fid, "\n");
    end
    
    % Print center
    center_fspec = "CENTER " + float_fspec + " " + float_fspec + " " ...
                                                 + float_fspec + "\n";
    fprintf(fid, center_fspec, z_center(1), z_center(2), z_center(3));

    fprintf(fid, "SLICE_VALS: %d\n", length(slc_gens));
    % Print slice values
    slc_fspec = "SLC %2d " + float_fspec + " " + float_fspec + " " ...
                           + float_fspec + " " + float_fspec + " " ...
                           + float_fspec + "\n";
    for i = 1:length(slc_dims)
       slc_dim = slc_dims(i);
       slc_gen = slc_gens{i};
       center_val = z_center(slc_dim,1);
       slc_val = slc_gen(slc_dim);
       xyh_val = slc_gen(1:3);
       fprintf(fid, slc_fspec, slc_dim, center_val, slc_val, xyh_val);
    end
    time_rng_fspec = "TIME_RNG: " + float_fspec + " " + float_fspec + "\n";
    fprintf(fid, time_rng_fspec, t_lb, t_ub);
end

function [t_lb, t_ub] = get_zono_time_lb_ub(zono_matrix)
    % TODO make sure that this is computed correctly
    [t_lb, t_ub] = get_min_max_slice_val(zono_matrix, ...
                                         size(zono_matrix, 1), true);
end

function [x_min, x_max] = get_min_max_slice_val(zono_matrix, dimension, allow_multiple_generators)
    % Zono Matrix: R^{Num Dimensions, 1 + (Num Generators)}
    % Dimension: the dimension to get the minimum and maximum sliceable
    % value along
    zono_matrix_reduced = zono_matrix(:, [true, abs(zono_matrix(dimension, 2:end)) > 1.0e-12]);

    % Make sure that we only found one valid sliceable generator
    % Expect two columns remaining (center col + one sliceable generator)
    if ~allow_multiple_generators
        ncols = size(zono_matrix_reduced, 2);
        if ncols > 2
            error('More than one valid sliceable generator found')
        elseif ncols < 2
            error('No sliceable generator found')
        end
    end

    c_d = zono_matrix_reduced(dimension, 1);

    % Take absolute value so that c_d + g_d is always the maximum even if
    % the generator is negative. Taking the sum allows for multiple
    % generators to be combined.
    g_d_abs = sum(abs(zono_matrix_reduced(dimension, 2)));
    
    x_min = c_d - g_d_abs;
    x_max = c_d + g_d_abs;
end

% File Format
% {string: FRS_NAME}
% M_MEGA {int: number of idxu0's}
% 
% MEGA {int: mega_idx}
% ({int: mega_idx}) AU           {int: au_a}
% {
%   veh_RS_save: {int: num_zonos}
%   {
%     zono: {int: dimensions} {int: num generators}
%     { values: double[] }
%   }
% }
% ({int: mega_idx}) AUTB   {int} {int: au_a}
% {values: double[]}
% ({int: mega_idx}) DIR          {int: dir_a} {int: dir_b}
% ({int: mega_idx}) DIRTB  {int} {int: dir_a} {int: dir_b}
% ({int: mega_idx}) LAN          {int: lan_a} {int: lan_b}
% ({int: mega_idx}) LANTB  {int} {int: lan_a} {int: lan_b}
