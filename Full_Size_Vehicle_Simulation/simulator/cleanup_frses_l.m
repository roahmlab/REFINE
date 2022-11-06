if ~exist('frs_full_data', 'var')
    disp('Loading FRS...');
    tic;
    frs_full_data = load('/home/roahm/Downloads/FRS_Rover_30-Aug-2022/FRS_Rover_30-Aug-2022_no_force.mat');
    frs_copy = frs_full_data;
    toc;
    disp('Loaded FRS!')
end
frs_full_data = frs_copy;
u_minmax = [];

manu_types = {'Au', 'dir', 'lan'};

first_nonempty_mega_idx = 1;
for mega_idx = 1:length(frs_full_data.M_mega)
    mega = frs_full_data.M_mega{mega_idx};

    % (number of manuever types) x 1 boolean vector to store whether each
    % manuever type was empty at this mega index
    manu_is_empty = false(length(manu_types), 1);

    for manu_idx = 1:length(manu_types)
        manu_name = manu_types{manu_idx};
        frs_set = mega(manu_name);
        empty_frses = cellfun(@isempty, frs_set);
        manu_is_empty(manu_idx, 1) = isempty(frs_set) || all(cellfun(@isempty, frs_set), 'all');
    end
    
    % If all were empty, increment the minimum non-empty index
    if all(manu_is_empty, 'all')
        first_nonempty_mega_idx = mega_idx + 1;
    else
        break;
    end
end

for i = 1:first_nonempty_mega_idx
    frs_full_data.M_mega(i) = [];
end
first_nonempty_mega_idx = 1;

if first_nonempty_mega_idx > length(frs_full_data.M_mega)
    error('No nonempty megas found')
end



for mega_idx = 1:length(frs_full_data.M_mega)
    mega = frs_full_data.M_mega{mega_idx};
    for manu_idx = 1:length(manu_types)
        manu_name = manu_types{manu_idx};
        frs_set = mega(manu_name);
        empty_frses = cellfun(@isempty, frs_set);
        empty_cols = all(empty_frses,1);

        if any(empty_frses,1) ~= empty_cols
            % The usual pattern is full columns of missing FRSes, in which
            % case we just remove the columns. If there are missing FRSes
            % only in specific rows, something weird is going on.
            error('Some columns have missing FRSes');
        end

        % If there are some empty columns
        if max(empty_cols) == true
            % If there are more than one jumps in emptiness, e.g. 
            % empty_cols == [0 1 1 0 0], something weird is happening and 
            % needs further investigation
            if nnz(diff(empty_cols)) ~= 1
                error('Empty columns are non-contiguous, investigate more');
            end

            % If there are some empty columns but the last column is not
            % part of the empty set, something weird is happening and 
            % needs further investigation.
            if empty_cols(end) ~= true
                error('Empty columns are not at the end, investigate more');
            end
        end
        frs_set = frs_set(:, ~empty_cols);
        mega(manu_name) = frs_set;
    end
end

u0_minmaxes = cell(length(manu_types), 1);
validate_all_zonotopes = true;
validate_all_frses = true;
first_nonempty_mega_idx = 1;
tic;
for mega_idx = first_nonempty_mega_idx:length(frs_full_data.M_mega)
    mega = frs_full_data.M_mega{mega_idx};
    for manu_type_idx = 1:length(manu_types)
        manu_type = manu_types{manu_type_idx};
        frs_set = mega(manu_type);
        [u0_min_orig, u0_max_orig] = get_min_max_slice_val(frs_set{1}.vehRS_save{1}.Z, 7, false);
        
        frs_idxes_to_check = [];

        % If we want to validate every FRS, make sure we iterate over each
        % FRS
        if validate_all_frses
            frs_idxes_to_check = 1:numel(frs_set);
        end

        % If we want to validate every zonotope, make sure that we are
        % going to iterate over at least one FRS
        if validate_all_zonotopes && isempty(frs_idxes_to_check)
            frs_idxes_to_check = 1;
        end

        for frs_idx = frs_idxes_to_check
            frs = frs_set{frs_idx};
%             disp(['frs idx: ', num2str(frs_idx)])

            zono_idxes_to_check = 1;
            if validate_all_zonotopes
                zono_idxes_to_check = 1:length(frs.vehRS_save);
            end

            t_vals = zeros(2,length(zono_idxes_to_check));
            for zono_idx = zono_idxes_to_check
                [t_lb, t_ub] = get_zono_time_lb_ub(frs.vehRS_save{zono_idx}.Z);
                t_vals(:,zono_idx) = [t_lb; t_ub];
                [u0_min, u0_max] = get_min_max_slice_val(frs.vehRS_save{zono_idx}.Z, 7, false);
                if u0_min ~= u0_min_orig || u0_max ~= u0_max_orig
                    new_u0_str = ['[',  num2str(u0_min), ', ', num2str(u0_max), ']'];
                    orig_u0_str = ['[',  num2str(u0_min_orig), ', ', num2str(u0_max_orig), ']'];
                    err_str_prefix = ['[', manu_type, '] [Mega ', num2str(mega_idx), '] [Zono ', num2str(zono_idx), '] '];
                    error([err_str_prefix, 'Different u0 min/max ', new_u0_str, ' than original ', orig_u0_str])
                end
            end
        end
        u0_minmaxes{manu_type_idx, 1} = [u0_minmaxes{manu_type_idx}, ...
                                         [u0_min_orig; u0_max_orig]];
    end
end
toc

for mega_idx = 2:length(manu_types)
    if ~isequal(u0_minmaxes{manu_type_idx-1, 1}, u0_minmaxes{manu_type_idx, 1})
        error('u0 minimum and maximumx differ between manuever types in the same mega');
    end
end

% Since we have verified that all manuevers have the same u0 min/maxes, we 
% can now arbitrarily choose one set of them
u0_minmaxes_individ = u0_minmaxes{1,1};
u0_is_equally_spaced = abs(diff(diff(u0_minmaxes_individ, 1, 2), 1, 2)) < 1.0e-19;
if ~u0_is_equally_spaced
    error('u0 values are not evenly spaced')
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