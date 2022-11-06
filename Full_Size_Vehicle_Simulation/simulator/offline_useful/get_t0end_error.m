perr = load('lateral_prediction_error.mat');
%always use lat error since lat eror always more than lon
[~,error_idx] = min(abs(perr.error_time_vector - t0_dt));
lon_err_gen = max([perr.lon_err_dir_max; perr.lon_err_lan_max; zeros(size(perr.lon_err_lan_max))],[],1); %min error not included here since we can just keep min at 0, still overapproximating
lon_err_one_half_sec = lon_err_gen(error_idx);
lat_err_gen = max([perr.lat_err_dir_max; perr.lat_err_lan_max ;abs(perr.lat_err_dir_min); abs(perr.lat_err_lan_min)],[],1);
lat_err_one_half_sec = lat_err_gen(error_idx);
head_err_gen = deg2rad(max([perr.head_err_dir_max; perr.head_err_lan_max ;abs(perr.head_err_dir_min); abs(perr.head_err_lan_min)],[],1));
head_err_one_half_sec = head_err_gen(error_idx);