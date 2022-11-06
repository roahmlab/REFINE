%[currpath, ~, ~] = fileparts(matlab.desktop.editor.getActiveFilename);
first_run = true;
if first_run
    rosshutdown();
    [currpath, ~, ~] = fileparts(mfilename('fullpath'));
    fpath = append(currpath, '/../../');
    rosgenmsg(fpath)
    addpath('/home/rover/src/rover-rtd-su21/src/matlab_msg_gen_ros1/glnxa64/install/m')
    clear classes
    rehash toolboxcache    
    rosinit('http://192.168.1.7:11311')
end
if ~exist('frs', 'var') || ~exist('frs_low', 'var')
    disp('Loading FRS Main...')
    frs_low_filename = 'FRS_Rover_14-Sep-2021_low_spd.mat';
    frs_filename = 'FRS_Rover_28-Sep-2021_no_ini_err.mat';
    
    %frs_filename = 'FRS_full_3.10_no_force.mat'
    frs = load(frs_filename);
    frs_low = load(frs_low_filename);
    frs_dir_info = load('dir_change_Ay_info.mat');
    frs_lan_info = load('lane_change_Ay_info.mat');
    disp('Loaded FRS')    
end

if ~exist('gpc', 'var')
    disp('Construcing GPC...')
    gpc = GenParamClass(frs, frs_low, frs_dir_info, frs_lan_info);
    disp('Constructed GPC')
end

%gpc.gen_params()
gen_param_trigger_sub_ = rossubscriber('/gen_param_trigger', 'std_msgs/Bool', @gpc.gen_param_trigger);
gen_param_update_state_sub_ = rossubscriber('/state_out/rover_debug_state_out', 'rover_control_msgs/RoverDebugStateStamped', @gpc.update_state);
gen_param_update_path_sub = rossubscriber('/move_base/GlobalPlanner/plan', 'nav_msgs/Path', @gpc.callback_path);
gen_param_update_obs = rossubscriber('/zonotope_visualization', 'jsk_recognition_msgs/PolygonArray', @gpc.callback_obstacle);
%gpc.gen_params();
%sub = rossubscriber('/state_out/rover_debug_state_out', 'rover_control_msgs/RoverDebugStateStamped', @sub_to_state_update);
% verbose level for printing