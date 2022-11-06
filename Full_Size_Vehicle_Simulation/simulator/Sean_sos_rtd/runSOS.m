clear, close all, clc

% This script is to run simulation on Sean's FORD SOS-RTD using the same
% scenarios as Simon tests for FLZONO and GPOPS.

save_log_flag = 1;
saveVideo = 1;


% load FRS
frs = struct;%most space efficient, just functions, so no truncation involved
frs.frsdir = dir('C:\Users\darkmoon\Documents\MATLAB\FRS_data\deg6');%put in your own directories
frs.frsdirbraking = dir('C:\Users\darkmoon\Documents\MATLAB\FRS_data\deg6braking');

% load episodes
% episode = dir('C:\Users\darkmoon\Downloads\Gpops_result_with_5_stopped_vehicles'); % old version


% episode = dir('C:\Users\darkmoon\Downloads\sim_summary_fl_zono_7.5'); % Simon's run
episode = dir('C:\Users\darkmoon\Documents\GitHub\RTD_FL_Highway_Simulation\NewSimulation_JLRunHighwayTesting\gpops'); % new run
new_SIM = 1;

addpath(genpath('C:\Users\darkmoon\Documents\MATLAB\ford_highway_rtd'));
addpath(genpath('C:\Users\darkmoon\Documents\MATLAB\RTD_FL_Highway_Simulation-master'));


for i = 3:length(episode) % ignore the first 2 episode file
    i
    simSOS(frs, episode(i), save_log_flag, saveVideo,new_SIM);
end

rmpath(genpath('C:\Users\darkmoon\Documents\MATLAB\ford_highway_rtd'));
rmpath(genpath('C:\Users\darkmoon\Documents\MATLAB\RTD_FL_Highway_Simulation-master'));

