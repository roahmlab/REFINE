clear, close all, clc


save_log_flag = 0;
saveVideo = 0;


frs_filename = 'C:\Users\darkmoon\Documents\MATLAB\FRS_data\FRS_full_3.26_no_force_small.mat';

episode = dir('C:\Users\darkmoon\Documents\GitHub\RTD_FL_Highway_Simulation\NewSimulation_JLRunHighwayTesting\gpops'); % new run

addpath(genpath('C:\Users\darkmoon\Documents\MATLAB\ford_highway_rtd'));
addpath(genpath('C:\Users\darkmoon\Documents\MATLAB\RTD_FL_Highway_Simulation-master'));

for i = 3:length(episode) % ignore the first 2 episode file
    i
    sim_flzono_ford(frs_filename, episode(i), save_log_flag, saveVideo);
end


addpath(genpath('C:\Users\darkmoon\Documents\MATLAB\ford_highway_rtd'));
addpath(genpath('C:\Users\darkmoon\Documents\MATLAB\RTD_FL_Highway_Simulation-master'));
