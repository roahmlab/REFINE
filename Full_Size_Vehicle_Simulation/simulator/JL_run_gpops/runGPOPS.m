clear, close all, clc


episode = dir('C:\Users\darkmoon\Downloads\sim_summary_fl_zono_7.5');


for i = 3+2 : length(episode) % ignore the first 2 episode file
    simGPOPS(episode(i));
end
