clear, close all, clc

stat = zeros(3,100);
avg_spd = zeros(3,100);
%% gpops
% 1. crash. 3. crash with safety layer. 4. safetly stopped but stuck. 5. goal reached
episode = dir('C:\Users\darkmoon\Documents\GitHub\RTD_FL_Highway_Simulation\NewSimulation_JLRunHighwayTesting\gpops');
cd gpops\
for i = 3:length(episode)
    eps = str2num(episode(i).name(23:end-17));
    stat(1,eps) = str2num(episode(i).name(21));
    data = load([episode(i).folder,'\',episode(i).name]);
    xfinal = data.agent_info.state(1,end);
    if xfinal >= 1010
        old_name = episode(i).name;
        new_name = old_name;
        new_name(21) = '5';
        stat(1,eps) = 5;
        system("rename " + convertCharsToStrings(old_name) + " " +...
            convertCharsToStrings(new_name));
    end

    if stat(1,eps) ~= 5
        avg_spd(1,eps) = inf;
        continue
    end
    data = load([episode(i).folder,'\',episode(i).name]);
    avg_spd(1,eps) = data.agent_info.state(1,end)/data.agent_info.time(end);
    
end
cd ..\
%% flzono
episode = dir('C:\Users\darkmoon\Documents\GitHub\RTD_FL_Highway_Simulation\NewSimulation_JLRunHighwayTesting\flzono');
cd flzono\
for i = 3:length(episode)
    eps = str2num(episode(i).name(23:end-17));
    stat(2,eps) = str2num(episode(i).name(21));
    data = load([episode(i).folder,'\',episode(i).name]);
    xfinal = data.agent_info.state(1,end);
    if xfinal >= 1010
        old_name = episode(i).name;
        new_name = old_name;
        new_name(21) = '5';
        stat(2,eps) = 5;
        system("rename " + convertCharsToStrings(old_name) + " " +...
            convertCharsToStrings(new_name));
    end

    if stat(2,eps) ~= 5
        avg_spd(2,eps) = inf;
        continue
    end
    data = load([episode(i).folder,'\',episode(i).name]);
    avg_spd(2,eps) = data.agent_info.state(1,end)/data.agent_info.time(end);
    
end
cd ..\
%% sos
% 0 ok, 1 collision, 2 overtime in real time, 3 no plan found at step 1, 4 came to a stop
res = dir('C:\Users\darkmoon\Documents\GitHub\RTD_FL_Highway_Simulation\NewSimulation_JLRunHighwayTesting\sos\mat');
for i = 3 : length(res)
    filename = res(i).name;
    stop = str2num(filename(end-4));
    eps = str2num(filename(24:end-11));
    stat(3,eps) = stop;

    if stop ~= 0
        avg_spd(3,eps) = inf;
        continue
    end
    data = load([res(i).folder,'\',res(i).name]);
    avg_spd(3,eps) = data.save_struct.Vehicle_state{1}(1,end)/data.save_struct.Vehicle_time{1}(1,end);
end





% flzono: 18 49 63. prefer 18, first 300m looks great
% flzono sos stop, gpops out: 50, 83 is better
% all great: 9







