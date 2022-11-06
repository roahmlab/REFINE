clear, close all, clc

episode = dir('C:\Users\darkmoon\Documents\GitHub\RTD_FL_Highway_Simulation\JL_run_gpops\data2545');

cd data2545\

for i = 3:length(episode)
    data = load([episode(i).folder,'\',episode(i).name]);
    xfinal = data.agent_info.state(1,end);
    if xfinal >= 980
        old_name = episode(i).name;
        new_name = old_name;
        new_name(13) = '5';
        system("rename " + convertCharsToStrings(old_name) + " " +...
            convertCharsToStrings(new_name));
    end
end

