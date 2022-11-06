clear;
files = dir('./*.mat');
avg_v_arr = [];
for i=1:length(files)
    files(i).name
     if contains(files(i).name,'sim_summary_4-50_00-57-00.945')
        display(files(i).name)
        summary = load (["./"+files(i).name]);
        x = summary.agent_info.state(1,:);y = summary.agent_info.state(2,:);
        d = sum(hypot(diff(x), diff(y))); 
        avg_v =d/ summary.agent_info.time(end);
        avg_v_arr = [avg_v_arr avg_v];
     end
end
% save('FRS_full_lane_dir_spd_large_forces.mat','M_mega')
%%
mean(avg_v_arr)
std(avg_v_arr)