save_struct = load("sim_summary_5-1_11-32-07.902.mat");
% save_struct = save_struct.save_struct;
ego_x = save_struct.Vehicle_state{1}(1,:)';
ego_x(ego_x>1000) = 1000;
ego_y = save_struct.Vehicle_state{1}(2,:)';
ego_h = save_struct.Vehicle_state{1}(3,:)';
egotimeVec = save_struct.Vehicle_time{1}';
refPosesX = [egotimeVec, ego_x];
refPosesY = [egotimeVec, ego_y];
refPosesH = [egotimeVec, rad2deg(ego_h)];

traffic1x = save_struct.Vehicle_state{2}(1,:)';
traffic1x(traffic1x>1000) = 1000;
traffic1y = save_struct.Vehicle_state{2}(2,:)';
traffic1h = save_struct.Vehicle_state{2}(3,:)';
traffic1time = save_struct.Vehicle_time{2}';
refPosesX1 = [traffic1time, traffic1x];
refPosesY1 = [traffic1time, traffic1y];
refPosesH1 = [traffic1time, rad2deg(traffic1h)];

traffic2x = save_struct.Vehicle_state{3}(1,:)';
traffic2x(traffic2x>1000) = 1000;
traffic2y = save_struct.Vehicle_state{3}(2,:)';
traffic2h = save_struct.Vehicle_state{3}(3,:)';
traffic2time = save_struct.Vehicle_time{3}';
refPosesX2 = [traffic2time, traffic2x];
refPosesY2 = [traffic2time, traffic2y];
refPosesH2 = [traffic2time, rad2deg(traffic2h)];

traffic3x = save_struct.Vehicle_state{4}(1,:)';
traffic3x(traffic3x>1000) = 1000;
traffic3y = save_struct.Vehicle_state{4}(2,:)';
traffic3h = save_struct.Vehicle_state{4}(3,:)';
traffic3time = save_struct.Vehicle_time{4}';
refPosesX3 = [traffic3time, traffic3x];
refPosesY3 = [traffic3time, traffic3y];
refPosesH3 = [traffic3time, rad2deg(traffic3h)];


traffic4x = repmat(250, length(save_struct.Vehicle_time{4}), 1); 
traffic4y = repmat(-2, length(save_struct.Vehicle_time{4}), 1)
traffic4h = repmat(deg2rad(-10),length(save_struct.Vehicle_time{4}),1);
traffic4time = save_struct.Vehicle_time{4}';
refPosesX4 = [traffic4time, traffic4x];
refPosesY4 = [traffic4time, traffic4y];
refPosesH4 = [traffic4time, rad2deg(traffic4h)];

light_control_sig = repmat( ones(1,6),length(save_struct.Vehicle_time{4}), 1); light_control_sig(:,4) =0;
light_control_sig = [traffic4time, light_control_sig];

set_param(gcs, 'StopTime', num2str(egotimeVec(end)));
