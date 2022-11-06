save_struct = load("sim_summary_5-24_14-21-39.560.mat");
ego_x = save_struct.agent_info.state(1,:)';
ego_x(ego_x>1000) = 1000;
ego_y = save_struct.agent_info.state(2,:)';
ego_h = save_struct.agent_info.state(3,:)';
egotimeVec = save_struct.agent_info.time';
refPosesX = [egotimeVec, ego_x];
refPosesY = [egotimeVec, ego_y];
refPosesH = [egotimeVec, rad2deg(ego_h)];

[~,sort_idx]=sort(save_struct.envCars(:,1));

save_struct.envCars = save_struct.envCars(sort_idx,:);


traffic1x = save_struct.envCars(2,1)+egotimeVec*save_struct.envCars(2,2);
traffic1x(traffic1x>1000) = 1000;
traffic1y = ones(size(egotimeVec))*save_struct.envCars(2,3);
traffic1h =  zeros(size(egotimeVec));
traffic1time =egotimeVec;
refPosesX1 = [traffic1time, traffic1x];
refPosesY1 = [traffic1time, traffic1y];
refPosesH1 = [traffic1time, rad2deg(traffic1h)];

traffic2x = save_struct.envCars(3,1)+egotimeVec*save_struct.envCars(3,2);
traffic2x(traffic2x>1000) = 1000;
traffic2y = ones(size(egotimeVec))*save_struct.envCars(3,3);
traffic2h =   zeros(size(egotimeVec));
traffic2time = egotimeVec;
refPosesX2 = [traffic2time, traffic2x];
refPosesY2 = [traffic2time, traffic2y];
refPosesH2 = [traffic2time, rad2deg(traffic2h)];

traffic3x = save_struct.envCars(4,1)+egotimeVec*save_struct.envCars(4,2);
traffic3x(traffic3x>1000) = 1000;
traffic3y = ones(size(egotimeVec))*save_struct.envCars(4,3);
traffic3h = zeros(size(egotimeVec));
traffic3time = egotimeVec;
refPosesX3 = [traffic3time, traffic3x];
refPosesY3 = [traffic3time, traffic3y];
refPosesH3 = [traffic3time, rad2deg(traffic3h)];


% traffic4x = repmat(250, length(save_struct.Vehicle_time{4}), 1); 
% traffic4y = repmat(-2, length(save_struct.Vehicle_time{4}), 1)
% traffic4h = repmat(deg2rad(-10),length(save_struct.Vehicle_time{4}),1);
% traffic4time = save_struct.Vehicle_time{4}';
% refPosesX4 = [traffic4time, traffic4x];
% refPosesY4 = [traffic4time, traffic4y];
% refPosesH4 = [traffic4time, rad2deg(traffic4h)];

% light_control_sig = repmat( ones(1,6),length(save_struct.Vehicle_time{4}), 1); light_control_sig(:,4) =0;
% light_control_sig = [traffic4time, light_control_sig];

set_param(gcs, 'StopTime', num2str(egotimeVec(end)));
