function [cost,dcost] = highway_fmincon_cost_fun(K,agent_state,slice_idx,FRS,x_des)
%K,agent_state, slice_idx,FRS.vehRS_save{FRS.brake_idx(1)}{1},x_des
[zono_slice, betas, slice_generators,slice_G_full]=zonotope_slice_cost_fun(FRS, [7;8;9;slice_idx], [agent_state(4);agent_state(5);agent_state(6);K]);
future_pose = center(zono_slice);
% cost = highway_cost_fun(future_pose(1:3),x_des);
delta = future_pose(1:2)-x_des(1:2);
cost = sum(delta.^2);
dcost = sum(2.*(delta).*slice_G_full(1:2,end)/slice_generators(4,4));
% beta_1 = 
% dcost = 0;
end
