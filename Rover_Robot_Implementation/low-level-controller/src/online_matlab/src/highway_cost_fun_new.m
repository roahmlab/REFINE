function [cost, cost_deriv] = highway_cost_fun_new(eval_fun,eval_fun_jacobian,K,x_des)
%K,agent_state, slice_idx,FRS.vehRS_save{FRS.brake_idx(1)}{1},x_des
% c = center(zonotope_slice(FRS, [7;8;9;slice_idx], [agent_state(4);agent_state(5);agent_state(6);K]));
cost = highway_cost_fun(eval_fun(K), x_des);
cost_deriv = eval_fun_jacobian(K, );
%cost = highway_cost_fun(c(1:3),x_des);
end

% eval_fun(ay, u0)
% eval_fun(au, u0)

% eval_fun_jacobian = d/dK eval_fun
