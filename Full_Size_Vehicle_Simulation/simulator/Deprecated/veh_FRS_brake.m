%------------- BEGIN CODE --------------
load frs_u=10.mat

load_const

options.R0 = vehRS{end}{1} - [zeros(13,1); tpk]; %initial state for reachability analysis
options.x0 = center(options.R0);
options.tFinal =3.12;% this value does not cause split! 10/amax;

dim = 14;

% set options for reachability analysis:
options.timeStep = 0.005;
options.taylorTerms=5; % number of taylor terms for reachable sets
options.zonotopeOrder= 2; % zonotope order... increase this for more complicated systems.
options.maxError = 1000*ones(dim, 1); % our zonotopes shouldn't be "splitting", so this term doesn't matter for now
options.verbose = 1;
options.uTrans = 0; % we won't be using any inputs, as traj. params specify trajectories
options.U = zonotope([0, 0]);
options.advancedLinErrorComp = 0;
options.tensorOrder = 1;
options.reductionInterval = inf;
options.reductionTechnique = 'girard';

options.tStart = 0;


sys = nonlinearSys(dim, 1, @veh_dyn_brake, options );

BrkRS = reach(sys, options);
tComp = toc;
disp(['computation time of reachable set: ', num2str(tComp)]);

% plot reachable set
%%
figure(1); clf; hold on; axis equal;
slice_value = 0.6;
title('Workspace', 'FontSize', 24);
xlabel('$x$', 'Interpreter', 'latex', 'FontSize', 24);
ylabel('$y$', 'Interpreter', 'latex', 'FontSize', 24);

%% plot lane change;
% for i = 1:10:length(vehRS)
%    p_FRS = plotFilled(vehRS{i}{1}, [1, 2], 'g'); 
%    p_FRS.FaceAlpha = 0.02;
%    p_FRS.EdgeAlpha = 0.4;
% end

hold on;
for i = 1:10:length(vehRS)
p_slice = plotFilled( zonotope_slice( vehRS{i}{1}, 7, slice_value), [1, 2 ], 'b' );
p_slice.FaceAlpha= 0.1;
end
for i = 1:10:length(BrkRS)
p_slice = plotFilled( zonotope_slice( BrkRS{i}{1}, 7, slice_value), [1, 2 ], 'r' );
p_slice.FaceAlpha= 0.1;
end
[T, U,Z] = gaussian_parameterized_traj_with_brake(slice_value,10,10,[],0);
plot(Z(1,:),-Z(2,:)) % reference traj
A_go = highway_cruising_6_state_agent() ;

z_0 = [0;0;0;10;0;0] ;
A_go.reset(z_0) ;
A_go.move(T(end),T,U,Z);
A_go.plot() % real traj
save('frs_u=10+brake.mat','vehRS','BrkRS')
% for i = 1:length(vehRS)
% p_slice = plotFilled( zonotope_slice( vehRS{i}{1}, 3, -0.1), [1, 2 ], 'b' );
% p_slice.FaceAlpha= 0.25;
% end
% for i = 1:length(vehRS)
% p_slice = plotFilled( zonotope_slice( vehRS{i}{1}, 3, 0), [1, 2 ], 'b' );
% p_slice.FaceAlpha= 0.25;
% end
% for i = 1:length(vehRS)
% p_slice = plotFilled( zonotope_slice( vehRS{i}{1}, 3, 0.1), [1, 2 ], 'b' );
% p_slice.FaceAlpha= 0.25;
% end
% for i = 1:length(vehRS)
% p_slice = plotFilled( zonotope_slice( vehRS{i}{1}, 3, 0.2), [1, 2 ], 'b' );
% p_slice.FaceAlpha= 0.25;
% end


% intial set
% params.R0 = zonotope([[0; 0; 0],...
%                       0.05*diag([1, 1, 0])]);
%                   
% % reference trajectory
% params.uTrans = 0;
% 
% % uncertain inputs
% params.U = zonotope([0, 0]);
% 
% % Reachability Settings ---------------------------------------------------
% 
% options.timeStep = 0.01;
% options.taylorTerms = 5;
% options.zonotopeOrder = 200;
% 
% options.alg = 'lin';
% options.tensorOrder = 2;
% 
% 
% % System Dynamics ---------------------------------------------------------
% 
% vehicle = nonlinearSys(dim, 1, @veh_dyn, options);
% 
% 
% % Reachability Analysis ---------------------------------------------------
% 
% tic
% R = reach(vehicle, params, options);
% tComp = toc;
% disp(['computation time of reachable set: ',num2str(tComp)]);
% 
% 
% % Simulation --------------------------------------------------------------
% 
% % simulation settings
% simOpt.points = 60;
% simOpt.fracVert = 0.5;
% simOpt.fracInpVert = 0.5;
% simOpt.inpChanges = 6;
% 
% % random simulation
% simRes = simulateRandom(vehicle, params, simOpt);
% 
% 
% % Visualization -----------------------------------------------------------
% 
% dims = {[1 2],[3 4],[5 6]};
% ref = {[17 18],[19 20],[21 22]};
% 
% for k = 1:length(dims)
%     
%     figure; hold on; box on
%     projDims = dims{k}; projRef = ref{k};
% 
%     % plot reachable sets 
%     plot(R,projDims,'FaceColor',[.8 .8 .8],'EdgeColor','none','Order',3);
%     
%     % plot initial set
%     plot(params.R0,projDims,'w','Filled',true,'EdgeColor','k');
%     
%     % plot simulation results     
%     plot(simRes,projDims);
%     
%     % plot reference trajectory
%     plot(params.u(projRef(1),:),params.u(projRef(2),:),'r','LineWidth',2);
% 
%     % label plot
%     xlabel(['x_{',num2str(projDims(1)),'}']);
%     ylabel(['x_{',num2str(projDims(2)),'}']);
% end
% 
% % example completed
% completed = 1;
% 
% %------------- END OF CODE --------------