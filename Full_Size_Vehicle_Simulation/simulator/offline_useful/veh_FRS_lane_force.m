% Testing using a FRS and its uncertainty to estimate the forces
% in the tires in a corner and thus apply force not exceeding the max
% tracking of the road.


%1 lane change manuver
%  brake after lane change until split
%  brake from 1-0 m/s

%2 speed change manuver
%  brake after speed change until split
%  brake from 1-0 m/s

%save index of of shortened zonotope series as brake_idx, low_idx
%save speed center and generator at braking idx, and low idx 
% clear;
% for t0 = 2.25:0.75:6-0.75
clear;
dim = 13;

options.tensorParallel = 0;%turn this to 1 if want to use parallel not working very well. 
% set options for reachability analysis:
options.taylorTerms=15; % number of taylor terms for reachable sets
options.zonotopeOrder= 100; % zonotope order... increase this for more complicated systems.
options.maxError = 1e100*ones(dim, 1); % our zonotopes shouldn't be "splitting", so this term doesn't matter for now
options.verbose = 1;
options.uTrans = 0; % we won't be using any inputs, as traj. params specify trajectories
options.U = zonotope([0, 0]);
options.advancedLinErrorComp = 0;
options.tensorOrder = 1;
options.errorOrder = 50;
options.reductionInterval = inf;
options.reductionTechnique = 'girard';

options.tStart = 0;
options.tFinal = 1;
options.reachabilitySteps=1;
options.timeStep=options.tFinal/options.reachabilitySteps;
load lane_change_t0=0.75_u=25_Ay=-0.027891.mat;
% peak_idx = FRS.brake_idx(1);
t_idx = 20;

%in2 states: x, y, psi, u, v, r,u0,v0, r0,t0 ,Au,   Ay,   t
%            1  2  3    4  5  6  7  8  9  10  11    12    13
% vehicle parameters
load_const;
u0 = 25;
Au = u0;
t0 = 0.75
del_y =-0.027891;
t = linspace(0, tpk+Au/amax);
syms_flag = 0;
[T_go,U_go,Z_go] = gaussian_T_parameterized_traj_with_brake(t0,del_y,Au,u0,t,syms_flag,1);
A_go = highway_cruising_6_state_agent() ;
v0= 0
r0= 0

figure(1);%clf;hold on; axis equal;
z_0 = [0;0;0;u0;v0;r0] ;
A_go.reset(z_0) ;
A_go.move(T_go(end),T_go,U_go) ;
plot(Z_go(1,:), Z_go(2,:));

A_go.move(tpk,T_go,U_go) ; % this may not be good


X = A_go.state';
T = A_go.time';

%get tire forces
[~,Fxf,Fyf,Fyr,Fyrn,delta] = cellfun(@(t,x)  A_go.dynamics(t,x.',T_go,U_go,Z_go), num2cell(T), num2cell(X,[2]),'uni',0);
A_go.plot();
%in2 states: x, y, psi, u, v, r,u0,v0, r0,t0 ,Au,   Ay,   t
%            1  2  3    4  5  6  7  8  9  10  11    12    13
figure(2);%clf; hold on;
figure(3);%clf; hold on;xlabel('Fyf');ylabel('delta');
plot(cell2mat(Fyf),cell2mat(delta));
for i = 1:length(vehRS_save) %change to 65 if only want to plot to peak
    zonoslice = zonotope_slice(vehRS_save{i}{1}, [7;8;9;12], [u0;v0;r0;del_y]);
    zonoslice = deleteAligned(zonoslice);
    % try force
    options.x0 = center(vehRS_save{i}{1});
    options.R0 = deleteAligned(vehRS_save{i}{1});
    sysDisc = nonlinearSysDT(dim,1,@dyn_y_change_forces,options);
    R = reach(sysDisc,options);
    force_slice = zonotope_slice(R{2}, [7;8;9;12], [u0;v0;r0;del_y]);
    figure(3);
    plot(project(force_slice,[2,3]));
    figure(2);
    p_FRS = plotFilled(zonoslice, [1, 2], 'r');
    p_FRS.FaceAlpha = .05;
    p_FRS.EdgeAlpha = 0.01;
    
    ft = deleteAligned(project(vehRS_save{i}{1},3));
    headingc = center(ft);
    headingg = abs(generators(ft));
    gen = [2.4 1];
    [len , width]=get_footprint_gen(gen,headingg);% make the footprint wiggle to get largest footprint
    h =  headingc;
    ego_gen = [[cos(h)*len; sin(h)*len], [sin(-h)*width; cos(-h)*width]];
    gen = zeros(dim,2);gen(1:2,1:2) = ego_gen;
    ftprint_zono  = zonotope([zeros(dim,1), gen]);
    slicewithft = zonoslice+ftprint_zono;
    p3 = plotFilled(slicewithft, [1, 2], 'g');
    p3.FaceAlpha = 0.01;
end
axis equal
xlim([-5,130])


yline(2,'LineWidth',2)
yline(6,'LineWidth',2)
yline(-2,'LineWidth',2)
drawnow


