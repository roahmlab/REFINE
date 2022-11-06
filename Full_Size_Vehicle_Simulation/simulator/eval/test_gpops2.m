%problem setup
% we have 3 waypoints, we want to find what to output to the car:
% w_cmd(wheel speed), delta(steering angle) given inputs
% world_info(traffic and static obstacles), agent_state(7 state), waypts(),
% dynamics(as differential equation), with path constraint to not hit
% anything
clear;
tfmin = 8; tmax = 8; t0 = 0;

x0 = 0; y0 = 0; h0 = 0; u0 = 15;v0 = 0; r0 = 0; w0 = 15;

bounds.phase.initialtime.lower = t0;
bounds.phase.initialtime.upper = t0;
bounds.phase.finaltime.lower = tfmin;
bounds.phase.finaltime.upper = tmax;


bounds.phase.initialstate.lower = [x0,y0,h0,u0,v0,r0,w0];
bounds.phase.initialstate.upper = [x0,y0,h0,u0,v0,r0,w0];
bounds.phase.state.lower = [-inf,-inf, -2, 6,-4,-4,6];
bounds.phase.state.upper = [inf,  inf,  2,20, 4, 4,20];
bounds.phase.finalstate.lower =  [-inf,-inf, -2, 6,-4,-4,6];
bounds.phase.finalstate.upper = [inf,  inf,  2,20, 4, 4,20];
%first control is steering, second is w_cmd
bounds.phase.control.lower = [-0.4,6];
bounds.phase.control.upper = [ 0.4,20];

bounds.phase.path.lower = [0   0];
bounds.phase.path.upper = [inf  inf] ;

bounds.parameter.lower = [120 0];
bounds.parameter.upper = [120 0];


initialguess.phase.time      = [0; 8];
guess_initial_state = bounds.phase.initialstate.lower;
guess_final_state = [x0+u0*tmax y0 h0 u0 v0 r0 w0];
initialguess.phase.state     = [guess_initial_state ; guess_final_state];
initialguess.phase.control   = [0.1 15; 0.1 15];
initialguess.parameter = [120 0];



mesh.method          = 'hp-LiuRao-Legendre';
mesh.tolerance       = 1e-3;
mesh.maxiterations   = 10;
mesh.colpointsmin    = 2;
mesh.colpointsmax    = 14;
mesh.phase.colpoints = 4*ones(1,10);
mesh.phase.fraction  = 0.1*ones(1,10);



setup.name                           = 'Car_Highway';
setup.functions.continuous           = @car_highway_dynamics_gpops;
setup.functions.endpoint             = @car_highway_endpoint_gpops;
setup.displaylevel                   = 2;
setup.bounds                         = bounds;
setup.guess                          = initialguess;
setup.mesh                           = mesh;
setup.nlp.solver                     = 'ipopt';
setup.nlp.snoptoptions.tolerance     = 1e-6;
setup.nlp.snoptoptions.maxiterations = 20000;
setup.nlp.ipoptoptions.linear_solver = 'ma57';
setup.nlp.ipoptoptions.tolerance     = 1e-3;
setup.derivatives.supplier           = 'adigator';
setup.derivatives.derivativelevel    = 'second';
setup.method                         = 'RPM-Differentiation';

tic
output = gpops2(setup);
toc

figure(1);clf;
subplot(3,1,1);hold on;
plot(output.result.solution.phase.state(:,1),output.result.solution.phase.state(:,2));
viscircles([40,1],3)
viscircles([80,-1],3)
axis equal
subplot(3,1,2)
plot(output.result.solution.phase.time,output.result.solution.phase.state(:,4));
legend('spd m/s')
subplot(3,1,3);
plot(output.result.solution.phase.time,output.result.solution.phase.control(:,1));
yyaxis right
plot(output.result.solution.phase.time,output.result.solution.phase.control(:,2));
legend('steering angle','w cmd')

