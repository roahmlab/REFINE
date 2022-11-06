% clear; clf;
%Viz code
run_zono = 1;% 0 for sos, 1 for fl zono, 2 for gpops

if run_zono
    frs_filename = 'FRS_Rover_22-Jan-2022_no_force.mat' ;
    if ~exist('frs','var')
        disp('Loading frs')
        frs = load(frs_filename) ;
    else
        disp('table already loaded') ;
    end
end
%%
%Some heading changes, so they dont face weird directions, can use
%quiver(x,y,cos(h), sin(h)),'AutoScale_Factor') to plot waypoints. 
% sim_data = sim_data.save_struct;
plot_sim_flag = 1;
plot_AH_flag = 1;
AH_debug_flag = 1;

%% set up required objects for simulation
% agent -0.7 2m 6m 10m 0.7m
% bounds = [0,100,-0.7,30] ;
lanewidth = 3.7;
bounds = [0,1000,-lanewidth/2-1,lanewidth/2*5+1] ;
goal_radius = 6;
world_buffer = 1 ; % this is used to make start and goal locations that are not too close to the robot or boundary
HLP_buffer = 0.1;
RTD_HLP_grow_tree_mode ='seed';
% planner
buffer = 0.3 ; % [m] distance used to buffer obstacles for planning, must be larger than the agent's footprint
t_move =3;
t_plan =3;
t_failsafe_move = 3;
Ts = t_move;
Tf = Ts*150;

% verbose level for printing
verbose_level = 0;

% automated from here
A3 = highway_cruising_10_state_agent('use_rover_controller',false);
A3.integrator_type= 'ode45';
A3. desired_initial_condition = [50;0; 0; 15;0;0;5;0;0;0];
A3.plot_footprint_color = [0,0,0]/256;
A3.plot_footprint_edge_color =[0,0,0]/256;
% A3. desired_initial_condition(4) = 6;
% A3. desired_initial_condition(7) = 6;
% A3.desired_initial_condition=[-5;0.0;0;1.0;0.0;0.0;0;0;1];
% A3. desired_initial_condition(1) = A3. desired_initial_condition(1);
% A3. desired_initial_condition(2) = -38;
% A3. desired_initial_condition(3) = 0;
% 
% A3. desired_initial_condition(4) = 0;
% A3. desired_initial_condition(7) = 0;
W = dynamic_car_world('bounds',bounds,'buffer',world_buffer,...
    'verbose',verbose_level,'goal_radius',goal_radius,'num_cars',10,'t_move_and_failsafe', t_move+t_failsafe_move) ;%1 is ego , 2 means 1 obs

HLP = highway_HLP;%RRT_star_HLP('timeout',0.5,'bounds',bounds,'grow_tree_mode',RTD_HLP_grow_tree_mode,'verbose',verbose_level);
if run_zono 
    AH = highwayAgentHelper(A3,frs,HLP,'t_plan',t_plan,'t_move',t_move,'t_failsafe_move',t_failsafe_move,...
        'eps',0.001,'verbose',verbose_level,'plot_flag',AH_debug_flag);
else
    AH = [];
end
% if run_zono == 0
%     frs =[]
%     AH = highwaySOSAgentHelper(A3,frs,HLP,'t_plan',t_plan,'t_move',t_move,'t_failsafe_move',t_failsafe_move,'eps',0.001,'verbose',verbose_level,'plot_flag',AH_debug_flag);
% end
S = rlsimulator(AH,W,'plot_sim_flag',plot_sim_flag, 'safety_layer','A','plot_AH_flag',plot_AH_flag);
% automatic mode: A: using a High level planner to generate waypoints, use a
% optimization program(sample) to select best parameter
% (Broke) replacement mode: Z: edit the user selected parameter to something safe
% No safe mode: N: no safe stuff..
AH.S = S;
AH.pause_flag = 0;
random_inputs = true; %true for user inputs using arrow keys, false for random inputs
%%
S.eval = 0; %turn on evaluation so summary will be saved
% to the episode number
S.epscur = 1;
sim_file = load('C:\Users\darkmoon\Documents\GitHub\RTD_FL_Highway_Simulation\NewSimulation_JLRunHighwayTesting\flzono\sim_summary_method0_3-62_11-18-46.258.mat');
if run_zono > 0
    A3.time = sim_file.agent_info.time;
    A3.state = sim_file.agent_info.state;
    W.envCars = sim_file.envCars;
else
    A3.time = sim_file.save_struct.Vehicle_time{1};
    A3.state = sim_file.save_struct.Vehicle_state{1};
    env_car_helper = load('GPOPSsim_summary_3-84_01-35-31.073.mat');
    W.envCars = env_car_helper.envCars;
end
% S.plot();



f2 = figure(2);
fh = figure(1);
if run_zono > 0
    plan_time_arr = sim_file.hist_info.time_hist;
else
    
end
writerObj = VideoWriter(strcat('video_',num2str(S.epscur),"_",datestr(now,'HH-MM-SS.FFF')),'Uncompressed AVI');
dt = 0.2;
writerObj.FrameRate = 1/dt; 
open(writerObj);
for plot_t = linspace(3,A3.time(end),40)
    plot_t
    figure(1);
    set(gcf,'position',[50,50,1400,200])
    clf;hold on;
    set(gca,'Color',[150,150,150]/256);
    [~,time_idx]=min(abs(A3.time - plot_t));
    state_at_time = A3.state(:,time_idx);
    if run_zono
        agent_info_dummy = sim_file.agent_info;
        agent_info_dummy.time = agent_info_dummy.time(1:time_idx);
    else
        agent_info_dummy = env_car_helper.agent_info;
        agent_info.time = A3.time (1:time_idx);
        agent_info.state = A3.state;
    end
    W.obstacles_seen  = W.obstacles_old;
    W.plot(agent_info_dummy)
    yline(-1.9,'w--','LineWidth',2); 
    yline(1.9,'w--','LineWidth',2);
    yline(1.9*3,'w--','LineWidth',2);
    yline(1.9*5,'w--','LineWidth',2);
    if run_zono 
    plan_time_idx = find(plan_time_arr  - plot_t <= 0, 1,'last');
    A3.plot_at_time(A3.time(time_idx));

    AH.plot_selected_parameter_FRS(sim_file.hist_info.K_hist(plan_time_idx),...
                                    sim_file.hist_info.type_manu_hist(plan_time_idx),...
                                     sim_file.hist_info.FRS_hist{plan_time_idx},...
                                     sim_file.hist_info.mirror_hist(plan_time_idx),...
                                     sim_file.hist_info.state_hist(:,plan_time_idx),sim_file.hist_info.mirror_hist(plan_time_idx)*(-2)+1 );
    else % sos plot
        dist = [];
        for i = 1: length(sim_file.save_struct.FRS)
            dist =[dist sim_file.save_struct.FRS(i).ego_state(1) - state_at_time(1)];
        end
        [~,frs_idx] = min(abs(dist));
        frs_plot = sim_file.save_struct.FRS(frs_idx).FRS;
        for frs_item_idx = 1:length(frs_plot)
        plot(frs_plot{frs_item_idx}{1}(1,:),frs_plot{frs_item_idx}{1}(2,:),'g-');
        end
        A3.plot_at_time(A3.time(time_idx));

    end
     if run_zono == 1
        world_info = W.get_world_info(agent_info_dummy);
        waypt = HLP.get_waypoint(world_info,sim_file.hist_info.state_hist(:,plan_time_idx));
    elseif run_zono == 2
        
        waypt_idx = min(floor(plot_t/sim_file.t_move)+1,length(sim_file.hist_info.wp_hist) );
        waypt = sim_file.hist_info.wp_hist{waypt_idx};
     else %run_zono == 0
         waypt =  sim_file.save_struct.FRS(frs_idx).waypoint;
    end

    figure(1);
    scatter(waypt(1),waypt(2),180,'k','x','LineWidth',3);
    ylim([-4.5627 11.9627]);

    xlim([state_at_time(1)-100,state_at_time(1)+10]);%[state_at_time(1)-10,state_at_time(1)+100]);
    %     fh.
    axis equal
    drawnow
    figure(2); clf;
    set(gcf,'position',[50,50,250,1000])
    a = subplot(1,1,1);
    set(gca,'Color',[150,150,150]/256);

    copyobj(fh.Children.Children,a);
    for i = 1:length(a.Children) 
        if ~isa(a.Children(i),'matlab.graphics.chart.decoration.ConstantLine')
            x_temp = a.Children(i).XData;
            a.Children(i).XData = a.Children(i).YData;
            a.Children(i).YData = x_temp;
        else
            a.Children(i).Value = -1000;
        end
    end
    xline(-1.9,'w--','LineWidth',2); 
    xline(1.9,'w--','LineWidth',2);
    xline(1.9*3,'w--','LineWidth',2);
    xline(1.9*5,'w--','LineWidth',2);
    set(a, 'XDir','reverse')
    ttt = text(12,state_at_time(1)+93,"Speed = "+num2str(state_at_time(4),'%.1f')+"m/s",'FontSize',12,'FontWeight','bold');
    uistack(ttt, 'top')
    
    ylim([state_at_time(1)-10,state_at_time(1)+100])
        xlim([-4.5627 11.9627]);

%     xlim([state_at_time(2)-10
    axis equal
    frame = getframe(gcf);
    writeVideo(writerObj,frame);
%     
    
% A3.animate()
end
close(writerObj);

%exportgraphics(gcf,'sim_pdf.pdf','Resolution',300)


