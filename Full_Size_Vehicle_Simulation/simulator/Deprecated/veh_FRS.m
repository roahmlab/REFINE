%1 lane change manuver
%  brake after lane change until split
%  brake from 1-0 m/s

%2 speed change manuver
%  brake after speed change until split
%  brake from 1-0 m/s

%save index of of shortened zonotope series as brake_idx, low_idx
%save speed center and generator at braking idx, and low idx 
clear;
dim = 15;

options.tensorParallel = 0;%turn this to 1 if want to use parallel not working very well. 
% set options for reachability analysis:
options.taylorTerms=15; % number of taylor terms for reachable sets
options.zonotopeOrder= 100; % zonotope order... increase this for more complicated systems.
options.maxError = 10000*ones(dim, 1); % our zonotopes shouldn't be "splitting", so this term doesn't matter for now
options.verbose = 1;
options.uTrans = 0; % we won't be using any inputs, as traj. params specify trajectories
options.U = zonotope([0, 0]);
options.advancedLinErrorComp = 0;
options.tensorOrder = 1;
options.errorOrder = 50;
options.reductionInterval = inf;
options.reductionTechnique = 'girard';

options.tStart = 0;
load_const
% x, y, psi, delu, delv, delr, av0,au0,u0,v0, r0,t0 ,Au, Ay,   t
% 1  2  3    4        5     6  7    8  9  10  11 12  13   14   15
options.x0 = zeros(15,1);
frs_gen_starttic = tic;

load lane_change_Ay_info
for u0 = 16:2:26 % set range of Ay or del_y allowed
    [~,Ay_idx ]=min(abs(u0_vec -u0)); 
    del_y_arr = linspace(2*Ay_vec(Ay_idx),0,9);%    del_y_arr = -0.8:0.2:0;
    del_y_arr = del_y_arr(2:2:end)
    delyspd = 0.5*(del_y_arr(2) - del_y_arr(1));
   
    for del_y = del_y_arr
        
%         brake_idx = [];
%         brake_u = [];
%         brake_gen = [];
        
        u0, del_y
        options.x0 = zeros(15,1);
        options.tFinal = tpk;
        options.timeStep = 0.001;
        options.x0(9) = u0;
        options.x0(13) = u0; %Au = u0
        options.x0(12) = 2; %set t0 = 0.5;
        options.x0(14) = del_y;
        % options.R0 = zonotope( [options.x0, 0.1*diag([1, 1, 2, 2, 0.5, 0, 0])] );
        % options.R0 = zonotope( [options.x0, [0;0;0.2;0;0;0;], [0;0;0;1;0;0;], [0;0;0;0;0.01;0;]] );
        % options.R0 = zonotope( [options.x0, 1e-2*[0;0;0;0;0;1;0]] );\
        delu0 = zeros(15,1); delu0(9) = 1;
        % delAu = zeros(15,1); delAu(12) = 0;
        delAy = zeros(15,1); delAy(14) = delyspd;
        delAu = zeros(15,1); delAu(13) = 1;
        delv0 = zeros(15,1); delv0(10) = 0.05;
        delr0 = zeros(15,1); delr0(11) = 0.05;
        delt0 = zeros(15,1); delt0(12) = 0;
        options.R0 = zonotope( [options.x0, delu0,delAy,delAu,delv0,delr0,delt0]);
        tic;
        sys = nonlinearSys(dim, 1, @dyn_y_change, options);
        vehRS = reach(sys, options);
        figure(1); clf; hold on; axis equal;
        vehRS_save = cell(0,0);
        for i = 1:50:length(vehRS)
            p_FRS = plot(vehRS{i}{1}, [1, 2], 'g');
            %             p_FRS.FaceAlpha = 0.02;
            %             p_FRS.EdgeAlpha = 0.4;
            vehRS_save{end+1} = vehRS{i};
        end
        %% save brake idx
        brake_idx = length(vehRS_save);
        brake_u   = center(project(vehRS_save{end}{1},4)) + center(project(vehRS_save{end}{1},9));
        gen4 = generators(deleteAligned(project(vehRS_save{end}{1},4))); % is empty most of the time
        if ~isempty(gen4) %get the range of braking speed here
            brake_gen = abs(gen4) + abs(generators(deleteAligned(project(vehRS_save{end}{1},9))));
        else 
            brake_gen = 0;
        end
        %% first part of brake
        t_prev = zeros(15,1); t_prev(end)=tpk;%assume it complete the whole time
        options.R0 = vehRS_save{end}{1} - t_prev; %initial state for reachability analysis
        options.x0 = center(options.R0);
        options.tFinal =tbrk;%maybe try adjust this dynamically for when it reaches the low speed. plot model differences.
%         options.verbose = 1;
        %         options.timeStep = 0.001;
        sys = nonlinearSys(dim, 1, @dyn_u_brake, options );
        vehBrk = reach(sys, options);
        
        tComp = toc;
        disp(['computation time of reachable set: ', num2str(tComp)]);
        toc(frs_gen_starttic)
        %% plot reachable set
        
        for i = 1:50:length(vehBrk)
            if size(vehBrk{i},2)~= 1
                break
            end
            p_FRS = plot(vehBrk{i}{1}, [1, 2], 'r');
            %             p_FRS = plot(vehBrk{i}{2}, [1, 2], 'y');
            %             p_FRS.FaceAlpha = 0.02;
            %             p_FRS.EdgeAlpha = 0.4;
            vehRS_save{end+1} = vehBrk{i};
        end
        %%
        brake_idx = [brake_idx length(vehRS_save)];
        brake_u   = [brake_u center(project(vehRS_save{end}{1},4)) + center(project(vehRS_save{end}{1},9))];
        gen4 = generators(deleteAligned(project(vehRS_save{end}{1},4))); % is empty most of the time
        brake_gen = [brake_gen abs(gen4) + abs(generators(deleteAligned(project(vehRS_save{end}{1},9))))];
        %% second part slow brake
        t_cur = center(vehRS_save{end}{1});
        t_cur = t_cur(end);
        options.R0 = vehRS_save{end}{1}; 
        options.x0 = center(options.R0);
        options.tFinal =tbrk-t_cur;
%         options.verbose = 1;
        %         options.timeStep = 0.001;
        sys = nonlinearSys(dim, 1, @dyn_u_slow, options );
        vehBrk = reach(sys, options);
        for i = 1:50:length(vehBrk)
            p_FRS = plot(vehBrk{i}{1}, [1, 2], 'c');
            %             p_FRS = plot(vehBrk{i}{2}, [1, 2], 'y');
            %             p_FRS.FaceAlpha = 0.02;
            %             p_FRS.EdgeAlpha = 0.4;
            vehRS_save{end+1} = vehBrk{i};
        end
        drawnow
        save("lane_change_u="+num2str(u0)+"_Ay="+num2str(del_y)+".mat",'vehRS_save','brake_idx','brake_u','brake_gen');
        
    end
end
%% Now spd change
clear;
dim = 15;
use_proper_braking = 0; %trouble trouble trouble
% set options for reachability analysis:
options.taylorTerms=15; % number of taylor terms for reachable sets
options.zonotopeOrder= 100; % zonotope order... increase this for more complicated systems.
options.maxError = 1000*ones(dim, 1); % our zonotopes shouldn't be "splitting", so this term doesn't matter for now
options.verbose = 0;
options.uTrans = 0; % we won't be using any inputs, as traj. params specify trajectories
options.U = zonotope([0, 0]);
options.advancedLinErrorComp = 0;
options.tensorOrder = 1;
options.reductionInterval = inf;
options.reductionTechnique = 'girard';

options.tStart = 0;
load_const
% options.tFinal = 3.25;

% x, y, psi, delu, delv, delr, av0,au0,u0,v0, r0,Au, Ay,   t
% 1  2  3    4        5     6  7    8  9  10  11 12  13   14
options.x0 = zeros(14,1);

for u0 = 3:2:15
    for Au = 5:2:15
        if abs(u0-Au) > 5 % range of avaliable commanded velocity
            continue;
        end
        u0, Au
        options.x0 = zeros(14,1);
        options.tFinal = tpk;
        options.timeStep = 0.001;
        options.x0(9) = u0;
        options.x0(12) = Au;
        %         options.x0(13) = del_y;
        % options.R0 = zonotope( [options.x0, 0.1*diag([1, 1, 2, 2, 0.5, 0, 0])] );
        % options.R0 = zonotope( [options.x0, [0;0;0.2;0;0;0;], [0;0;0;1;0;0;], [0;0;0;0;0.01;0;]] );
        % options.R0 = zonotope( [options.x0, 1e-2*[0;0;0;0;0;1;0]] );\
        delu0 = zeros(14,1); delu0(9) = 1;
        % delAu = zeros(14,1); delAu(12) = 0;
        delAu = zeros(14,1); delAu(12) = 1;
        
        options.R0 = zonotope( [options.x0, delu0,delAu] );
        
        
        tic;
        sys = nonlinearSys(dim, 1, @dyn_u_change, options );
        
        vehRS = reach(sys, options);
        figure(1); clf; hold on; axis equal;
        vehRS_save = cell(0,0);
        for i = 1:50:length(vehRS)
            p_FRS = plot(vehRS{i}{1}, [1, 2], 'g');
            %             p_FRS.FaceAlpha = 0.02;
            %             p_FRS.EdgeAlpha = 0.4;
            vehRS_save{end+1} = vehRS{i};
        end
        brake_idx = length(vehRS_save);
        brake_u   = center(project(vehRS_save{end}{1},4)) + center(project(vehRS_save{end}{1},9));
        gen4 = generators(deleteAligned(project(vehRS_save{end}{1},4))); % is empty most of the time
        if ~isempty(gen4)
            brake_gen = abs(gen4) + abs(generators(deleteAligned(project(vehRS_save{end}{1},9))));
        else 
            brake_gen = 0;
        end
        %% brake for fast u 
        % This part has problem!!! left limil larger than right limit, why?
        % cora does not split, so u got really small and cause large
        % numbers e.g. u_center = 2.54 u_gen = 2.50
        if use_proper_braking
            t_prev = zeros(14,1); t_prev(end)=tpk;
            options.R0 = (vehRS_save{end}{1} - t_prev); %initial state for reachability analysis
            options.x0 = center(options.R0);
            options.tFinal = tbrk;
            options.verbose = 1;
            % Now needs to brake to 0 so set Au properly
            % Need to set generator? how? options.R0 = zonotope_slice(options.R0, [12], [Au]);
    %         options.R0 = zonotope( [options.x0, delu0,delAy] );
            %         options.timeStep = 0.001;
            sys = nonlinearSys(dim, 1, @dyn_u_brake, options );
            vehBrk = reach(sys, options);
            for i = 1:50:length(vehBrk)
                 if size(vehBrk{i},2)~= 1
                    break
                end
                p_FRS = plot(vehBrk{i}{1}, [1, 2], 'r');
                %             p_FRS = plot(vehBrk{i}{2}, [1, 2], 'y');
                %             p_FRS.FaceAlpha = 0.02;
                %             p_FRS.EdgeAlpha = 0.4;
                vehRS_save{end+1} = vehBrk{i};
                c = center(project(vehBrk{i}{1},4))
                gen4 = generators(deleteAligned(project(vehBrk{i}{1},4)));
                g = abs(gen4) 
            end
            brake_idx = [brake_idx length(vehRS_save)];
            brake_u   = [brake_u center(project(vehRS_save{end}{1},4)) + center(project(vehRS_save{end}{1},9))];
            gen4 = generators(deleteAligned(project(vehRS_save{end}{1},4))); % is empty most of the time
            brake_gen = [brake_gen abs(gen4) + abs(generators(deleteAligned(project(vehRS_save{end}{1},9))))];
            tComp = toc;
            disp(['computation time of reachable set: ', num2str(tComp)]);
        end
        %% brake slow
        
        t_cur = center(vehRS_save{end}{1});
        t_cur = t_cur(end);
        t_cur_z = zeros(14,1); t_cur_z(end)=t_cur;
        options.R0 = vehRS_save{end}{1} - t_cur_z;%initial state for reachability analysis
        options.x0 = center(options.R0);
        if use_proper_braking
            options.tFinal =tbrk - t_cur;
        else
            options.tFinal =tbrk;
        end
%         options.verbose = 1;
        %         options.timeStep = 0.001;
        sys = nonlinearSys(dim, 1, @dyn_u_slow, options );
        vehBrk = reach(sys, options);
        for i = 1:50:length(vehBrk)
            p_FRS = plot(vehBrk{i}{1}, [1, 2], 'c');
            %             p_FRS = plot(vehBrk{i}{2}, [1, 2], 'y');
            %             p_FRS.FaceAlpha = 0.02;
            %             p_FRS.EdgeAlpha = 0.4;
            vehRS_save{end+1} = vehBrk{i};
        end
        
        %%
        drawnow
        save("spd_change_u="+num2str(u0)+"_Au="+num2str(Au)+".mat",'vehRS_save','brake_idx','brake_u','brake_gen');
        
    end
end

%% Inspect FRS
% First load the desired FRS in the FRS folder and run. 
figure(1); clf;hold on;
% for i = 1:length(vehRS_save)
%     p_FRS = plotFilled(vehRS_save{i}{1}, [1, 2], 'g');
%     p_FRS.FaceAlpha = 0.02;
%     p_FRS.EdgeAlpha = 0.4;
%     %         vehRS_save{end+1} = vehRS_save{i};
% end
load("test_5_lane_change_u=16_Ay=-0.12031.mat"); % I only have this one in the repo, genearte your own will take 90 seconds each
load_const;
u0 = 16;
Au = u0;
t0 = 2;
del_y =-0.12;
t = linspace(0, tpk+Au/amax);
syms_flag = 0;
[T_go,U_go,Z_go] = gaussian_T_parameterized_traj_with_brake(t0,del_y,Au,u0,t,syms_flag,1);
A_go = highway_cruising_6_state_agent() ;
v0= 0.049;
r0= 0.049;

z_0 = [0;0;0;u0;v0;r0] ;
A_go.reset(z_0) ;
A_go.move(T_go(end),T_go,U_go) ;
plot(Z_go(1,:), Z_go(2,:));
A_go.plot();
% x, y, psi, delu, delv, delr, av0,au0,u0,v0, r0,t0 ,Au, Ay,   t
% 1  2  3    4        5     6  7    8  9  10  11 12  13   14   15
for i = 1:118%length(vehRS_save) %change to 65 if only want to plot to peak
    zonoslice =zonotope_slice(vehRS_save{i}{1}, [9;10;11;13;14], [u0;v0;r0;Au;del_y]);
    zonoslice = deleteAligned(zonoslice);
    
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
    gen = zeros(15,2);gen(1:2,1:2) = ego_gen;
    ftprint_zono  = zonotope([zeros(15,1), gen]);
    slicewithft = zonoslice+ftprint_zono;
    p3 = plotFilled(slicewithft, [1, 2], 'g');
    p3.FaceAlpha = 0.01;
    if i == 93
        a= 1;
    end
end
axis equal
xlim([-5,130])


yline(2,'LineWidth',2)
yline(6,'LineWidth',2)
yline(-2,'LineWidth',2)
drawnow

return
%% not sure what the following does
figure(1); clf;  subplot(2,1,1);hold on;
for i = 1:length(vehRS_save)
    p_FRS = plotFilled(vehRS_save{i}{1}, [1, 2], 'g');
    p_FRS.FaceAlpha = 0.1;
    p_FRS.EdgeAlpha = 0.4;
    %         vehRS_save{end+1} = vehRS_save{i};
end
load_const;
u0 = 16;
Au = 16;
del_y = 0;
t = linspace(0, tpk+tbrk);
syms_flag = 0;
[T_go,U_go,Z_go] = gaussian_parameterized_traj_with_brake(del_y,Au,u0,t,syms_flag);
A_go = highway_cruising_6_state_agent() ;

z_0 = [0;0;0;u0;0;0] ;
A_go.reset(z_0) ;
A_go.move(T_go(end),T_go,U_go) ;
plot(Z_go(1,:), Z_go(2,:));
A_go.plot();
subplot(2,1,2);hold on;
for i = 1:length(vehRS_save)
    p_FRS = plotFilled(zonotope_slice(vehRS_save{i}{1}, [9;12], [u0;Au]), [1, 2], 'r');
    %     p2.FaceAlpha = 0.25;plotFilled(vehRS_save{i}{1}, [1, 2], 'g');
    p_FRS.FaceAlpha = 0.01;
    p_FRS.EdgeAlpha = 0.4;
    %         vehRS_save{end+1} = vehRS_save{i};
end
drawnow
