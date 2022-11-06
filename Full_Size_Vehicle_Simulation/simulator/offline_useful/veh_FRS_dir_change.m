% Use CORA to propogat dynamics forward using the dynamcis generated in veh
%_dyn_gen_new.m
%save index of of shortened zonotope series as brake_idx, low_idx
%save speed center and generator at braking idx, and low idx 
clear;
t0_arr = 0:0.75:3-0.75;
for t0_idx = 1:length(t0_arr)
    t0= t0_arr(t0_idx);
dim = 14;

options.tensorParallel = 0;%turn this to 1 if want to use parallel not working very well. 
% set options for reachability analysis:
options.taylorTerms=15; % number of taylor terms for reachable sets
options.zonotopeOrder= 100; % zonotope order... increase this for more complicated systems.
options.maxError =1e100*ones(dim, 1); % our zonotopes shouldn't be "splitting", so this term doesn't matter for now
options.verbose = 0;
options.uTrans = 0; % we won't be using any inputs, as traj. params specify trajectories
options.U = zonotope([0, 0]);
options.advancedLinErrorComp = 0;
options.tensorOrder = 2;
options.errorOrder = 50;
options.reductionInterval = inf;
options.reductionTechnique = 'girard';
options.verbose = 0;
options.tStart = 0;
load_const
%in2 states: x, y, psi, u, v, r,u0,v0, r0,t0 ,Au,   Ay,   t
%            1  2  3    4  5  6  7  8  9  10  11    12    13
% vehicle parameters
options.x0 = zeros(dim,1);
frs_gen_starttic = tic;

% load dir_change_Ay_info
for u0 = 1 % set range of Ay or del_y allowed
%     [~,u0_idx ]=min(abs(u0_vec -u0)); 
%     del_y_arr = linspace(0,Ay_vec(u0_idx),9);%    del_y_arr = -0.8:0.2:0;
%     del_y_arr = del_y_arr(2:2:end);
%     delyspd = 0.5*(del_y_arr(2) - del_y_arr(1));
%     r0v0_gen = Ay_vec(u0_idx);
    del_y_arr = 0.2;
    
    for del_y_idx = 1
        del_y = del_y_arr(del_y_idx);
        
        u0, del_y
        options.x0 = zeros(dim,1);
        options.tFinal = tpk_dir;
        options.timeStep = 0.01;
        options.x0(4) = u0;
        options.x0(5) = v0_limit_c(t0_idx,del_y_idx,u0_idx);
        options.x0(8) = v0_limit_c(t0_idx,del_y_idx,u0_idx);
        options.x0(6) = r0_limit_c(t0_idx,del_y_idx,u0_idx);
        options.x0(9) = r0_limit_c(t0_idx,del_y_idx,u0_idx);
        options.x0(7) = u0;
        options.x0(11) = u0; %Au = u0
        options.x0(10) = t0; %set t0 = 0.5;
        options.x0(12) = del_y;
        % options.R0 = zonotope( [options.x0, 0.1*diag([1, 1, 2, 2, 0.5, 0, 0])] );
        % options.R0 = zonotope( [options.x0, [0;0;0.2;0;0;0;], [0;0;0;1;0;0;], [0;0;0;0;0.01;0;]] );
        % options.R0 = zonotope( [options.x0, 1e-2*[0;0;0;0;0;1;0]] );\
        delu0 = zeros(dim,1);delu0(4) = 1; delu0(7) = 1; delu0(11) = 1;
        % delAu = zeros(dim,1); delAu(12) = 0;
        delAy = zeros(dim,1); delAy(12) = delyspd;
%         delAu = zeros(dim,1); delAu(11) = 1; %this is ignored during lane change, but is helpful during braking
        delv0 = zeros(dim,1); delv0(5) = v0_limit_gen(t0_idx,del_y_idx,u0_idx); delv0(8) = v0_limit_gen(t0_idx,del_y_idx,u0_idx);
        delr0 = zeros(dim,1); delr0(6) = r0_limit_gen(t0_idx,del_y_idx,u0_idx); delr0(9) = r0_limit_gen(t0_idx,del_y_idx,u0_idx);
%          delt0 = zeros(dim,1); delt0(12) = 0;
        options.R0 = zonotope( [options.x0, delu0,delAy,delv0,delr0]);
        tic;
        
        if u0 > 5
                sys = nonlinearSys(dim, 1, @dyn_dir_change, options);
            else
                sys = nonlinearSys(dim, 1, @dyn_dir_change_u5, options);
        end
        
        [vehRS,Rt] = reach(sys, options);
        figure(1); clf; hold on; axis equal;
        for i = 1:10:length(vehRS)
            p_FRS = plot(vehRS{i}{1}, [1, 2], 'g');
        end
        %% first part of brake (we want to brake to 5m/s first with acceleration -4 m/s2)
        if u0>5
            % perform the first part of brake
            acc = 4; % acceleration of speed
            brake_time = (u0+1-5)/acc;


            tbrake1 = tpk_dir;
            t_prev = zeros(dim,1); t_prev(end)=tpk_dir;%assume it complete the whole time
            options.R0 = deleteZeros(Rt{end}{1}.set)- t_prev;
            Z = options.R0.Z;
            Z(13,1) = brake_time;
            options.R0 = zonotope(Z);
            options.x0 = center(options.R0);

            % rotate R0
            th0 = options.x0(3);
            xy0 = options.x0(1:2);
            options.R0 = ROTxy(options.R0, -th0);
            options.x0 = center(options.R0); 


            options.tFinal = brake_time;
            sys = nonlinearSys(dim, 1, @dyn_brake_high, options );
            [vehBrk,Rt_brk] = reach(sys, options);

            % rotate everything back, and shift time
            for i = 1:length(vehBrk)
                vehBrk{i}{1} = ROTxy(vehBrk{i}{1},th0,xy0);
                Rt_brk{i}{1}.set = ROTxy(Rt_brk{i}{1}.set,th0,xy0);

                temp = zeros(dim,1); temp(end) = tbrake1;
                vehBrk{i}{1} = vehBrk{i}{1} + temp;
                Rt_brk{i}{1}.set = Rt_brk{i}{1}.set + temp;
            end

            tComp = toc;
            disp(['computation time of reachable set: ', num2str(tComp)]);
            toc(frs_gen_starttic)
            %% plot reachable set

            for i = 1:10:length(vehBrk)
                if size(vehBrk{i},2)~= 1
                    break
                end
                p_FRS = plot(vehBrk{i}{1}, [1, 2], 'r');
            end
        end
       %% Sean's low speed model
        % usually the first braking maneuver has already brake the
        % vehicle below 6m/s and very close to 5m/s, but let's check if
        % the speed is around 5m/s anyway.

        JL_WARNING = 0;

        % set initial condition
        if u0 > 5
            frs = interval(Rt_brk{end}{1}.set);
            if supremum(frs(4))>6
                error('first part of brake should perform longer!!!!!!!!!!!');
                JL_WARNING = 1;
            end
            frs_c = mid(frs);
            t_prev = zeros(dim,1);t_prev(end) = frs_c(end); t_tmep(13) = frs_c(13);
            tbrake2 = tpk_dir + brake_time;
            options.R0 = deleteZeros(Rt_brk{i}{1}.set) - t_prev;
            options.x0 = center(options.R0);    
        else
            tbrake2 = tpk_dir;
            t_prev = zeros(dim,1); t_prev(end)=tpk_dir;%assume it complete the whole time
            options.R0 = deleteZeros(Rt{end}{1}.set)- t_prev;
            options.x0 = center(options.R0);
        end



        % JL NEW idea: rotate R0 first in case it split
        th0 = options.x0(3);
        xy0 = options.x0(1:2);
        options.R0 = ROTxy(options.R0, -th0);
        options.x0 = center(options.R0); 


        options.tFinal = 2;
        sys = nonlinearSys(dim, 1, @dyn_brake_low, options );
        [vehBrk_low,Rt_brk_low] = reach(sys, options);

        tComp = toc;
        disp(['computation time of reachable set: ', num2str(tComp)]);
        toc(frs_gen_starttic)


        % JL NEW idea continue: rotate back everything, and shift time
        for i = 1:length(vehBrk_low)
            vehBrk_low{i}{1} = ROTxy(vehBrk_low{i}{1},th0,xy0);
            Rt_brk_low{i}{1}.set = ROTxy(Rt_brk_low{i}{1}.set,th0,xy0);

            temp = zeros(dim,1); temp(end) = tbrake2;
            vehBrk_low{i}{1} = vehBrk_low{i}{1} + temp;
            Rt_brk_low{i}{1}.set = Rt_brk_low{i}{1}.set + temp;
        end

        for i = 1:10:length(vehBrk_low)
            if size(vehBrk_low{i},2)~= 1
                break
            end
            p_FRS = plot(vehBrk_low{i}{1}, [1, 2], 'k');
        end

        %% gather all FRSs and enclose adjacent FRSs (vehRS_save is re-generated)
        if u0>5
            temp = [vehRS; vehBrk;vehBrk_low];
        else
            temp = [vehRS; vehBrk_low];
        end
        vehRS_save = cell(0,0);
        ENCLOSE = 0;
        slice_dim = [7,8,9,12];
        figure(10), clf, hold on, axis equal
        for i = 1:length(temp)
            if ENCLOSE == 0
                ENCLOSE = 1;
            else
                ENCLOSE = 0;
                vehRS_save{end+1} = deleteAligned(deleteZeros(enclose(temp{i-1}{1},temp{i-1}{1})),slice_dim);
                plot(vehRS_save{end},[1,2],'b');
            end
        end
        
        % check if length(temp) is odd
        if mod(length(temp),2)==1
            vehRS_save{end+1} = deleteAligned(temp{end}{1},slice_dim);
        end
        
        
        save(".\data\dir_change\dir_change_t0="+num2str(t0)+"_u="+num2str(u0)+"_Ay="+num2str(del_y_idx)+","+num2str(del_y)+".mat",'vehRS_save','JL_WARNING');
        
    end
end
end
%% Inspect FRS % Uncomment the following to inspect result
% % First load the desired FRS in the FRS folder and run. 
%  figure(1); clf;hold on;
% % for i = 1:length(vehRS_save)
% %     p_FRS = plotFilled(vehRS_save{i}{1}, [1, 2], 'g');
% %     p_FRS.FaceAlpha = 0.02;
% %     p_FRS.EdgeAlpha = 0.4;
% %     %         vehRS_save{end+1} = vehRS_save{i};
% % end
% load("lane_spd_change_t0=0_u=27_Au=23_Ay=4,0.050449.mat"); % I only have this one in the repo, genearte your own will take 90 seconds each
% dim = 14;
% load_const;
% u0 = 27;
% Au = 23;
% t0 = 0;
% del_y =0.0504;
% t = linspace(0, tpk_dir+Au/amax);
% syms_flag = 0;
% [T_go,U_go,Z_go] = gaussian_T_parameterized_traj_with_brake(t0,del_y,Au,u0,t,syms_flag,1);
% % A_go = highway_cruising_6_state_agent('use_rover_controller',false);
% % v0 = 0;
% % r0 = 0;
% % 
% % z_0 = [0;0;0;u0;v0;r0] ;
% % A_go.reset(z_0) ;
% % A_go.move(T_go(end),T_go,U_go) ;
% % plot(Z_go(1,:), Z_go(2,:));
% % A_go.plot();
% %in2 states: x, y, psi, u, v, r,u0,v0, r0,t0 ,Au,   Ay,   t
% %            1  2  3    4  5  6  7  8  9  10  11    12    13
% 
% for i = 1:length(vehRS_save) %change to 65 if only want to plot to peak
%     zonoslice =zonotope_slice(vehRS_save{i}, [7;8;9;11;12], [u0;v0;r0;Au;del_y]);
%     zonoslice = deleteAligned(zonoslice);
%     
%     p_FRS = plotFilled(zonoslice, [1, 2], 'r');
%     p_FRS.FaceAlpha = .05;
%     p_FRS.EdgeAlpha = 0.01;
%     
%     ft = deleteAligned(project(vehRS_save{i},3));
%     headingc = center(ft);
%     headingg = abs(generators(ft));
%     if isempty(headingg)
%         headingg = 0;
%     end
%     gen = [2.4 1];
%     [len , width]=get_footprint_gen(gen,headingg);% make the footprint wiggle to get largest footprint
%     h =  headingc;
%     ego_gen = [[cos(h)*len; sin(h)*len], [sin(-h)*width; cos(-h)*width]];
%     gen = zeros(dim,2);gen(1:2,1:2) = ego_gen;
%     ftprint_zono  = zonotope([zeros(dim,1), gen]);
%     slicewithft = zonoslice+ftprint_zono;
%     p3 = plotFilled(slicewithft, [1, 2], 'g');
%     p3.FaceAlpha = 0.01;
% end
% axis equal
% xlim([-5,130])
% 
% 
% yline(2,'LineWidth',2)
% yline(6,'LineWidth',2)
% yline(-2,'LineWidth',2)
% drawnow
% 
% return
