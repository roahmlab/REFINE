%1 lane change manuver
%  brake after lane change until split
%  brake from 1-0 m/s

%2 speed change manuver
%  brake after speed change until split
%  brake from 1-0 m/s

%save index of of shortened zonotope series as brake_idx, low_idx
%save speed center and generator at braking idx, and low idx 
clear;

perr = load('lateral_prediction_error.mat');
previous_error_flag = 0;

lon_err_gen = max([perr.lon_err_dir_max; perr.lon_err_lan_max ;abs(perr.lon_err_dir_min); abs(perr.lon_err_lan_max)],[],1);
lon_err_one_half_sec = lon_err_gen(60);
lat_err_gen = max([perr.lat_err_dir_max; perr.lat_err_lan_max ;abs(perr.lat_err_dir_min); abs(perr.lat_err_lan_max)],[],1);
lat_err_one_half_sec = lat_err_gen(60);
% head_err_gen = max([perr.head_err_dir_max; perr.head_err_lan_max ;abs(perr.head_err_dir_min); abs(perr.head_err_lan_max)],[],1);
head_err_one_half_sec = deg2rad(3.19);

t0_arr = [0 1.5];
for t0_idx = 1:length(t0_arr)
    t0= t0_arr(t0_idx);
dim = 14;

options.tensorParallel = 0;%turn this to 1 if want to use parallel not working very well. 
% set options for reachability analysis:
options.taylorTerms=15; % number of taylor terms for reachable sets
options.zonotopeOrder= 100; % zonotope order... increase this for more complicated systems.
options.maxError = 1e100*ones(dim, 1); % our zonotopes shouldn't be "splitting", so this term doesn't matter for now
options.verbose = 1;
options.uTrans = 0; % we won't be using any inputs, as traj. params specify trajectories
options.U = zonotope([0, 0]);
options.advancedLinErrorComp = 0;
% options.intermediateOrder = 20; if 
options.tensorOrder = 2;
options.errorOrder = 50;
options.reductionInterval = inf;
options.reductionTechnique = 'girard';

options.tStart = 0;
load_const
%in2 states: x, y, psi, u, v, r,u0,v0, r0,t0 ,Au,   Ay,   t
%            1  2  3    4  5  6  7  8  9  10  11    12    13
% vehicle parameters
options.x0 = zeros(dim,1);
frs_gen_starttic = tic;

 load lane_change_Ay_info
 
 u0_gen = 0.05
for u0 = u0_vec % set range of Ay or del_y allowed
     [~,u0_idx ]=min(abs(u0_vec -u0)); 
    del_y_arr = linspace(0,Ay_vec(u0_idx),2*num_Ay+1);%    del_y_arr = -0.8:0.2:0;
    delyspd = del_y_arr(2) - del_y_arr(1);
    del_y_arr = del_y_arr(2:2:end);
%     delyspd = 0.3;
%     r0v0_gen = 0.1;
%     r0v0_gen = Ay_vec(u0_idx);
%     del_y_arr = [0.3];
    for del_y_idx = 1:length(del_y_arr)
        del_y = del_y_arr(del_y_idx);
        
        u0, del_y
        options.x0 = zeros(dim,1);
        options.tFinal = tpk;
        options.timeStep = 0.005;
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

        delx0 = zeros(dim,1);delx0(1) = lon_err_one_half_sec*previous_error_flag;
        dely0 = zeros(dim,1);dely0(2) = lat_err_one_half_sec*previous_error_flag;
        delh0 = zeros(dim,1);delh0(3) = head_err_one_half_sec*previous_error_flag;
        % delAu = zeros(dim,1); delAu(12) = 0;
        delAy = zeros(dim,1); delAy(12) = delyspd;
        delu0 = zeros(dim,1); delu0(4) = u0_gen; delu0(7) = u0_gen; delu0(11) = u0_gen;
%         delAu = zeros(dim,1); delAu(11) = 1; %this is ignored during lane change, but is helpful during braking
        delv0 = zeros(dim,1); delv0(5) = v0_limit_gen(t0_idx,del_y_idx,u0_idx); delv0(8) = v0_limit_gen(t0_idx,del_y_idx,u0_idx);
        delr0 = zeros(dim,1); delr0(6) = r0_limit_gen(t0_idx,del_y_idx,u0_idx); delr0(9) = r0_limit_gen(t0_idx,del_y_idx,u0_idx);
%         delt0 = zeros(dim,1); delt0(12) = 0;
        options.R0 = zonotope( [options.x0, delx0,dely0,delh0,delu0, delAy,delv0,delr0]);
        tic;
        
%         if u0 > 5
                sys = nonlinearSys(dim, 1, @dyn_y_change, options);
%             else
%                 sys = nonlinearSys(dim, 1, @dyn_y_change_u5, options);
%         end
            
        [vehRS,Rt] = reach(sys, options);
        figure(1); clf; hold on; axis equal;
        for i = 1:10:length(vehRS)
            p_FRS = plot(vehRS{i}{1}, [1, 2], 'g');

        end
        yline(-0.5);yline(1.5);
        carbody = make_box([0.5,0.2]);
        plot(carbody(1,:),carbody(2,:));
        saveas(gcf,'r_ctrl_zono_0.001.pdf')
        %% first part of brake (we want to brake to 5m/s first with acceleration -4 m/s2)
%         if u0>5
            % perform the first part of brake
            brake_time = (u0- u_really_slow )/amax;


            tbrake1 = tpk;
            t_prev = zeros(dim,1); t_prev(end)=tpk;%assume it complete the whole time
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


            options.tFinal = 0.47;
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

            for i = 1:length(vehBrk)
                if size(vehBrk{i},2)~= 1
                    break
                end
                p_FRS = plot(vehBrk{i}{1}, [1, 2], 'r');
            end
%         end
       %% Sean's low speed model
        % usually the first braking maneuver has already brake the
        % vehicle below 6m/s and very close to 5m/s, but let's check if
        % the speed is around 5m/s anyway.

        JL_WARNING = 0;

        % set initial condition
%         if u0 > 5
%             frs = interval(Rt_brk{end}{1}.set);
%             if supremum(frs(4))>5.3
%                 warning('first part of brake should perform longer!!!!!!!!!!!');
%                 JL_WARNING = 1;
%             end
%             frs_c = mid(frs);
%             t_prev = zeros(dim,1);t_prev(end) = frs_c(end); t_tmep(13) = frs_c(13);
%             tbrake2 = tpk + brake_time;
%             options.R0 = deleteZeros(Rt_brk{i}{1}.set) - t_prev;
%             options.x0 = center(options.R0);    
%         else
%             tbrake2 = tpk;
%             t_prev = zeros(dim,1); t_prev(end)=tpk;%assume it complete the whole time
%             options.R0 = deleteZeros(Rt{end}{1}.set)- t_prev;
%             options.x0 = center(options.R0);
%         end



        % JL NEW idea: rotate R0 first in case it split
%         th0 = options.x0(3);
%         xy0 = options.x0(1:2);
%         options.R0 = ROTxy(options.R0, -th0);
%         options.x0 = center(options.R0); 
% 
% 
%         options.tFinal = 2;
%         sys = nonlinearSys(dim, 1, @dyn_brake_low, options );
%         [vehBrk_low,Rt_brk_low] = reach(sys, options);
% 
%         tComp = toc;
%         disp(['computation time of reachable set: ', num2str(tComp)]);
%         toc(frs_gen_starttic)
% 
% 
%         % JL NEW idea continue: rotate back everything, and shift time
%         for i = 1:length(vehBrk_low)
%             vehBrk_low{i}{1} = ROTxy(vehBrk_low{i}{1},th0,xy0);
%             Rt_brk_low{i}{1}.set = ROTxy(Rt_brk_low{i}{1}.set,th0,xy0);
% 
%             temp = zeros(dim,1); temp(end) = tbrake2;
%             vehBrk_low{i}{1} = vehBrk_low{i}{1} + temp;
%             Rt_brk_low{i}{1}.set = Rt_brk_low{i}{1}.set + temp;
%         end
% 
%         for i = 1:10:length(vehBrk_low)
%             if size(vehBrk_low{i},2)~= 1
%                 break
%             end
%             p_FRS = plot(vehBrk_low{i}{1}, [1, 2], 'k');
%         end

        %% gather all FRSs and enclose adjacent FRSs (vehRS_save is re-generated)
%         if u0>5
%             temp = [vehRS; vehBrk;vehBrk_low];
%         else
%             temp = [vehRS; vehBrk_low];
%         end
%         vehRS_save = cell(0,0);
%         ENCLOSE = 0;
%         slice_dim = [7,8,9,12];
%         figure(10), clf, hold on, axis equal
%         for i = 1:length(temp)
%             if ENCLOSE == 0
%                 ENCLOSE = 1;
%             else
%                 ENCLOSE = 0;
%                 vehRS_save{end+1} = deleteAligned(deleteZeros(enclose(temp{i-1}{1},temp{i-1}{1})),slice_dim);
%                 plot(vehRS_save{end},[1,2],'b');
%             end
%         end
%         
        % check if length(temp) is odd
%         if mod(length(temp),2)==1
%             vehRS_save{end+1} = deleteAligned(temp{end}{1},slice_dim);
%         end
        
        save(".\data\lane_change\lane_change_t0="+num2str(t0)+"_u="+num2str(u0)+"_Ay="+num2str(del_y_idx)+","+num2str(del_y)+".mat",'vehRS_save','JL_WARNING');
        
    end
end
end