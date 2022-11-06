clear;close all
previous_error_flag = 1;
load dir_change_Ay_info
load_const

get_t0end_error;

t0_arr = t0_dt*((1:tpk_dir/t0_dt)-1);
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
    %in2 states: x, y, psi, u, v, r,u0,v0, r0,t0 ,Au,   Ay,   t
    %            1  2  3    4  5  6  7  8  9  10  11    12    13
    % vehicle parameters
    options.x0 = zeros(dim,1);
    frs_gen_starttic = tic;
    
    u0_gen = 0.5*(u0_vec(2)-u0_vec(1));
    
    for u0 = u0_vec % set range of Ay or del_y allowed
        [~,u0_idx ]=min(abs(u0_vec -u0));
        
        %     delyspd = 0.3;
        %     r0v0_gen = 0.1;
        %     r0v0_gen = Ay_vec(u0_idx);
        %     del_y_arr = [0.3];
        for Au_idx = 1:length(u0_vec)
            del_y_idx = 1; %constant
%             del_y = del_y_arr(del_y_idx);
            Au = u0_vec(Au_idx);
            u0,Au 
            options.x0 = zeros(dim,1);
            options.tFinal = tpk_dir-t0;
            options.timeStep = 0.005;
            options.x0(4) = u0;
            options.x0(5) = v0_limit_c(t0_idx,del_y_idx,u0_idx);
            options.x0(8) = v0_limit_c(t0_idx,del_y_idx,u0_idx);
            options.x0(6) = r0_limit_c(t0_idx,del_y_idx,u0_idx);
            options.x0(9) = r0_limit_c(t0_idx,del_y_idx,u0_idx);
            options.x0(7) = u0;
            options.x0(11) = Au; 
            options.x0(10) = t0; %set t0 = 0.5;
            options.x0(12) = 0;
            % options.R0 = zonotope( [options.x0, 0.1*diag([1, 1, 2, 2, 0.5, 0, 0])] );
            % options.R0 = zonotope( [options.x0, [0;0;0.2;0;0;0;], [0;0;0;1;0;0;], [0;0;0;0;0.01;0;]] );
            % options.R0 = zonotope( [options.x0, 1e-2*[0;0;0;0;0;1;0]] );\
            
            delx0 = zeros(dim,1);delx0(1) = lon_err_one_half_sec*previous_error_flag;
            dely0 = zeros(dim,1);dely0(2) = lat_err_one_half_sec*previous_error_flag;
            delh0 = zeros(dim,1);delh0(3) = head_err_one_half_sec*previous_error_flag;
            % delAu = zeros(dim,1); delAu(12) = 0;
%             delAy = zeros(dim,1); delAy(12) = delyspd;
            delu0 = zeros(dim,1); delu0(4) = u0_gen; delu0(7) = u0_gen; 
            delAu = zeros(dim,1); delAu(11) = u0_gen;
            %         delAu = zeros(dim,1); delAu(11) = 1; %this is ignored during lane change, but is helpful during braking
            delv0 = zeros(dim,1); delv0(5) = v0_limit_gen(t0_idx,del_y_idx,u0_idx); delv0(8) = v0_limit_gen(t0_idx,del_y_idx,u0_idx);
            delr0 = zeros(dim,1); delr0(6) = r0_limit_gen(t0_idx,del_y_idx,u0_idx); delr0(9) = r0_limit_gen(t0_idx,del_y_idx,u0_idx);
            %         delt0 = zeros(dim,1); delt0(12) = 0;
            options.R0 = zonotope( [options.x0, delx0,dely0,delh0,delu0, delAu,delv0,delr0]);
            tic;
            
            %         if u0 > 5
            sys = nonlinearSys(dim, 1, @dyn_u_change, options);
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
            brake_time = (u0- u_really_slow )/amax;% + tbrk2;
            
            
            %             tbrake1 = tpk;
            t_prev = zeros(dim,1); t_prev(end)=tpk_dir-t0;%assume it complete the whole time
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
            
            
            options.tFinal = brake_time + tbrk2;
            sys = nonlinearSys(dim, 1, @dyn_u_brake, options );
            [vehBrk,Rt_brk] = reach(sys, options);
            
            
            % rotate everything back, and shift time
            for i = 1:length(vehBrk)
                vehBrk{i}{1} = ROTxy(vehBrk{i}{1},th0,xy0);
                Rt_brk{i}{1}.set = ROTxy(Rt_brk{i}{1}.set,th0,xy0);

                temp = zeros(dim,1); temp(end) = tpk_dir-t0;
                vehBrk{i}{1} = vehBrk{i}{1} + temp;
                Rt_brk{i}{1}.set = Rt_brk{i}{1}.set + temp;
            end

            for i = 1:10:length(vehBrk)
                if size(vehBrk{i},2)~= 1
                    break
                end
                p_FRS = plot(vehBrk{i}{1}, [1, 2], 'r');
            end
            drawnow
            %         JL_WARNING = 0;
            
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
            temp = [vehRS; vehBrk];
            %         end
            vehRS_save = cell(0,0);
            ENCLOSE = 0;
            slice_dim = [7,8,9,12];
            figure(10), clf, hold on, axis equal
            for i = 1:10:length(temp)
                if ENCLOSE == 0
                    ENCLOSE = 1;
                else
                    ENCLOSE = 0;
                    vehRS_save{end+1} = deleteAligned(deleteZeros(temp{i}{1}),slice_dim);
                    plot(vehRS_save{end},[1,2],'b');
                end
            end
            drawnow
            %
            %         check if length(temp) is odd
            %             if mod(length(temp),2)==1
            %                 vehRS_save{end+1} = deleteAligned(temp{end}{1},slice_dim);
            %             end
            if previous_error_flag
                save("./data/spd_change/various_initial_condition/spd_change_t0="+num2str(t0)+"_u="+num2str(u0)+"_Au="+num2str(Au)+","+num2str(Au_idx)+".mat",'vehRS_save');
            else
                save("./data/spd_change/zero_initial_condition/spd_change_t0="+num2str(t0)+"_u="+num2str(u0)+"_Au="+num2str(Au)+","+num2str(Au_idx)+".mat",'vehRS_save');
            end
        end
    end
end